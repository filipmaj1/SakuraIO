//For EEPROM
#include <arduino.h>
#include <avr/eeprom.h>

//USB
#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

//Sakura Stuff
#include "sakEEPROM.h"
#include "GenericHIDParser.h"

//Debug Flag (Adds/Removes log code)
#if 1
#define DEBUG
#include <AltSoftSerial.h>
AltSoftSerial debugSerial;
char debugBuffer[100];
#define DebugLog(...) sprintf_P(debugBuffer, __VA_ARGS__); debugSerial.print(debugBuffer)

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')
#else
#define DebugLog(...)
#endif

///////////////
//CONFIGURATION
///////////////

//Pin Assignments
#define PIN_JVS_DIR     2
#define PIN_BUS_TERM    3
#define PIN_SENSE_OUT   4
#define PIN_SENSE_IN    A0

#define JVS_TX          HIGH
#define JVS_RX          LOW

//Identification
const char IDENTIFICATION[] PROGMEM = {"Fragmenter Works;Sakura I/O;v0.01;By Filip Maj\0"};
#define VERSION_CMD   0x13
#define VERSION_JVS   0x30
#define VERSION_COM   0x10

//Capabilities of this I/O device
const byte test_functions[] PROGMEM = {
  0x01, 0x02, 0x0C, 0x00, //Two Players, 12 Buttons each
  0x02, 0x02, 0x00, 0x00, //Two Coin slots
  0x00         //Terminator
};

#define JVS_TIMEOUT 15

///////////////
//JVS CONSTANTS
///////////////

//Special Bytes
#define SYNC          0xE0
#define ESCAPE        0xD0
#define BROADCAST     0xFF

//Command Codes
enum : byte {
  //Address Setup
  OP_BUS_RESET          = 0xF0,
  OP_SET_ADDR           = 0xF1,
  OP_SET_COMM           = 0xF2,

  //Initialization
  OP_GET_IO_ID          = 0x10,
  OP_GET_CMD_VER        = 0x11,
  OP_GET_JVS_VER        = 0x12,
  OP_GET_COM_VER        = 0x13,
  OP_GET_FUNCTIONS      = 0x14,
  OP_MAIN_ID            = 0x15,

  //Input
  OP_INPUT_DIGITAL      = 0x20,
  OP_INPUT_COINS        = 0x21,
  OP_INPUT_ANALOG       = 0x22,
  OP_INPUT_ROTARY       = 0x23,
  OP_INPUT_KEYBOARD     = 0x24,
  OP_INPUT_SCREENPOS    = 0x25,
  OP_INPUT_GENERAL      = 0x26,

  //Output
  OP_REMAINING_PAYOUT   = 0x2E,
  OP_DATA_RESEND        = 0x2F,
  OP_OUTPUT_SUB_COINS   = 0x30,
  OP_OUTPUT_ADD_PAYOUT  = 0x31,
  OP_OUTPUT_GP1         = 0x32,
  OP_OUTPUT_ANALOG      = 0x33,
  OP_OUTPUT_CHARACTER   = 0x34,
  OP_OUTPUT_ADD_COINS   = 0x35,
  OP_OUTPUT_SUB_PAYOUT  = 0x36,
  OP_OUTPUT_GP2         = 0x37,
  OP_OUTPUT_GP3         = 0x38
};

enum : byte {
  STATUS_NORMAL         = 0x01,
  STATUS_CMD_UNKNOWN    = 0x02,
  STATUS_CHECKSUM_ERR   = 0x03,
  STATUS_OVERFLOW       = 0x04
};

enum : byte {
  REPORT_NORMAL         = 0x01,
  REPORT_ERR_NUMPARAMS  = 0x02,
  REPORT_ERR_PARAMS     = 0x03,
  REPORT_BUSY           = 0x04,
};

//////////////////
//Sakura Constants
//////////////////

enum : byte {
  STATE_UNKNOWN,
  STATE_RESET,
  STATE_READY
};

enum {
  SAK_CHECKSUM_FAIL = -1,
  SAK_UNKNOWN_CMD   = -2,
  SAK_RESEND        = -3,
  SAK_BUS_RESET     = -4,
  SAK_FALSE         = 0,
  SAK_TRUE          = 1
};

//Digital Switch Constants
#define SW_TEST     7
#define SW_TILT1    6
#define SW_TILT2    5
#define SW_TILT3    4

#define SW_START    15
#define SW_SERVICE  14
#define SW_UP       13
#define SW_DOWN     12
#define SW_LEFT     11
#define SW_RIGHT    10
#define SW_PUSH1    9
#define SW_PUSH2    9

#define SW_PUSH3    7
#define SW_PUSH4    6
#define SW_PUSH5    5
#define SW_PUSH6    4
#define SW_PUSH7    3
#define SW_PUSH8    2

//Extensions
#define SW_LEFT_UP     19
#define SW_LEFT_DOWN   18
#define SW_RIGHT_DOWN  17
#define SW_RIGHT_UP    16

//USB Constants
#define USB_TYPE_BUTTON 0
#define USB_TYPE_HAT_SW 1
#define USB_TYPE_AXIS   2

/////////
//GLOBALS
/////////

//USB Objects
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
void processUSB(USBHID* hid, bool isRpt, uint8_t len, uint8_t* buff);
GenericHIDParser genericHIDParser(&processUSB);

byte* currentMaps[2];
int   mapLength[2];

//Current Sakura I/O State
byte currentState = STATE_UNKNOWN;
byte currentAddress = 0xFF;

//Saved info about the last send packet in case of resend
byte resultBuffer[0xFF];
byte lastResultStatus = 0;
byte lastResultSize = 0;

//Arcade State (Input, Output, Coins)
byte systemSwitches = 0;
unsigned int playerSwitches[] = {0x0, 0x0};
byte coinStatus = 0x0;
short coinCounts[] = {10, 0};

#define setBit(val,nbit)   ((val) |=  (1<<(nbit)))
#define clearBit(val,nbit) ((val) &= ~(1<<(nbit)))


////////////////
//MAIN PROGRAM//
////////////////

/*~~~~~~~~~~~~PC-MAP STORAGE CODE~~~~~~~~~~~~~~~~~~*/

/*
   Map Storage:
     numMaps (1 byte)
     [Repeat *numMaps]
     UID (4 bytes)
     Offset (2 byte)

   Map Entry:
     name (15 bytes)
     mapSize (1 byte)
     mapData (x bytes)
*/

void addMap(unsigned long uid, char* mapName, const byte* mapData, byte mapDataSize) {
  byte numMappings;
  unsigned short lastMapOffset;
  byte lastMapSize;

  //Get the last offset, and add it's size for the total entry. Note: deleted maps MUST be followed by
  //a defragmentation or this won't work.
  sakEEPROM_Read(0, numMappings);
  sakEEPROM_Read(1 + ((numMappings - 1) * 6) + 4, lastMapOffset);
  sakEEPROM_Read(lastMapOffset + 0xF, lastMapSize);

  //Write out the new entry
  unsigned int newOffset = lastMapOffset + 0x10 + lastMapSize;
  int nameLength = strlen(mapName);
  sakEEPROM_WriteBytes(newOffset, mapName, nameLength <= 0xF ? nameLength : 0xF);
  sakEEPROM_Write(newOffset + 0xF, mapDataSize);
  sakEEPROM_WriteBytes(newOffset + 0x10, mapData, mapDataSize);

  //Write the header entry
  sakEEPROM_Write(1 + (numMappings * 6), uid);
  sakEEPROM_Write(1 + (numMappings * 6) + 4, newOffset);
}

void deleteMap(int index) {
  byte numMappings;
  sakEEPROM_Read(0, numMappings);

  //Copy each entry to the last spot
  for (byte i = numMappings - index; i < numMappings; i++) {
    unsigned long uid;
    unsigned int offset;

    sakEEPROM_Read(1 + (i * 6), uid);
    sakEEPROM_Read(1 + (i * 6) + 4, offset);
    sakEEPROM_Write(1 + ((i - 1) * 6), uid);
    sakEEPROM_Write(1 + ((i - 1) * 6) + 4, offset);
  }

  defragRom();
}

void defragRom() {

}

long loadMap(long uid, const byte* mapData) {
  byte numMappings;
  unsigned long readUID;
  unsigned int offset;
  byte mapSize;

  sakEEPROM_Read(0, numMappings);

  for (byte i = 0; i < numMappings; i++) {
    sakEEPROM_Read(1 + (i * 6), readUID);
    if (uid == readUID) {
      sakEEPROM_Read(1 + (i * 6) + 4, offset);
      sakEEPROM_Read(offset + 0xF, mapSize);
      sakEEPROM_ReadBytes(offset + 0x10, mapData, mapSize);
      return mapSize;
    }
  }

  return 0;
}

/*
   Quick and dirty protocol to communicate between the Sakura I/O and a PC. A `!` marker is
   used to sync the beginning of a packet. The structure looks like:
   Sync(!)    - 1 byte
   packetSize - 2 bytes
   opcode     - 1 byte
   checksum   - 1 byte
   mapdata    - variable

   Sakura I/O will respond with a result code, followed by the result code XORed with the sync code.
   If there is data to return, the data size (2 bytes) and the data will follow, ending with another
   result code and XORed version.

   Results: 'O' - OK, 'X' - Error, 'Y' - Checksum error, '?' - More data.
*/
void processMapManager() {
  byte packetBytes[0x100];
  char mapName[0xF];
  byte resultCode;
  unsigned int resultSize;

  if (Serial.available()) {
    byte huh = Serial.read();
    if (huh == '!') {
      byte sizeLO = Serial.read();
      byte sizeHI = Serial.read();

      unsigned int packetSize = sizeHI << 8 | sizeLO;
      byte opcode = Serial.read();
      byte checksum = Serial.read();

      Serial.readBytes(packetBytes, packetSize);

      //Checksum
      byte testChecksum = 0;
      for (int i = 0; i < packetSize; i++)
        testChecksum += packetBytes[i];

      if (checksum != testChecksum) {
        resultCode = 'Y';
        Serial.write(resultCode);
        Serial.write(resultCode ^ '!');
        return;
      }

      //Do Command
      switch (opcode) {
        //Ping
        case 0: {
            resultCode = 'O';
            break;
          }
        //Get Maps
        case 1: {

          }
        //Add Map
        case 2: {
            unsigned long uid = ((unsigned long*)&packetBytes)[0];
            memcpy(mapName, &packetBytes + 4, 0xF);
            byte mapDataSize = packetBytes[0x10];
            addMap(uid, mapName, (const byte*) &packetBytes + 0x10, mapDataSize);
            resultCode = 'O';
            break;
          }
        //Delete Map
        case 3: {
            unsigned long uid = ((unsigned long*)&packetBytes)[0];
            deleteMap(uid);
            resultCode = 'O';
            break;
          }
        //Get Info
        case 4: {
            memcpy(packetBytes, IDENTIFICATION, strlen(IDENTIFICATION));
            packetBytes[strlen(IDENTIFICATION) + 1] = 0;
            resultSize = strlen(IDENTIFICATION) + 1;
            resultCode = '?';
          }
        default:
          resultCode = 'X';
      }

      //Write out the result code and any data if needed.
      Serial.write(resultCode);
      Serial.write(resultCode ^ '!');
      if (resultCode == '?') {
        Serial.write(resultSize);
        Serial.write(packetBytes, resultSize);
        Serial.write('O');
        Serial.write('O' ^ '!');
      }
    }
  }
}

byte testMap[] = {USB_TYPE_BUTTON, 0x00, 1, //[Type][Bit Position][JVS Dest]
                  USB_TYPE_BUTTON, 0x01, 13,
                  USB_TYPE_BUTTON, 0x02, 12,
                  USB_TYPE_BUTTON, 0x03, 0,
                  USB_TYPE_BUTTON, 0x04, 14,
                  USB_TYPE_BUTTON, 0x05, 15,
                  USB_TYPE_BUTTON, 0x06, 10,
                  USB_TYPE_BUTTON, 0x07, 11,
                  USB_TYPE_BUTTON, 0x08, 6,
                  USB_TYPE_BUTTON, 0x09, 7,
                  USB_TYPE_BUTTON, 0x0B, 255,
                  USB_TYPE_HAT_SW, 0x10, 0b1111, 8, 0, 5, 1, 16, 2, 4, 3, 17, 4, 3, 5, 18, 6, 2, 7, 19 //[Type][Bit Position][Length Mask][Hat Maps][Map Pairs (Value,JVS Dest)]
                 };

/*~~~~~~~~~~~~~~~~~~USB CODE~~~~~~~~~~~~~~~~~~*/

/*
   Deals with processing the USB Host as well as loading map files.
*/
void processUSB(USBHID* hid, bool isRpt, uint8_t len, uint8_t* buff) {
  int playerIndex = 1;
  int mapIndex = 0;

  for (int i = 0; i < mapLength[mapIndex]; i++) {
    byte type = currentMaps[mapIndex][i++];
    byte bitPosition = currentMaps[mapIndex][i++];

    //Start Post
    byte bytePosition = bitPosition / 8;
    byte remainder = bitPosition % 8;

    //Button
    if (type == USB_TYPE_BUTTON) {
      byte jvsDest = currentMaps[mapIndex][i];

      //Unassigned (needed?)
      if (jvsDest == 0xFF)
        continue;

      bool value = (buff[bytePosition] >> remainder) & 1;

      if (value) {
        playerSwitches[playerIndex] |= (1 << jvsDest);
      }
      else
        playerSwitches[playerIndex] &= ~(1 << jvsDest);
    }
    //Tophat
    else if (type == USB_TYPE_HAT_SW) {
      byte valueMask = currentMaps[mapIndex][i++];
      byte innerMapLen = currentMaps[mapIndex][i++];

      uint32_t* intBuffer = (uint32_t*)(buff + bytePosition);
      int value = (intBuffer[0] >> remainder) & valueMask;

      playerSwitches[playerIndex] &= 0b1111111111000011;

      for (int j = 0; j < innerMapLen; j++) {
        //Found the value. Assign and set scanner to the end.
        if (currentMaps[mapIndex][i + (j * 2)] == value) {
          switch (currentMaps[mapIndex][i + (j * 2) + 1]) {
            case SW_LEFT_UP:
              playerSwitches[playerIndex] |= 0b0000000000100100;
              break;
            case SW_LEFT_DOWN:
              playerSwitches[playerIndex] |= 0b0000000000001100;
              break;
            case SW_RIGHT_UP:
              playerSwitches[playerIndex] |= 0b0000000000110000;
              break;
            case SW_RIGHT_DOWN:
              playerSwitches[playerIndex] |= 0b0000000000011000;
              break;
            default:
              playerSwitches[playerIndex] |= (1 << currentMaps[mapIndex][i + (j * 2) + 1]);
          }
        }
      }
      i += innerMapLen * 2;
    }
    //Analog
    else if (type == USB_TYPE_AXIS) {
      DebugLog(PSTR("ERROR: AXIS NOT SUPPORTED, aborting usb...\r\n"));
      return;
    }
  }

  //DebugLog(PSTR("Switches: %c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c\r\n"), BYTE_TO_BINARY((playerSwitches[playerIndex] >> 8) & 0xFF), BYTE_TO_BINARY(playerSwitches[playerIndex] & 0xFF));
}

/*~~~~~~~~~~~~~~~~~~JVS CODE~~~~~~~~~~~~~~~~~~*/

/*
   Blocking reads one byte from the serial link, handling the escape byte case.
*/
byte jvsReadByte() {
  while (!Serial.available());
  byte in = Serial.read();
  if (in == ESCAPE) {
    while (!Serial.available());
    in = Serial.read() + 1;
  }
  delayMicroseconds(10);
  return in;
}

/*
   The SENSE_IN pin is used during address assignment to check if this device can grab the broadcast
   address from the master device. Possible voltages are:
   5V - This is the last device in the daisy chain.
   2.5V - The next device has not received an address, keep ignoring the address broadcast.
   0V - The next device has had it's address assigned. This device can read the next broadcasted
   address.
*/
#define jvsGetSenseVoltage() analogRead(PIN_SENSE_IN) * (5.0f / 1023.0f)

/*
   Sets the SENSE_OUT pin for the next device in the chain, informing it that our address has been
   set. When addressSet is true, the pin voltage is 0V. Otherwise it emits 2.5V.
*/
#define jvsSenseHigh() digitalWrite(PIN_SENSE_OUT, LOW) //2.5v if address is not set.
#define jvsSenseLow() digitalWrite(PIN_SENSE_OUT, HIGH) //0v if address is set.

/*
   JVS standard uses half-duplex RS-485, so the direction of data must be set before transfering.
   Mode can be JVS_RX or JVS_TX.
*/
#define jvsSetDirectionTX() PORTD |= 0b00000100
#define jvsSetDirectionRX() PORTD &= 0b11111011

/*
   The two ends of a RS-485 daisy chain should be terminated with a 120Ohm resistor. When the SENSE_IN
   pin is reading 5V, this is the last device in the chain so turn on the terminator. Pin you connect
   to a ss-relay that bridges the resistor between A & B lines.
*/
#define jvsSetBusTerminator(isOn) digitalWrite(PIN_BUS_TERM, isOn)

/*
   Reads in the packet after the SYNC byte, verifying the checksum.
   Args: Data buffer, will store the packet if successful.
   Returns: Bytes read if successful. -1 if checksum failed.
*/
short rcvPacket(byte* dataBuffer) {
  //Check if it's for us
  byte targetNode = jvsReadByte();
  //DebugLog(PSTR("===PACKET===\r\n"));
  if (targetNode == BROADCAST || targetNode == currentAddress) {
    //Read data
    byte numBytes = jvsReadByte();

    Serial.readBytes(dataBuffer, numBytes);

    //Test checksum
    byte checksum = dataBuffer[numBytes - 1];
    byte testChecksum = targetNode + numBytes;
    for (byte i = 0; i < numBytes - 1; i++)
      testChecksum += dataBuffer[i];
    //DebugLog(PSTR("[NumBytes: 0x%x]\r\n"), numBytes);
    //DebugLog(PSTR("[Checksum: 0x%x][Test Checksum: 0x%x]\r\n"), checksum, testChecksum);

    if (checksum == testChecksum)
      return numBytes - 1;
    return SAK_CHECKSUM_FAIL;
  }
  DebugLog("\r\nIgnoring Packet.\r\n");
  return 0; //Not for us
}

/*
   Sends a result packet down the line. Payload version.
   Args: The resulting STATUS and it's payload.
*/
byte sendback[] = {0xE0, 0x00, 0x03, 0x1, 0x1, 0x5};
void sendResponse(byte statusCode, byte payloadSize) {

  payloadSize += 2;

  //Build Checksum
  byte checksum = payloadSize + statusCode;
  for (byte i = 0; i < payloadSize - 2; i++)
    checksum += resultBuffer[i];
  checksum %= 0xFF;

  //Write out
  jvsSetDirectionTX();
  Serial.write(SYNC);                           //SYNC Byte
  Serial.write(0x00);                           //Node Num (always 0)
  Serial.write(payloadSize);                    //Num Bytes
  Serial.write(statusCode);                     //Status
  for (int i = 0; i < payloadSize - 2; i++) {
    Serial.write(resultBuffer[i]);
    delayMicroseconds(50);
  }
  //Serial.write(resultBuffer, payloadSize - 2);  //Data
  Serial.write(checksum);                       //Checksum

  delayMicroseconds((payloadSize + 2) * 100);
  jvsSetDirectionRX();
}

//////////////////////////
//~~MAIN JVS CMD HANDLER~~
//////////////////////////
short parseCommand(const byte* packet, byte* readSize, byte* result, short* resultSize) {
  *readSize = 1;
  *resultSize = 1;

  switch (packet[0]) {
    //Address Setup
    case OP_BUS_RESET: {
        if (packet[1] == 0xD9) {
          currentState = STATE_RESET;
          currentAddress = BROADCAST;
          jvsSenseHigh();
          DebugLog(PSTR("Bus was reset. Setting SENSE_OUT to 2.5v.\r\n"));
          jvsSetDirectionRX();
          return SAK_BUS_RESET;
        }
      }
    case OP_SET_ADDR: {
        //Check sense line voltage. Address is only for us
        //if it's either 0v or 5v.
        float voltage = jvsGetSenseVoltage();
        if (voltage < 0.25f || voltage > 4.75f)
        {
          *readSize += 1;
          result[0] = REPORT_NORMAL;
          currentAddress = packet[1];
          currentState = STATE_READY;
          jvsSenseLow();
          DebugLog(PSTR("Address was set to: %d\r\n"), packet[1]);
          DebugLog(PSTR("Setting SENSE_OUT to 0v.\r\n"));
          return REPORT_NORMAL;
        }
        else {
          DebugLog(PSTR("Got address, but SENSE_IN is 2.5v. Ignoring.\r\n"));
          return SAK_BUS_RESET;
        }
      }
    //Initialization
    case OP_GET_IO_ID: {
        byte idSize = strlen_P(IDENTIFICATION);
        *resultSize = idSize + 2;
        result[0] = REPORT_NORMAL;
        memcpy_P(result + 1, IDENTIFICATION, idSize);
        result[idSize + 2] = 0;
        DebugLog(PSTR("Sent I/O ID: %S\r\n"), IDENTIFICATION);
        return REPORT_NORMAL;
      }
    case OP_GET_CMD_VER: {
        *resultSize = 2;
        result[0] = REPORT_NORMAL;
        result[1] = VERSION_CMD;
        DebugLog(PSTR("Sent Command Version: 0x%x\r\n"), VERSION_CMD);
        return REPORT_NORMAL;
      }
    case OP_GET_JVS_VER: {
        *resultSize = 2;
        result[0] = REPORT_NORMAL;
        result[1] = VERSION_JVS;
        DebugLog(PSTR("Sent JVS Version: 0x%x\r\n"), VERSION_JVS);
        return REPORT_NORMAL;
      }
    case OP_GET_COM_VER: {
        *resultSize = 2;
        result[0] = REPORT_NORMAL;
        result[1] = VERSION_COM;
        DebugLog(PSTR("Sent Communication Version: 0x%x\r\n"), VERSION_COM);
        return REPORT_NORMAL;
      }
    case OP_GET_FUNCTIONS: {
        *resultSize = 1 + 9;
        result[0] = REPORT_NORMAL;
        memcpy_P(result + 1, test_functions, 9);
        DebugLog(PSTR("Sent I/O Functions\r\n"));
        return REPORT_NORMAL;
      }
    case OP_MAIN_ID: {
        result[0] = REPORT_NORMAL;

        //Get end of string
        byte i;
        for (i = 0; i < 100; i++) {
          if (packet[i + 1] == 0x00)
            break;
        }
        if (i < 100)
          i++;
        *readSize += i;

        char mainID[100];
        memcpy(mainID, packet + 1, i);
        DebugLog(PSTR("Got Mainboard ID: %s\r\n"), mainID);
        return REPORT_NORMAL;
      }
    case OP_DATA_RESEND: {
        DebugLog(PSTR("Got a resend request\r\n"));
        return SAK_RESEND;
      }
    //Input
    case OP_INPUT_DIGITAL: {
        byte playerCount = packet[1];
        byte dataSize = packet[2];

        result[0] = REPORT_NORMAL;
        result[1] = systemSwitches;
        memcpy(result + 2, &playerSwitches, 2 * playerCount);

        *readSize = 2 + (playerCount * dataSize);
        *resultSize = 2 + (playerCount * dataSize);

        return REPORT_NORMAL;
      }
    case OP_INPUT_COINS: {
        DebugLog(PSTR("Coin Request\r\n"));
        byte slotCount = packet[1];
        result[0] = REPORT_NORMAL;
        for (int i = 0; i < slotCount; i++) {
          byte cStat = (coinStatus >> (i * 2)) & 3;
          byte r1 = (cStat << 6) | ((coinCounts[i] >> 8) & 0b00111111);
          byte r2 = coinCounts[i] & 0xFF;
          result[(i * 2) + 1] = r1;
          result[(i * 2) + 2] = r2;
        }

        *readSize = 1 + (2 * slotCount);
        *resultSize = 1 + (2 * slotCount);
        return REPORT_NORMAL;
      }
    //Output
    //If all else fails, unknown code error
    default:
      DebugLog(PSTR("Got unknown command: 0x%x\r\n"), packet[0]);
      return SAK_UNKNOWN_CMD;
  }
}

/*
   Deals with processing a single JVS packet at a time. The function waits
   for the SYNC byte, reads in the packet (checking checksum), and feeds
   each command into the command parser. Each result is collected and then
   sent in a response packet.
*/
long lastJVSMillis;
void processJVS() {
  byte dataBuffer[0xFF];
  long lastTime;

  //Wait for SYNC byte
  lastJVSMillis = millis();
  
  while (Serial.read() != SYNC) {
    //Timeout so it doesn't get lock execution.
    if (millis() - lastJVSMillis >= JVS_TIMEOUT)
      return;
  }

  short result = rcvPacket(dataBuffer);
  jvsSetDirectionTX();

  if (result == SAK_CHECKSUM_FAIL)
  {
    DebugLog(PSTR("Checksum failed.\r\n"));
    lastResultStatus = STATUS_CHECKSUM_ERR;
    sendResponse(STATUS_CHECKSUM_ERR, 0);
    return;
  }

  //Not for us
  if (!result)
    return;

  //Parse packet, building the response from every command
  byte commandSize = 0;
  byte commandIndex = 0;
  short resultSize = 0;
  short resultIndex = 0;
  while (commandIndex < result) {
    short reportCode = parseCommand(dataBuffer + commandIndex, &commandSize, resultBuffer + resultIndex, &resultSize);

    //Bus was reset
    if (reportCode == SAK_BUS_RESET) {
      return;
    }
    //Unknown command
    else if (reportCode == SAK_UNKNOWN_CMD) {
      lastResultStatus = STATUS_CMD_UNKNOWN;
      sendResponse(STATUS_NORMAL, resultIndex);
      return;
    }
    //Resend request
    else if (reportCode == SAK_RESEND) {
      sendResponse(lastResultStatus, lastResultSize);
      return;
    }

    commandIndex += commandSize;
    resultIndex += resultSize;

    if (resultIndex > 0xFF) {
      sendResponse(STATUS_OVERFLOW, REPORT_NORMAL);
      return;
    }
  }
  lastResultStatus = STATUS_NORMAL;
  lastResultSize = resultIndex;

  //Send the response back
  sendResponse(STATUS_NORMAL, resultIndex);
}

void setup() {
#ifdef DEBUG
  debugSerial.begin(9600);
#endif
  DebugLog(PSTR("Sakura I/O Board; a JVS to USB adapter.\r\n"));

  DebugLog(PSTR("Setting up pins...\r\n"));
  pinMode(PIN_JVS_DIR,    OUTPUT);
  pinMode(PIN_SENSE_OUT,  OUTPUT);
  pinMode(PIN_SENSE_IN,   INPUT_PULLUP);
  pinMode(PIN_BUS_TERM,   OUTPUT);

  DebugLog(PSTR("Starting JVS Serial Port...\r\n"));
  Serial.begin(115200);
  jvsSetDirectionRX();
  jvsSenseHigh();

  //Check if we are the last device. If so, turn on bus term.
  float voltage = jvsGetSenseVoltage();
  if (voltage > 4.75f) {
    jvsSetBusTerminator(true);
    DebugLog(PSTR("Last device in chain detected. Bus Terminator: ON\r\n"));
  }
  else
    jvsSetBusTerminator(false);

  //Start USB
  DebugLog(PSTR("Starting USB...\r\n"));
  if (Usb.Init() == -1) {
    DebugLog(PSTR("ERROR: OSC did not start.\r\n"));
  }
  else {
    DebugLog(PSTR("USB OSC started.\r\n"));
    Hid.SetReportParser(0, &genericHIDParser);
  }
  DebugLog(PSTR("Sakura I/O Initialized!\r\n"));

  currentMaps[0] = testMap;
  mapLength[0] = 45;
}

void loop() {
  while (true) {
    //Poll the USB devices
    Usb.Task();

    //Process a JVS Packet
    processJVS();

    //if (Serial.available()) {
    //  DebugLog(PSTR(" 0x%x\r\n"), jvsReadByte());
    //}
  }
}
