#include <arduino.h>
#include <SoftwareSerial.h>

#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

//Debug Flag (Adds/Removes log code)W
#if 0
#define DEBUG
#define DLOG(MSG) Serial.print(MSG)
#define DLOGLN(MSG) DLOG(MSG); DLOG("\r\n")
#else
#define DLOG(MSG)
#define DLOGLN(MSG)
#endif

///////////////
//CONFIGURATION
///////////////

//Pin Assignments
#define PIN_SERIAL_RX   0
#define PIN_SERIAL_TX   1

#define PIN_JVS_RX      2
#define PIN_JVS_TX      3
#define PIN_JVS_DIR     4

#define PIN_SENSE_OUT   5
#define PIN_SENSE_IN    A0

#define PIN_BUS_TERM    8

#define JVS_TX          1
#define JVS_RX          0

//Identification
const char IDENTIFICATION[] PROGMEM = {"Fragmenter Works;SAKURA I/O;Ver 1.0;By Filip Maj (Ioncannon)\0"};
#define VERSION_CMD   0x13
#define VERSION_JVS   0x30
#define VERSION_COM   0x10

//Capabilities of this I/O device
const byte test_functions[] PROGMEM = {
  0x01, 0x02, 0x0C, 0x00, //Two Players, 12 Buttons each
  0x02, 0x02, 0x00, 0x00, //Two Coin slots
  0x00         //Terminator
};

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

/////////
//GLOBALS
/////////

SoftwareSerial jvsSerial(PIN_JVS_RX, PIN_JVS_TX);
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

//Current Sakura I/O State
byte currentState = STATE_UNKNOWN;
byte currentAddress = 0xFF;

//Saved info about the last send packet in case of resend
byte resultBuffer[0xFF];
byte lastResultStatus = 0;
short lastResultSize = 0;

//Arcade State (Input, Output, Coins)
byte systemSwitches = 0;
short playerSwitches[] = {0x0, 0x0};
byte coinStatus = 0x0;
short coinCounts[] = {0, 0};

//////////////
//MAIN PROGRAM
//////////////

#define setBit(val,nbit)   ((val) |=  (1<<(nbit)))
#define clearBit(val,nbit) ((val) &= ~(1<<(nbit)))

/*
   Blocking reads one byte from the serial link, handling the escape byte case.
*/
byte jvsReadByte() {
  while (Serial.available() == 0);
  byte in = Serial.read();
  if (in == ESCAPE)
    in = Serial.read() + 1;
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
#define jvsSenseHigh() analogWrite(PIN_SENSE_OUT, 128.0f) //2.5v if address is not set.
#define jvsSenseLow() analogWrite(PIN_SENSE_OUT, 0.0f) //0v if address is set.

/*
   JVS standard uses half-duplex RS-485, so the direction of data must be set before transfering.
   Mode can be JVS_RX or JVS_TX.
*/
#define jvsSetDirection(mode) digitalWrite(PIN_JVS_DIR, mode)

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
  DLOG(F("[TargetNode: ")); DLOG(targetNode); DLOG(F("]"));
  if (targetNode == BROADCAST || targetNode == currentAddress) {
    DLOGLN(", packet is for us.");
    //Read data
    byte numBytes = jvsReadByte();

    DLOG(F("[NumBytes: ")); DLOG(numBytes) ;DLOG(F("]"));

    Serial.readBytes(dataBuffer, numBytes - 1);

    //Test checksum
    byte checksum = jvsReadByte();

    DLOG(F("[Checksum: ")); DLOG(checksum); DLOG(F("]"));

    byte testChecksum = targetNode + numBytes;
    for (byte i = 0; i < numBytes - 1; i++)
      testChecksum += dataBuffer[i];

    DLOG(F("[Test Checksum: ")); DLOG(testChecksum); DLOGLN(F("]"));

    if (checksum == testChecksum)
      return numBytes - 1;
    return SAK_CHECKSUM_FAIL;
  }
  DLOGLN("\r\nIgnoring Packet.");
  return 0; //Not for us
}

/*
   Sends a result packet down the line. Payload version.
   Args: The resulting STATUS and it's payload.
*/
void sendResponse(byte status, byte payloadSize) {
  //Build Checksum
  byte checksum = payloadSize + status;
  for (byte i = 0; i < payloadSize; i++)
    checksum += resultBuffer[i];
  checksum %= 0xFF;

  //Write out
  jvsSetDirection(JVS_TX);
  jvsSerial.write(SYNC);
  jvsSerial.write((byte)0x00);                 //Node Num (always 0)
  jvsSerial.write(payloadSize + 2);      //Num Bytes
  jvsSerial.write(status);               //Status
  jvsSerial.write(resultBuffer, payloadSize); //Data
  jvsSerial.write(checksum);             //Checksum
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
          DLOGLN(F("Bus was reset. Setting SENSE_OUT to 2.5v."));
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
          DLOG(F("Address was set to: ")); DLOGLN(packet[1]);
          DLOGLN(F("Setting SENSE_OUT to 0v."));
          return REPORT_NORMAL;
        }
        else {          
          DLOGLN(F("Got address, but SENSE_IN is 2.5v. Ignoring."));
          return SAK_BUS_RESET;
        }
      }
    //Initialization
    case OP_GET_IO_ID: {
        byte idSize = strlen(IDENTIFICATION) + 1;
        *resultSize = idSize + 1;
        result[0] = REPORT_NORMAL;
        memcpy(result + 1, IDENTIFICATION, idSize);
        DLOG(F("Sent I/O ID: ")); DLOGLN(IDENTIFICATION);
        return REPORT_NORMAL;
      }
    case OP_GET_CMD_VER: {
        result[0] = REPORT_NORMAL;
        result[1] = VERSION_CMD;
        DLOG(F("Sent Command Version: ")); DLOGLN(VERSION_CMD);
        return REPORT_NORMAL;
      }
    case OP_GET_JVS_VER: {
        result[0] = REPORT_NORMAL;
        result[1] = VERSION_JVS;
        DLOG(F("Sent JVS Version: ")); DLOGLN(VERSION_JVS);
        return REPORT_NORMAL;
      }
    case OP_GET_COM_VER: {
        result[0] = REPORT_NORMAL;
        result[1] = VERSION_COM;
        DLOG(F("Sent Communication Version: ")); DLOGLN(VERSION_COM);
        return REPORT_NORMAL;
      }
    case OP_GET_FUNCTIONS: {
        result[0] = REPORT_NORMAL;
        memcpy(result + 1, test_functions, 9);
        DLOGLN(F("Sent I/O Functions"));
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
        DLOG(F("Got Mainboard ID: ")); DLOGLN(mainID);
        return REPORT_NORMAL;
      }
    case OP_DATA_RESEND: {
        DLOGLN(F("Got a resend request"));
        return SAK_RESEND;
      }
    //Input
    case OP_INPUT_DIGITAL: {
        byte playerCount = packet[1];
        byte dataSize = packet[2];

        result[0] = REPORT_NORMAL;
        result[1] = systemSwitches;
        for (int i = 0; i < playerCount; i++) {
          result[2 + (i * 2)] = (playerSwitches[i] >> 8) & 0xFF;
          result[2 + (i * 2) + 1] = playerSwitches[i] & 0xFF;
        }

        *readSize = 2 + (playerCount * dataSize);
        return REPORT_NORMAL;
      }
    case OP_INPUT_COINS: {
        byte slotCount = packet[1];
        result[0] = REPORT_NORMAL;

        for (int i = 0; i < slotCount; i++) {
          byte cStat = (coinStatus >> (i * 2)) & 3;
          byte r1 = (cStat << 6) | ((coinCounts[i] >> 2) & 0b00111111);
          byte r2 = coinCounts[i] & 0xFF;
          result[i * 2 + 1] = r1;
          result[i * 2 + 2] = r2;
        }

        *readSize = 1 + (2 * slotCount);
        return REPORT_NORMAL;
      }
    //Output
    //If all else fails, unknown code error
    default:
      DLOG(F("Got unknown command: ")); DLOGLN(packet[0]);
      return SAK_UNKNOWN_CMD;
  }
}

/*
 * Deals with processing a single JVS packet at a time. The function waits
 * for the SYNC byte, reads in the packet (checking checksum), and feeds
 * each command into the command parser. Each result is collected and then
 * sent in a response packet.
 */
void processJVS() {
  byte dataBuffer[0xFF];

  jvsSetDirection(JVS_RX);
  DLOGLN(F("Waiting for SYNC..."));

  while (jvsReadByte() != SYNC); //Wait for SYNC byte

  DLOGLN(F("Packet found! Parsing."));

  short result = rcvPacket(dataBuffer);

  if (result == SAK_CHECKSUM_FAIL)
  {
    DLOGLN(F("Checksum failed."));
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
  pinMode(PIN_SERIAL_RX,  INPUT);
  pinMode(PIN_SERIAL_TX,  OUTPUT);
  pinMode(PIN_JVS_RX,     INPUT_PULLUP);
  pinMode(PIN_JVS_TX,     OUTPUT);
  pinMode(PIN_JVS_DIR,    OUTPUT);
  pinMode(PIN_SENSE_OUT,  OUTPUT);
  pinMode(PIN_SENSE_IN,   INPUT_PULLUP);
  pinMode(PIN_BUS_TERM,   OUTPUT);
  Serial.begin(9600);
  jvsSerial.begin(115200);

  jvsSenseHigh();

  //Check if we are the last device. If so, turn on bus term.
  float voltage = jvsGetSenseVoltage();
  if (voltage > 4.75f)
    jvsSetBusTerminator(true);
  else
    jvsSetBusTerminator(false);
}

unsigned long lastMillis = 0;
void loop() {  
  //Get latest controller states
  if (lastMillis - millis() > 200) {
    playerSwitches[0] = 0;
    playerSwitches[1] = 0;
    lastMillis = millis();
  }
  if (Serial.read() == 'T')
    setBit(playerSwitches[0], SW_TEST);
  else if (Serial.read() == 'S')
    setBit(playerSwitches[0], SW_SERVICE);

  //Process a JVS Packet
  processJVS();
}
