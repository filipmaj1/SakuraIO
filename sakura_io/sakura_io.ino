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

#include <AltSoftSerial.h>
AltSoftSerial AltSerial;

#define JvsSerial Serial
#define PcSerial AltSerial

//Debug Flag (Adds/Removes log code)
#if 0x
#define DEBUG
char debugBuffer[100];
#define DebugSerial AltSerial
#define DebugLog(...) sprintf_P(debugBuffer, __VA_ARGS__); DebugSerial.print(debugBuffer)

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

#define AVR_EEPROM_SIZE 1024
#define MAP_DATA_START  0x60

//Pin Assignments
#define PIN_JVS_DIR     2
#define PIN_BUS_TERM    3
#define PIN_SENSE_OUT   4
#define PIN_SENSE_IN    A0

#define JVS_TX          HIGH
#define JVS_RX          LOW

//Identification
const char IDENTIFICATION[] PROGMEM = {"Fragmenter Works;Sakura I/O;v1.0a (Arduino Prototype);Created by Ioncannon"};
#define VERSION_CMD   0x13
#define VERSION_JVS   0x30
#define VERSION_COM   0x10

//Capabilities of this I/O device
const byte JVS_FUNCTIONS[] PROGMEM = {
  0x01, 0x02, 0x0E, 0x00, //Two Players, 14 Buttons each
  0x02, 0x02, 0x00, 0x00, //Two Coin slots
  0x03, 0x08, 0x00, 0x00, //Eight Analog inputs
  0x12, 0x06, 0x00, 0x00, //General Purpose Output Driver
  0x00                    //Terminator
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
#define SW_UP_LEFT     19
#define SW_DOWN_LEFT   18
#define SW_DOWN_RIGHT  17
#define SW_UP_RIGHT    16

//USB Constants
#define USB_TYPE_BUTTON 0
#define USB_TYPE_HAT_SW 1
#define USB_TYPE_AXIS   2

/////////
//GLOBALS
/////////

struct Map {
  uint32_t vidpid;
  byte size;
  byte* data;
};

//USB Objects
USB Usb;
USBHub Hub(&Usb);
void processUSB(int slot, USBHID* hid, bool isRpt, uint8_t len, uint8_t* buff);
GenericHIDParser genericHIDParser0(0, &processUSB);
GenericHIDParser genericHIDParser1(1, &processUSB);

//Maps
GenericHID Hid1(&Usb);
GenericHID Hid2(&Usb);
#define MAX_INPUTS 2
uint32_t lastLoadAttempts[MAX_INPUTS];
Map* currentMaps[MAX_INPUTS];
Map* loadedMaps[MAX_INPUTS];

//Current Sakura I/O State
byte currentState = STATE_UNKNOWN;
byte currentAddress = 0xFF;

//Saved info about the last send packet in case of resend
byte resultBuffer[0xFF];
byte lastResultStatus = 0;
byte lastResultSize = 0;

//Arcade State (Input, Output, Coins)
byte systemSwitches = 0;
uint16_t playerSwitches[] = {0x0000, 0x0000};
uint16_t playerAnalogs[8] = {0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000};
byte coinStatus = 0x0;
uint16_t coinCounts[] = {0x0000, 0x0000};

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
     vidpid (4 bytes)
     Offset (2 byte)

   Map Entry:
     name (15 bytes)
     mapSize (1 byte)
     mapData (x bytes)
*/

void fs_addMap(uint32_t vidpid, const char* mapName, const byte* mapData, byte mapDataSize) {
  byte numMappings;
  uint16_t lastMapOffset;
  byte lastMapSize;

  //Get the last offset, and add it's size for the total entry. Note: deleted maps MUST be followed by
  //a defragmentation or this won't work.
  EEPROM.get(0, numMappings);
  if ((numMappings & 0xF0) != 0xA0) //Mappings were corrupted or need to init
    numMappings = 0;
  else
    numMappings &= 0xF;

  //Check if vidpid exists, delete if it does
  for (int i = 0; i < numMappings; i++) {
    uint32_t readVidpid;
    EEPROM.get(1 + (i * 6), readVidpid);
    if (vidpid == readVidpid) {
      DebugLog(PSTR("Mapping already exists, deleting first.\r\n"));
      fs_deleteMap(i);
      EEPROM.get(0, numMappings); //Reload mapping count
      break;
    }
  }

  if (numMappings != 0) {
    EEPROM.get(1 + ((numMappings - 1) * 6) + 4, lastMapOffset);
    EEPROM.get(lastMapOffset + 0xF, lastMapSize);
  }

  //Write out the new entry
  uint32_t newOffset = numMappings != 0 ? lastMapOffset + 0x10 + lastMapSize : MAP_DATA_START;
  int nameLength = strlen(mapName) + 1;
  sakEEPROM_writeBytes(newOffset, mapName, nameLength <= 0xF ? nameLength : 0xF);
  EEPROM.put(newOffset + 0xE, 0);
  EEPROM.put(newOffset + 0xF, mapDataSize);
  sakEEPROM_writeBytes(newOffset + 0x10, mapData, mapDataSize);

  //Write the header entry
  EEPROM.put(1 + (numMappings * 6), vidpid);
  EEPROM.put(1 + (numMappings * 6) + 4, newOffset);

  //Update mapping count
  EEPROM.put(0, (byte)(0xA0 | ++numMappings));
}

void fs_deleteMap(int index) {
  byte numMappings;
  EEPROM.get(0, numMappings);
  if ((numMappings & 0xF0) != 0xA0) //Mappings were corrupted or need to init
  {
    numMappings = 0;
    return;
  }
  else
    numMappings &= 0xF;

  if (index >= numMappings)
    return;

  if (index != numMappings - 1) {
    uint32_t oldOffset;
    EEPROM.get(1 + (index * 6) + 4, oldOffset);

    //Copy each entry to the last spot
    for (byte i = index + 1; i < numMappings; i++) {
      uint32_t vidpid;
      uint16_t offset;
      uint8_t totalSize;

      EEPROM.get(1 + (i * 6), vidpid);
      EEPROM.get(1 + (i * 6) + 4, offset);
      EEPROM.put(1 + ((i - 1) * 6), vidpid);
      EEPROM.put(1 + ((i - 1) * 6) + 4, oldOffset);

      EEPROM.get(offset + 0xF, totalSize);
      totalSize += 0x10;

      for (int i = 0; i < totalSize; i++) {
        EEPROM.update(oldOffset++, EEPROM.read(offset + i));
      }
    }
  }

  EEPROM.update(0, 0xA0 | (numMappings - 1));
}

void fs_clear() {
  EEPROM.update(0, 0xA0);
}

uint16_t fs_getMap(byte* buff, int index) {
  uint8_t numMappings;
  uint32_t readVIDPID;
  uint16_t offset;
  unsigned char mapName[0xF];
  uint8_t mapSize;

  //LOAD MAP
  EEPROM.get(0, numMappings);
  if ((numMappings & 0xF0) != 0xA0) //Mappings were corrupted or need to init
    numMappings = 0;
  else
    numMappings &= 0xF;

  if (index >= numMappings) {
    return 0;
  }
  
  EEPROM.get(1 + (index * 6), ((uint32_t*)buff)[0]); //VIDPID
  EEPROM.get(1 + (index * 6) + 4, offset); //OFFSET

  for (int i = 0; i < 0xF; i++) {
    char charVal;
    EEPROM.get(offset + i, charVal);
    buff[4 + i] = charVal;
    if (charVal == 0)
      break;
  }

  EEPROM.get(offset + 0xF, buff[4 + 0xF]);
  sakEEPROM_readBytes(offset + 0x10, buff + 0x4 + 0x10, buff[0x4 + 0xF]);

  return buff[0x4 + 0xF];
}

int fs_loadMap(uint32_t vidpid, Map** newMap) {
  uint8_t numMappings;
  uint32_t readVIDPID;
  uint16_t offset;
  uint8_t mapSize;

  //LOAD MAP
  EEPROM.get(0, numMappings);
  for (byte i = 0; i < numMappings; i++) {
    EEPROM.get(1 + (i * 6), readVIDPID);
    //Found
    if (vidpid == readVIDPID) {
      *newMap = (Map*) malloc(sizeof(Map));

      if (*newMap == NULL) //OUT OF MEMORY
        return -1;

      EEPROM.get(1 + (i * 6) + 4, offset);
      EEPROM.get(offset + 0xF, mapSize);
      (*newMap)->data = (byte*) malloc(mapSize);
      sakEEPROM_readBytes(offset + 0x10, (*newMap)->data, mapSize);

      (*newMap)->vidpid = vidpid;
      (*newMap)->size = mapSize;

      return 1;
    }
  }
  return 0;
}

uint16_t fs_getUsedSpace() {
  uint8_t numMappings;
  uint16_t offset;
  uint8_t mapSize;
  uint16_t totalSize = MAP_DATA_START;

  //LOAD MAP
  EEPROM.get(0, numMappings);
  if ((numMappings & 0xF0) != 0xA0) //Mappings were corrupted or need to init
    numMappings = 0;
  else
    numMappings &= 0xF;

  for (int i = 0; i < numMappings; i++) {
    EEPROM.get(1 + (i * 6) + 4, offset);
    EEPROM.get(offset + 0xF, mapSize);
    totalSize += mapSize + 0x10;
  }

  return totalSize;
}

#ifdef DEBUG
void fs_printROM(int numPages) {
  byte data[0x100];

  int hexReset = 0;
  int hexCount = 0;
  DebugLog(PSTR("\r\n0x%02x: "), hexCount);
  for (int j = 0; j < numPages; j++) {
    sakEEPROM_readBytes(j * 0x100, data, 0x100);
    for (int i = 0; i < 0x100; i++) {
      DebugLog(PSTR("0x%02x "), data[i]);

      if (++hexReset == 0x10) {
        DebugLog(PSTR("\r\n0x%02x: "), hexCount + 1);
        hexReset = 0;
      }

      hexCount++;
    }
  }
}
#endif

int loadMap(uint32_t vidpid, Map** newMap) {
  int loadedMapIndex = -1;
  for (int i = 0; i < MAX_INPUTS; i++) {
    if (loadedMaps[i] == NULL) {
      loadedMapIndex = i;
      break;
    }
  }

  //NO FREE SLOTS (shouldn't happen if things are properly freed (and under max inputs).
  if (loadedMapIndex == -1)
    return -2;

  int rcode = fs_loadMap(vidpid, newMap);

  if (rcode)
    loadedMaps[loadedMapIndex] = *newMap;

  DebugLog(PSTR("Map Indx = %d\r\n"), loadedMapIndex);

  return rcode;
}

void freeMap(Map* toFree) {
  if (!toFree)
    return;
  for (int i = 0; i < MAX_INPUTS; i++) {
    if (loadedMaps[i] == toFree) {
      free(toFree->data);
      free(toFree);
      loadedMaps[i] = NULL;
    }
  }
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
  uint16_t resultSize = 0;

  if (PcSerial.available()) {
    byte huh = PcSerial.read();

    if (huh == '!') {
      byte header[4];
      PcSerial.readBytes(header, 4);

      byte opcode = header[0];
      byte sizeLO = header[1];
      byte sizeHI = header[2];
      byte checksum = header[3];

      unsigned int packetSize = sizeHI << 8 | sizeLO;

      PcSerial.readBytes(packetBytes, packetSize);

      //Checksum
      byte testChecksum = 0;
      for (int i = 0; i < packetSize; i++) {
        testChecksum += packetBytes[i];
      }

      if (checksum != testChecksum) {
        resultCode = 'Y';
        PcSerial.write(resultCode);
        PcSerial.write(resultCode ^ '!');
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
            resultSize = fs_getMap(packetBytes, packetBytes[0]);
            if (resultSize == 0)
              resultCode = 'O';
            else {
              resultSize += 0x14; 
              resultCode = '?';
            }
            break;
          }
        //Add Map
        case 2: {
            uint32_t vidpid = ((uint32_t*)&packetBytes)[0];
            memcpy(mapName, packetBytes + 4, 0xF);
            byte mapDataSize = packetBytes[0x4 + 0xF];
            //DebugLog(PSTR("Got add: vidpid: %lx, name: %s, mapsize: %d\r\n"), vidpid, mapName, mapDataSize);
            fs_addMap(vidpid, mapName, (const byte*) packetBytes + 0x14, mapDataSize);
            resultCode = 'O';
            break;
          }
        //Delete Map
        case 3: {            
            uint8_t index = packetBytes[0];            
            fs_deleteMap(index);
            resultCode = 'O';
            break;
          }
        //Get Info
        case 4: {
            uint16_t used, total;

            used = fs_getUsedSpace();
            total = AVR_EEPROM_SIZE;

            uint8_t strSize = strlen_P(IDENTIFICATION);
            memcpy_P(packetBytes, IDENTIFICATION, strSize);
            int writeSize = sprintf_P(packetBytes + strSize, PSTR(";%d;%d"), used, total);
            packetBytes[strSize + writeSize] = 0;
            
            resultSize = strSize + writeSize;
            resultCode = '?';
            break;
          }
        //Clear
        case 5: {
            fs_clear();
            resultCode = 'O';
            break;
          }
        default: {
            resultCode = 'X';
          }
      }

      //Write out the result code and any data if needed.
      PcSerial.write(resultCode);
      PcSerial.write(resultCode ^ '!');
      if (resultCode == '?') {
        PcSerial.write(resultSize);
        PcSerial.write(packetBytes, resultSize);
      }
      PcSerial.flush();
    }
  }
}

/* JVS DEST POSITIONS (http://superusr.free.fr/arcade/JVS/JVST_VER3.pdf)
   START    - 7
   SERVICE  - 6
   UP       - 5
   DOWN     - 4
   LEFT     - 3
   RIGHT    - 2
   B1       - 1
   B2       - 0
   B3       - 15
   B4       - 14
   B5       - 13
   B6       - 12
   B7       - 11
   B8       - 10
*/

//6 button map
byte testMap6Btn[] = {USB_TYPE_BUTTON, 0x00, 1, //[Type][Bit Position][JVS Dest]
                  USB_TYPE_BUTTON, 0x01, 14,
                  USB_TYPE_BUTTON, 0x02, 13,
                  USB_TYPE_BUTTON, 0x03, 0,
                  USB_TYPE_BUTTON, 0x04, 255,
                  USB_TYPE_BUTTON, 0x05, 15,
                  USB_TYPE_BUTTON, 0x06, 255,
                  USB_TYPE_BUTTON, 0x07, 12,
                  USB_TYPE_BUTTON, 0x08, 6,
                  USB_TYPE_BUTTON, 0x09, 7,
                  USB_TYPE_BUTTON, 0x0B, 255,
                  USB_TYPE_HAT_SW, 0x10, 0b1111, 8, 0, 5, 1, SW_UP_RIGHT, 2, 2, 3, SW_DOWN_RIGHT, 4, 4, 5, SW_DOWN_LEFT, 6, 3, 7, SW_UP_LEFT //[Type][Bit Position][Length Mask][Hat Maps][Map Pairs (Value,JVS Dest)]
                 };

//8 button map
byte testMap8Btn[] = {USB_TYPE_BUTTON, 0x00, 1, //[Type][Bit Position][JVS Dest]
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
                  USB_TYPE_HAT_SW, 0x10, 0b1111, 8, 0, 5, 1, SW_UP_RIGHT, 2, 2, 3, SW_DOWN_RIGHT, 4, 4, 5, SW_DOWN_LEFT, 6, 3, 7, SW_UP_LEFT //[Type][Bit Position][Length Mask][Hat Maps][Map Pairs (Value,JVS Dest)]
                 };

//6 button map (Tom's Arcade Stick)
byte tomStick6Btn[] = {
  USB_TYPE_BUTTON, 0x00, 1,   //B1
  USB_TYPE_BUTTON, 0x01, 14,  //B5
  USB_TYPE_BUTTON, 0x02, 13,  //B6
  USB_TYPE_BUTTON, 0x03, 0,   //B2
  USB_TYPE_BUTTON, 0x04, 255, //B4
  USB_TYPE_BUTTON, 0x05, 15,  //B3
  USB_TYPE_BUTTON, 0x06, 255, //B8
  USB_TYPE_BUTTON, 0x07, 12,  //B7
  USB_TYPE_BUTTON, 0x08, 6,   //SELECT
  USB_TYPE_BUTTON, 0x09, 7,   //START
  USB_TYPE_BUTTON, 0x0C, 255, //HOME
  USB_TYPE_HAT_SW, 0x10, 0b1111, 8, 0, 5, 1, SW_UP_RIGHT, 2, 2, 3, SW_DOWN_RIGHT, 4, 4, 5, SW_DOWN_LEFT, 6, 3, 7, SW_UP_LEFT, //[Type][Bit Position][Length Mask][Hat Maps][Map Pairs (Value,JVS Dest)]
  USB_TYPE_AXIS, 0x18, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0,
  USB_TYPE_AXIS, 0x20, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 1,
  USB_TYPE_AXIS, 0x28, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 2,
  USB_TYPE_AXIS, 0x30, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 3
};

byte hotas[] = {  
  USB_TYPE_AXIS, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 1,
  USB_TYPE_AXIS, 0x10, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0,
  USB_TYPE_AXIS, 0x20, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x0F, 0x00, 0x00, 2,
  // qrwUSB_TYPE_HAT_SW, 0x2C, 0b1111, 0, //[Type][Bit Position][Length Mask][Hat Maps][Map Pairs (Value,JVS Dest)]
  USB_TYPE_BUTTON, 0x30, 7,   //B1
  USB_TYPE_BUTTON, 0x31, 1,   //B2
  USB_TYPE_BUTTON, 0x32, 0,  //B3
  USB_TYPE_BUTTON, 0x33, 15,  //B4
  USB_TYPE_BUTTON, 0x34, 6,  //B5
  USB_TYPE_BUTTON, 0x35, 7,  //B6
};

//0x7382215
//0: 2bytes, joy x
//16: 2bytes, joy y
//32: 1.5bytes, rudder
//44: 1 byte, pov - hat
//48: trigger button
//49: a button
//50: b button
//51: c button
//52: d button
//53: e (pull) button
//54: H1 Up 
//55: H1 Right 
//56: H1 Down 
//57: H1 Left
//58: H2 Up
//59: H2 Right
//60: H2 Down
//61: H2 Left

/*
   Analog
   24th bit (0x18), 1 Byte, LS X AXIS (Pointer)(0, 80, FF)
   32th bit (0x20), 1 Byte, LS Y AXIS (Pointer)(0, 80, FF)
   40th bit (0x28), 1 Byte, RS X AXIS (Slide)(0, 80, FF, 7F when OFF)
   48th bit (0x30), 1 Byte, RS Y AXIS (Slide)(0, 80, FF, 7F when OFF)
*/

/*~~~~~~~~~~~~~~~~~~USB CODE~~~~~~~~~~~~~~~~~~*/

/*
   Deals with processing the USB Host as well as loading map files.
*/
void processUSB(int id, USBHID* hid, bool isRpt, uint8_t len, uint8_t* buff) {
  GenericHID* thisHid = (GenericHID*) hid;
  uint32_t vidpid = thisHid->getVIDPID();

  //debugSerial.print("XXXXXXXXXXXXXX");
  //debugSerial.write(buff, len);

  //If VIDPID has changed, a different usb device was plugged into this slot. Load Map!
  if ((currentMaps[id] == NULL && lastLoadAttempts[id] != vidpid) || (currentMaps[id] != NULL && currentMaps[id]->vidpid != vidpid)) {

    DebugLog(PSTR("New Map Needed\r\n"));
    DebugLog(PSTR("ID: %x, VIDPID: %lx\r\n"), id, vidpid);

    //Check if map is already loaded
    Map* newMap = NULL;
    for (int i = 0; i < MAX_INPUTS; i++) {
      if (loadedMaps[i] != NULL && vidpid == loadedMaps[i]->vidpid) {
        newMap = loadedMaps[i];
        DebugLog(PSTR("Map already loaded\r\n"));
        break;
      }
    }

    //If not, free this map and load a new one
    if (newMap == NULL) {
      free(currentMaps[id]);
      int loaded = loadMap(vidpid, &newMap);
      DebugLog(PSTR("Loading new map %d\r\n"), loaded);
      //If loaded, assign the map. Otherwise it stays unassigned.
      if (loaded == 1)
        currentMaps[id] = newMap;
      else {
        lastLoadAttempts[id] = vidpid;
        DebugLog(PSTR("Last Attempt SET: %x\r\n"), lastLoadAttempts[id]);
        if (loaded == -1) {
          DebugLog(PSTR("ERROR: Cannot load map, out of memory!\r\n"));
        }
      }
    }  else {
      currentMaps[id] = newMap; //Otherwise set to this map;
    }
  }

  if (currentMaps[id] == NULL) {
    DebugLog(PSTR("No map loaded....\r\n-----------------\r\n"));
    return;
  }

  //Handle map/input reading
  int playerIndex = id;

  for (int i = 0; i < currentMaps[id]->size; i++) {
    byte type = currentMaps[id]->data[i++];
    byte bitPosition = currentMaps[id]->data[i++];

    //Start Post
    byte bytePosition = bitPosition / 8;
    byte remainder = bitPosition % 8;

    //Button
    if (type == USB_TYPE_BUTTON) {
      byte jvsDest = currentMaps[id]->data[i];

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
      byte valueMask = currentMaps[id]->data[i++];
      byte innerMapLen = currentMaps[id]->data[i++];

      uint32_t* intBuffer = (uint32_t*)(buff + bytePosition);
      int value = (intBuffer[0] >> remainder) & valueMask;

      playerSwitches[playerIndex] &= 0b1111111111000011;

      for (int j = 0; j < innerMapLen; j++) {
        //Found the value. Assign and set scanner to the end.
        if (currentMaps[id]->data[i + (j * 2)] == value) {
          switch (currentMaps[id]->data[i + (j * 2) + 1]) {
            case SW_UP_LEFT:
              playerSwitches[playerIndex] |= 0b0000000000101000;
              break;
            case SW_DOWN_LEFT:
              playerSwitches[playerIndex] |= 0b0000000000011000;
              break;
            case SW_UP_RIGHT:
              playerSwitches[playerIndex] |= 0b0000000000100100;
              break;
            case SW_DOWN_RIGHT:
              playerSwitches[playerIndex] |= 0b0000000000010100;
              break;
            default:
              playerSwitches[playerIndex] |= (1 << currentMaps[id]->data[i + (j * 2) + 1]);
          }
        }
      }
      i += innerMapLen * 2;
      i--;
    }
    //Analog
    else if (type == USB_TYPE_AXIS) {
      uint32_t* intBuffer = (uint32_t*)(currentMaps[id]->data + i);
      uint32_t valueMask = intBuffer[0];
      uint32_t minVal = intBuffer[1];
      uint32_t maxVal = intBuffer[2];
      i += 12;
      byte channel = currentMaps[id]->data[i];

      intBuffer = (uint32_t*)(buff + bytePosition);
      uint32_t value = (intBuffer[0] >> remainder) & valueMask;

      uint16_t scaledValue = scaleToJVS(value, minVal, maxVal);
      playerAnalogs[channel] = (scaledValue>>8) | (scaledValue<<8);
    }
  }

  //DebugLog(PSTR("Switches: %c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c\r\n"), BYTE_TO_BINARY((playerSwitches[playerIndex] >> 8) & 0xFF), BYTE_TO_BINARY(playerSwitches[playerIndex] & 0xFF));
  //DebugLog(PSTR("Analog: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\r\n"), playerAnalogs[0], playerAnalogs[1], playerAnalogs[2], playerAnalogs[3], playerAnalogs[7]);
}

uint16_t scaleToJVS(uint32_t value, uint32_t usbMin, uint32_t usbMax) {
  return 0xFFFF * (value - usbMin) / (usbMax - usbMin);
}

/*~~~~~~~~~~~~~~~~~~JVS CODE~~~~~~~~~~~~~~~~~~*/

/*
   Blocking reads one byte from the serial link, handling the escape byte case.
*/
byte jvsReadByte() {
  while (!JvsSerial.available());
  byte in = JvsSerial.read();
  if (in == ESCAPE) {
    while (!JvsSerial.available());
    in = JvsSerial.read() + 1;
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
  //Read packet and check if it is for us
  byte targetNode = jvsReadByte();
  byte numBytes = jvsReadByte();
  JvsSerial.readBytes(dataBuffer, numBytes); //TODO: Change to jvsRead
  
  if (targetNode == BROADCAST || targetNode == currentAddress) {
    //Test checksum
    byte checksum = dataBuffer[numBytes - 1];
    byte testChecksum = targetNode + numBytes;
    for (byte i = 0; i < numBytes - 1; i++)
      testChecksum += dataBuffer[i];

    if (checksum == testChecksum)
      return numBytes - 1;
    return SAK_CHECKSUM_FAIL;
  }
  
  //DebugLog(PSTR("\r\nIgnoring Packet.\r\n"));
  return 0; //Not for us
}

/*
   Sends a result packet down the line. Payload version.
   Args: The resulting STATUS and it's payload.
*/
void sendResponse(byte statusCode, byte payloadSize) {

  payloadSize += 2;

  //Build Checksum
  byte checksum = payloadSize + statusCode;
  for (byte i = 0; i < payloadSize - 2; i++)
    checksum += resultBuffer[i];
  checksum %= 0xFF;

  //Write out
  jvsSetDirectionTX();
  JvsSerial.write(SYNC);                           //SYNC Byte
  JvsSerial.write(0x00);                           //Node Num (always 0)
  JvsSerial.write(payloadSize);                    //Num Bytes
  JvsSerial.write(statusCode);                     //Status
  for (int i = 0; i < payloadSize - 2; i++) {
    JvsSerial.write(resultBuffer[i]);
  }
  JvsSerial.write(checksum);                       //Checksum
  JvsSerial.flush();
  
  delayMicroseconds(100);
  
  jvsSetDirectionRX();
}

//////////////////////////
//~~MAIN JVS CMD HANDLER~~
//////////////////////////
short parseCommand(const byte* packet, byte* readSize, byte* result, short* resultSize) {
  *readSize = 1;
  *resultSize = 1;

  switch (packet[0]) {
    //Unknown command send by Initial D 3.
    case 0: {
        DebugLog(PSTR("Wut\r\n"));
        DebugLog(PSTR("%x "), packet[0]);
        DebugLog(PSTR("%x "), packet[1]);
        DebugLog(PSTR("%x "), packet[2]);
        DebugLog(PSTR("%x "), packet[3]);
        DebugLog(PSTR("%x "), packet[4]);
        DebugLog(PSTR("%x "), packet[5]);
        result[0] = REPORT_NORMAL;
        return REPORT_NORMAL;
      }
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
        byte idSize = strlen_P(IDENTIFICATION) + 1;
        memcpy_P(result + 1, IDENTIFICATION, idSize);
        result[idSize] = 0;
        *resultSize = idSize + 1;
        DebugLog(PSTR("Sent I/O ID\r\n"));
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
        *resultSize = 1 + 17;
        result[0] = REPORT_NORMAL;
        memcpy_P(result + 1, JVS_FUNCTIONS, 17);
        DebugLog(PSTR("Sent I/O Functions\r\n"));
        return REPORT_NORMAL;
      }
    case OP_MAIN_ID: {
        DebugLog(PSTR("!!!!MainID!!!!\r\n"));
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

        *readSize += 2;
        *resultSize = 2 + (playerCount * dataSize);

        return REPORT_NORMAL;
      }
    case OP_INPUT_COINS: {
        byte slotCount = packet[1];
        result[0] = REPORT_NORMAL;
        for (int i = 0; i < slotCount; i++) {
          byte cStat = (coinStatus >> (i * 2)) & 3;
          byte r1 = (cStat << 6) | ((coinCounts[i] >> 8) & 0b00111111);
          byte r2 = coinCounts[i] & 0xFF;
          result[(i * 2) + 1] = r1;
          result[(i * 2) + 2] = r2;
        }

        *readSize += 1;
        *resultSize = 1 + (2 * slotCount);

        return REPORT_NORMAL;
      }
    case OP_INPUT_ANALOG: {
        byte numChannels = packet[1];

        result[0] = REPORT_NORMAL;

        memcpy(result + 1, &playerAnalogs, 2 * numChannels);

        *readSize += 1;
        *resultSize = 1 + (2 * numChannels);

        return REPORT_NORMAL;
      }
    //Output
    case OP_OUTPUT_SUB_COINS: {
        result[0] = REPORT_NORMAL;
        *readSize += 3;

        byte coinSlot = packet[1];
        short subtraction = packet[2] << 8 | packet[3];
        coinCounts[coinSlot] -= subtraction;

        DebugLog(PSTR("Got request to reduce coins by: %d.\r\n"), subtraction);

        return REPORT_NORMAL;
      }
    case OP_OUTPUT_GP1: {
        result[0] = REPORT_NORMAL;

        byte dataSize = packet[1];

        *readSize += 1 + dataSize;
        return REPORT_NORMAL;
      }
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

  while (JvsSerial.read() != SYNC) {
    //Timeout so it doesn't get lock execution.
    if (millis() - lastJVSMillis >= JVS_TIMEOUT)
      return;
  }

  short result = rcvPacket(dataBuffer);

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
  DebugSerial.begin(9600);
#endif
  DebugLog(PSTR("Sakura I/O Board; a JVS to USB adapter.\r\n"));

  DebugLog(PSTR("Setting up pins...\r\n"));
  pinMode(PIN_JVS_DIR,    OUTPUT);
  pinMode(PIN_SENSE_OUT,  OUTPUT);
  pinMode(PIN_SENSE_IN,   INPUT_PULLUP);
  pinMode(PIN_BUS_TERM,   OUTPUT);

  DebugLog(PSTR("Starting JVS Serial Port...\r\n"));
  JvsSerial.begin(115200);
  jvsSetDirectionRX();
  jvsSenseLow();

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
    Hid1.SetReportParser(0, &genericHIDParser0);
    Hid2.SetReportParser(1, &genericHIDParser1);
  }
  DebugLog(PSTR("Sakura I/O Initialized!\r\n"));
}

void loop() {
  //fs_clear();
  //fs_addMap(0x0F0D0040, "Hori Stick", testMap6Btn, 53);
  //fs_addMap(0x07388838, "Tom's Stick", tomStick6Btn, 113);
  //fs_addMap(0x7382215, "Hotas Test", hotas, 60);
  //fs_printROM(2);

  while (true) {
    //Process PC serial if needed
    processMapManager();

    //Poll the USB devices
    Usb.Task();

    //Process a JVS Packet
    processJVS();
  }
}
