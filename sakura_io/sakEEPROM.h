#include <EEPROM.h>
#include <Arduino.h>

template <class T> int sakEEPROM_Write(int address, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(address++, *p++);
  return i;
}

template <class T> int sakEEPROM_Read(int address, T& value)
{
  byte* p = (byte*)(void*)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(address++);
  return i;
}

int sakEEPROM_WriteBytes(int address, const void* value, int count)
{
  byte* p = (byte*)(void*)&value;
  unsigned int i;
  for (i = 0; i < count; i++)
    EEPROM.write(address++, *p++);
  return i;
}

int sakEEPROM_ReadBytes(int address, const void* value, int count)
{
  byte* p = (byte*)(void*)&value;
  unsigned int i;
  for (i = 0; i < count; i++)
    *p++ = EEPROM.read(address++);
  return i;
}
