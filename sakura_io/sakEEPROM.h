#include <EEPROM.h>
#include <Arduino.h>

int sakEEPROM_writeBytes(int address, const byte* buff, int count)
{
  unsigned int i;
  for (i = 0; i < count; i++)
    EEPROM.update(address++, buff[i]);
  return i;
}

int sakEEPROM_readBytes(int address, byte* buff, int count)
{
  unsigned int i;
  for (i = 0; i < count; i++)
    buff[i] = EEPROM.read(address++);
  return i;
}
