#include "GenericHIDParser.h"

GenericHIDParser::GenericHIDParser(int id, void (*callback)(int, USBHID*,bool,uint8_t,uint8_t*)) {
  this->id = id;
  this->callback = callback;
}

void GenericHIDParser::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
        bool match = true;
        callback(id, hid, is_rpt_id, len, buf);
}

GenericHID::GenericHID(USB *p) : HIDUniversal(p) {
  lastResponse = -1;
}
  
uint32_t GenericHID::getVIDPID() {
  return ((uint32_t)VID << 16) | PID;
}

uint64_t GenericHID::getLastResponse() {
  return lastResponse;
}
