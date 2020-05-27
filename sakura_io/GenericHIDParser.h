#ifndef GENERIC_HID_PARSER_H
#define GENERIC_HID_PARSER_H

#include <usbhid.h>
#include <hiduniversal.h>

class GenericHIDParser : public HIDReportParser {
  private:
    int id;
    void (*callback)(int, USBHID*, bool, uint8_t, uint8_t*);

  public:
    GenericHIDParser(int id, void (*callback)(int, USBHID*, bool, uint8_t, uint8_t*));

    virtual void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);
};

class GenericHID : public HIDUniversal {
  public:
    GenericHID(USB *p);
    uint32_t getVIDPID();
    uint64_t getLastResponse();

  private:
    uint64_t lastResponse;

};

#endif
