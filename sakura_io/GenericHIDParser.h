#ifndef GENERIC_HID_PARSER_H
#define GENERIC_HID_PARSER_H

#include <usbhid.h>

class GenericHIDParser : public HIDReportParser {
        uint8_t oldPad[5];
        uint8_t oldHat;
        uint16_t oldButtons;

private:
        void (*callback)(USBHID*,bool,uint8_t,uint8_t*);

public:
        GenericHIDParser(void (*callback)(USBHID*,bool,uint8_t,uint8_t*));

        virtual void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);
};

#endif
