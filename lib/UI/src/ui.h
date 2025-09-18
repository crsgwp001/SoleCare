#pragma once

#include <Adafruit_ssd1306.h>
#include <Wire.h>

class DisplayUnit {
public:
    DisplayUnit(uint8_t sda, uint8_t scl, uint8_t wireID);
    bool begin();
    void showMessage(const String& msg);

private:
    uint8_t sdaPin;
    uint8_t sclPin;
    uint8_t wireID;
    TwoWire wire;
    Adafruit_SSD1306 display;
};
