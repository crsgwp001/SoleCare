#pragma once

#include <Wire.h>
#include <Adafruit_ssd1306.h>

class DisplayUnit {
public:
  DisplayUnit(uint8_t sda, uint8_t scl, uint8_t wireID);
  bool begin();
  void showMessage(const String &msg);
  // show message and render UV status icons (uv0, uv1) on the right side
  // uv0/uv1: whether UV is started; uv0p/uv1p: whether UV is paused
  void showMessage(const String &msg, bool uv0, bool uv1, bool uv0p, bool uv1p);

private:
  uint8_t sdaPin;
  uint8_t sclPin;
  uint8_t wireID;
  TwoWire wire;
  Adafruit_SSD1306 display;
};
