#pragma once

#include <Wire.h>
#include <Adafruit_SSD1306.h>

class DisplayUnit {
public:
  DisplayUnit(uint8_t sda, uint8_t scl, uint8_t wireID);
  bool begin();
  void showMessage(const String &msg);
  // Show message and render UV status icons (uv0, uv1) on the right side
  void showMessage(const String &msg, bool uv0, bool uv1);
  // Splash animation: letter-by-letter reveal with optional fade
  // xPos: horizontal cursor position (left=0, right=100+)
  // skipFade: if true, skip fade-out and leave text on screen
  void showSplash(const String &text, uint16_t letterDelayMs = 150, uint16_t holdMs = 800, int xPos = 20, bool skipFade = false);
  
  // Direct clear and show for synchronized transitions
  void directClear();
  void directShow(const String &text, int xPos);
  uint8_t sdaPin;
  uint8_t sclPin;
  uint8_t wireID;
  TwoWire wire;
  Adafruit_SSD1306 display;
};
