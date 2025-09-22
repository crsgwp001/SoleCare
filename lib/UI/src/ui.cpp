// system includes
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Arduino.h>

// project includes
#include "ui.h"

// Mutex to protect I2C/display accesses across tasks. Lazily created on first use.
static SemaphoreHandle_t s_displayMux = NULL;

// File-local helpers
static void lockDisplay() {
  if (!s_displayMux) {
    s_displayMux = xSemaphoreCreateMutex();
  }
  if (s_displayMux)
    xSemaphoreTake(s_displayMux, portMAX_DELAY);
}

static void unlockDisplay() {
  if (s_displayMux)
    xSemaphoreGive(s_displayMux);
}

// DisplayUnit implementation
DisplayUnit::DisplayUnit(uint8_t sda, uint8_t scl, uint8_t wireID)
    : sdaPin(sda), sclPin(scl), wire(wireID), display(128, 64, &wire, -1) {}

bool DisplayUnit::begin() {
  lockDisplay();
  wire.begin(sdaPin, sclPin);
  bool ok = true;
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.printf("OLED on bus %d init failed\n", wireID);
    ok = false;
  } else {
    display.clearDisplay();
    display.display();
  }
  unlockDisplay();
  return ok;
}

void DisplayUnit::showMessage(const String &msg) {
  lockDisplay();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(msg);
  display.display();
  unlockDisplay();
}

void DisplayUnit::showMessage(const String &msg, bool uv0, bool uv1, bool uv0p, bool uv1p) {
  lockDisplay();
  // Render text on left, small UV icons on the right
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(msg);

  // Draw two small UV indicators at the bottom-right corner
  const int iconW = 18, iconH = 10;
  const int iconX = 128 - iconW - 2;
  // place the bottom icon with a small margin from the bottom
  int yBottom = 64 - iconH - 2;   // bottom-most icon y
  int yTop = yBottom - iconH - 2; // icon above it

  // UV0 is the upper of the two (yTop), UV1 is bottom (yBottom)
  display.drawRect(iconX, yTop, iconW, iconH, SSD1306_WHITE);
  if (uv0)
    display.fillRect(iconX + 2, yTop + 2, iconW - 4, iconH - 4, SSD1306_WHITE);
  // If paused, draw a small pause glyph (two vertical bars) inside the icon
  if (uv0p) {
    int bx = iconX + 4;
    int by = yTop + 2;
    int bh = iconH - 4;
    display.fillRect(bx, by, 2, bh, SSD1306_BLACK);
    display.fillRect(bx + 4, by, 2, bh, SSD1306_BLACK);
    // If icon is not filled, draw white pause bars instead
    if (!uv0) {
      display.fillRect(bx, by, 2, bh, SSD1306_WHITE);
      display.fillRect(bx + 4, by, 2, bh, SSD1306_WHITE);
    }
  }
  display.setCursor(iconX - 22, yTop + 1);
  display.print("UV0");
  // (no uv-complete rendering in this version)

  display.drawRect(iconX, yBottom, iconW, iconH, SSD1306_WHITE);
  if (uv1)
    display.fillRect(iconX + 2, yBottom + 2, iconW - 4, iconH - 4, SSD1306_WHITE);
  if (uv1p) {
    int bx = iconX + 4;
    int by = yBottom + 2;
    int bh = iconH - 4;
    display.fillRect(bx, by, 2, bh, SSD1306_BLACK);
    display.fillRect(bx + 4, by, 2, bh, SSD1306_BLACK);
    if (!uv1) {
      display.fillRect(bx, by, 2, bh, SSD1306_WHITE);
      display.fillRect(bx + 4, by, 2, bh, SSD1306_WHITE);
    }
  }
  display.setCursor(iconX - 22, yBottom + 1);
  display.print("UV1");
  // (no uv-complete rendering in this version)

  display.display();
  unlockDisplay();
}
