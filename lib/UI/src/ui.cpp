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

void DisplayUnit::showMessage(const String &msg, bool uv0, bool uv1) {
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
  display.setCursor(iconX - 22, yTop + 1);
  display.print("UV0");
  // (no uv-complete rendering in this version)

  display.drawRect(iconX, yBottom, iconW, iconH, SSD1306_WHITE);
  if (uv1)
    display.fillRect(iconX + 2, yBottom + 2, iconW - 4, iconH - 4, SSD1306_WHITE);
  display.setCursor(iconX - 22, yBottom + 1);
  display.print("UV1");
  // (no uv-complete rendering in this version)

  display.display();
  unlockDisplay();
}

void DisplayUnit::showSplash(const String &text, uint16_t letterDelayMs, uint16_t holdMs, int xPos, bool skipFade) {
  lockDisplay();
  
  // Letter-by-letter reveal animation
  String revealed = "";
  int len = text.length();
  
  for (int i = 0; i < len; i++) {
    revealed += text[i];
    
    // Clear and redraw with revealed letters
    display.clearDisplay();
    display.setTextSize(3);  // Larger size for prominent branding
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(xPos, 22);  // Adjusted for textSize(3) centering
    display.println(revealed);
    display.display();
    
    // Delay between letters
    vTaskDelay(pdMS_TO_TICKS(letterDelayMs));
  }
  
  // Hold fully revealed text
  vTaskDelay(pdMS_TO_TICKS(holdMs));
  
  // Only fade if not skipped
  if (!skipFade) {
    // Smooth fade-out with 4 blink frames (slower for better transition)
    for (int blink = 0; blink < 4; blink++) {
      display.clearDisplay();
      display.display();
      vTaskDelay(pdMS_TO_TICKS(200));  // Increased for smoother fade
      
      display.clearDisplay();
      display.setTextSize(3);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(xPos, 22);
      display.println(text);
      display.display();
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    // Final clear
    display.clearDisplay();
    display.display();
  }
  
  unlockDisplay();
}

void DisplayUnit::directClear() {
  lockDisplay();
  display.clearDisplay();
  display.display();
  unlockDisplay();
}

void DisplayUnit::directShow(const String &text, int xPos) {
  lockDisplay();
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(xPos, 22);
  display.println(text);
  display.display();
  unlockDisplay();
}
