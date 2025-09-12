#include "ui.h"

DisplayUnit::DisplayUnit(uint8_t sda, uint8_t scl, uint8_t wireID)
    : sdaPin(sda)
    , sclPin(scl)
    , wire(wireID)
    , display(128, 64, &wire, -1) {}

bool DisplayUnit::begin() {
    wire.begin(sdaPin, sclPin);  
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.printf("OLED on bus %d init failed\n", wireID);
    return false;
    }

    display.clearDisplay();
    display.display();
}

void DisplayUnit::showMessage(const String& msg) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(msg);
    display.display();
}
