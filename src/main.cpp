#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <DHT.h>

// OLED setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_SDA      23
#define OLED_SCL      19
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DHT setup
#define DHTTYPE DHT22  // or DHT11 if you're using that variant
DHT dht1(17, DHTTYPE);
DHT dht2(16, DHTTYPE);
DHT dht3(4,  DHTTYPE);

void setup() {
  Serial.begin(115200);
  Wire.begin(OLED_SDA, OLED_SCL);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  dht1.begin();
  dht2.begin();
  dht3.begin();

  display.println("DHT Test Init...");
  display.display();
  delay(2000);
}

void loop() {
  float t1 = dht1.readTemperature();
  float h1 = dht1.readHumidity();
  float t2 = dht2.readTemperature();
  float h2 = dht2.readHumidity();
  float t3 = dht3.readTemperature();
  float h3 = dht3.readHumidity();

  Serial.printf("Sensor 1 (Pin 17): T=%.1f°C H=%.1f%%\n", t1, h1);
  Serial.printf("Sensor 2 (Pin 16): T=%.1f°C H=%.1f%%\n", t2, h2);
  Serial.printf("Sensor 3 (Pin 4):  T=%.1f°C H=%.1f%%\n\n", t3, h3);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("DHT1: %.1fC %.1f%%\n", t1, h1);
  display.printf("DHT2: %.1fC %.1f%%\n", t2, h2);
  display.printf("DHT3: %.1fC %.1f%%\n", t3, h3);
  display.display();

  delay(2000);
}