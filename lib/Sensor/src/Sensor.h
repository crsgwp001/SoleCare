#pragma once

// system includes
#include <Arduino.h>
#include <DHT.h>

class DHTSensor {
public:
  DHTSensor(uint8_t dhtPin, uint8_t dhtType = DHT22);
  void begin();
  float readTemperature();
  float readHumidity();
  bool isValid(); // Optional: check for NaN or fault

private:
  DHT dht;
  uint8_t pin;
  float lastTemp;
  float lastHum;
};

// Helper: compute absolute humidity (g/m^3)
float computeAH(float T, float RH);
