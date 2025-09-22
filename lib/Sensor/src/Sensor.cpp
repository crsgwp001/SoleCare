#include <Arduino.h>
#include "Sensor.h"

DHTSensor::DHTSensor(uint8_t pin, uint8_t type) : dht(pin, type) {}

void DHTSensor::begin() {
  dht.begin();
}

float DHTSensor::readTemperature() {
  lastTemp = dht.readTemperature();
  return lastTemp;
}

float DHTSensor::readHumidity() {
  lastHum = dht.readHumidity();
  return lastHum;
}

bool DHTSensor::isValid() {
  return !isnan(lastTemp) && !isnan(lastHum);
}

float computeAH(float T, float RH) {
  // Guard against NaN or unrealistic sensor values. If inputs are invalid
  // return NAN so callers can skip using the result.
  if (isnan(T) || isnan(RH))
    return NAN;
  // clamp RH to sensible physical range [0,100]
  if (RH < 0.0f)
    RH = 0.0f;
  if (RH > 100.0f)
    RH = 100.0f;
  // clamp T to a reasonable range to avoid overflow in expf
  if (T < -50.0f)
    T = -50.0f;
  if (T > 150.0f)
    T = 150.0f;

  float TK = T + 273.15f;
  float Psat = 610.78f * expf((17.2694f * T) / (T + 237.3f));
  float Pv = (RH / 100.0f) * Psat;
  return 1000.0f * Pv / (461.5f * TK);
}
