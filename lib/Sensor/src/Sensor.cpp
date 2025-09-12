#include "Sensor.h"
#include <Arduino.h>

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
    float TK   = T + 273.15f;
    float Psat = 610.78f * expf((17.2694f * T)/(T + 237.3f));
    float Pv   = (RH / 100.0f) * Psat;
    return 1000.0f * Pv / (461.5f * TK);
}

