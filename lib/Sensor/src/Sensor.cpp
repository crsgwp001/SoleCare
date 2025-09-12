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