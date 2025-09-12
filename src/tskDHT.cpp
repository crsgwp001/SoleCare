#include "tskDHT.h"
#include <Sensor.h>
#include <Arduino.h>
#include "global.h"

// Define the global variables
float g_dhtTemp[3] = {0};
float g_dhtHum[3] = {0};

DHTSensor sensor1(17);
DHTSensor sensor2(16);
DHTSensor sensor3(4);

void vSensorTask(void* pvParameters) {
    sensor1.begin();
    sensor2.begin();
    sensor3.begin();

    while (true) {
        g_dhtTemp[0] = sensor1.readTemperature();
        g_dhtHum[0]  = sensor1.readHumidity();
        g_dhtTemp[1] = sensor2.readTemperature();
        g_dhtHum[1]  = sensor2.readHumidity();
        g_dhtTemp[2] = sensor3.readTemperature();
        g_dhtHum[2]  = sensor3.readHumidity();

        Serial.printf("S1: %.1f°C %.1f%%\n", g_dhtTemp[0], g_dhtHum[0]);
        Serial.printf("S2: %.1f°C %.1f%%\n", g_dhtTemp[1], g_dhtHum[1]);
        Serial.printf("S3: %.1f°C %.1f%%\n", g_dhtTemp[2], g_dhtHum[2]);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void createSensorTask() {
    xTaskCreatePinnedToCore(
        vSensorTask,
        "SensorTask",
        4096,
        NULL,
        1,
        NULL,
        1
    );
}