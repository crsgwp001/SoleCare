#include "tskUI.h"
#include <ui.h>
#include <Arduino.h>
#include "global.h"
#include <events.h>

DisplayUnit oled1(23, 19, 0);  // SDA, SCL, Wire0
DisplayUnit oled2(22, 21, 1);  // SDA, SCL, Wire1

void vMainOledTask(void* pv) {
  if (!oled1.begin()) {
    Serial.println("vMainOledTask: init failed, stopping task");
    vTaskDelete(NULL);
  }
  Serial.println("OLED1 (Humidity) task started");

  while (true) {
    float h0 = g_dhtHum[0], h1 = g_dhtHum[1], h2 = g_dhtHum[2];

    // 2) Build display message
    char msg[80];
    snprintf(msg, sizeof(msg),
             "H1: %.1f%% \nH2: %.1f%%\nH3: %.1f%%",
             h0, h1, h2);
    
    oled1.showMessage(msg);
    vTaskDelay(pdMS_TO_TICKS(2000));


  }
}

void vAuxOledTask(void* pv) {
  if (!oled2.begin()) {
    Serial.println("vAuxOledTask: init failed, stopping task");
    vTaskDelete(NULL);
  }
  Serial.println("OLED2 (Temp) task started");

  while (true) {
    char msg[64];
    snprintf(msg, sizeof(msg),
             "T1: %.1fC\nT2: %.1fC\nT3: %.1fC",
             g_dhtTemp[0], g_dhtTemp[1], g_dhtTemp[2]);
    oled2.showMessage(msg);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}


void createOledTasks() {
  xTaskCreatePinnedToCore(vMainOledTask, "OLED1", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vAuxOledTask,  "OLED2", 4096, NULL, 1, NULL, 1);
}
