#include "tskUI.h"
#include <ui.h>
#include <Arduino.h>
#include "global.h"
#include <events.h>
#include "dev_debug.h"

DisplayUnit oled1(23, 19, 0);  // SDA, SCL, Wire0
DisplayUnit oled2(22, 21, 1);  // SDA, SCL, Wire1

void vMainOledTask(void* pv) {
  if (!oled1.begin()) {
    DEV_DBG_PRINTLN("vMainOledTask: init failed, stopping task");
    vTaskDelete(NULL);
  }
  DEV_DBG_PRINTLN("OLED1 (Humidity) task started");

  while (true) {
    float h0 = g_dhtHum[0], h1 = g_dhtHum[1], h2 = g_dhtHum[2];

  // 2) Build display message: humidity + shoe status
  char msg[128];
  const char* s1 = g_dhtStatus[0];
  const char* s2 = g_dhtStatus[1];
  snprintf(msg, sizeof(msg),
       "H1: %.1f%%  S1:%-4s\nH2: %.1f%%  S2:%-4s\nH3: %.1f%%",
       h0, s1, h1, s2, h2);

  oled1.showMessage(String(msg));
    vTaskDelay(pdMS_TO_TICKS(2000));


  }
}

void vAuxOledTask(void* pv) {
  if (!oled2.begin()) {
    DEV_DBG_PRINTLN("vAuxOledTask: init failed, stopping task");
    vTaskDelete(NULL);
  }
  DEV_DBG_PRINTLN("OLED2 (Temp) task started");

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
  xTaskCreatePinnedToCore(vMainOledTask, "OLED1", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(vAuxOledTask,  "OLED2", 4096, NULL, 1, NULL, 1);
}
