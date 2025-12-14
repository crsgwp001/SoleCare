#include "global.h"
#include "tskDHT.h"
#include "tskFSM.h"
#include "tskUI.h"
#include "config.h"
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  
  // Initialize LED pins
  pinMode(HW_STATUS_LED_PIN, OUTPUT);
  pinMode(HW_ERROR_LED_PIN, OUTPUT);
  digitalWrite(HW_STATUS_LED_PIN, LOW);
  digitalWrite(HW_ERROR_LED_PIN, LOW);
  
  createSensorTask();
  createOledTasks();
  createStateMachineTask();
}

void loop() {}
