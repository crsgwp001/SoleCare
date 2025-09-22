#include "global.h"
#include "tskDHT.h"
#include "tskFSM.h"
#include "tskUI.h"
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  createSensorTask();
  createOledTasks();
  createStateMachineTask();
}

void loop() {}
