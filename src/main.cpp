#include <Arduino.h>
#include "global.h"
#include "tskDHT.h"
#include "tskUI.h"
#include "tskFSM.h"



void setup() {
  Serial.begin(115200);
  createSensorTask();
  createOledTasks();
  createStateMachineTask();
}

void loop() {
  // put your main code here, to run repeatedly:
}
