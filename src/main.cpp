#include <Arduino.h>
#include "global.h"
#include "tskDHT.h"
#include "tskUI.h"

void setup() {
  Serial.begin(115200);
  createSensorTask();
  createOledTasks();
}

void loop() {
  // put your main code here, to run repeatedly:
}
