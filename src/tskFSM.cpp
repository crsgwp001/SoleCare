#include "tskFSM.h"
#include <Arduino.h>
#include <StateMachine.h>
#include <Events.h>

// Button on GPIO0 with INPUT_PULLUP
static constexpr int START_PIN = 35;
static constexpr int RESET_PIN = 34;

// Instantiate your FSMs as globals (visible only in this translation unit)
static StateMachine<GlobalState, Event> fsmGlobal(GlobalState::Idle);
static StateMachine<SubState,  Event> fsmSub1  (SubState::S_IDLE);
static StateMachine<SubState,  Event> fsmSub2  (SubState::S_IDLE);

// Simple 50 ms software debounce; returns true once per press
static bool readStart() {
  static bool    lastState = HIGH;
  static uint32_t lastTime = 0;

  bool raw = digitalRead(START_PIN);
  if (raw != lastState && (millis() - lastTime) > 50) {
    lastTime  = millis();
    lastState = raw;
    Serial.println("Start Pressed");
    if (raw == LOW) {
      return true;  // pressed event
    }
  }
  return false;
}

static bool readReset() {
  static bool    lastState = HIGH;
  static uint32_t lastTime = 0;

  bool raw = digitalRead(RESET_PIN);
  if (raw != lastState && (millis() - lastTime) > 50) {
    lastTime  = millis();
    lastState = raw;
    Serial.println("Reset Pressed");
    if (raw == LOW) {
      return true;  // pressed event
    }
  }
  return false;
}

// Variadic helper to broadcast an event to any number of FSMs
template<typename StateT, typename EventT, typename... FSMs>
static void broadcast(EventT ev, StateMachine<StateT, EventT>& first, FSMs&... rest) {
  first.handleEvent(ev);
  if constexpr(sizeof...(rest) > 0) {
    broadcast(ev, rest...);
  }
}

// Configure transitions and run-callbacks for your FSMs
static void setupStateMachines() {
  // Global: Off → On on Start
  fsmGlobal.addTransition({
    GlobalState::Idle , Event::StartPressed, GlobalState::Running, [](){
      Serial.println("GLOBAL: Running");
    }
  });

  // Global: On → Off on Stop
  fsmGlobal.addTransition({
    GlobalState::Running, Event::ResetPressed, GlobalState::Idle, [](){
      Serial.println("GLOBAL: Idle");
    }
  });

  // While in Global.On, tick sub-machines each loop
  fsmGlobal.setRun(GlobalState::Running, [](){
    fsmSub1.run();
    fsmSub2.run();
  });

  // Sub1: Idle → Active / Active → Idle
  fsmSub1.addTransition({ SubState::S_IDLE,   Event::StartPressed, SubState::S_DRYER, [](){
    Serial.println("SUB1: UP");
  }});
  fsmSub1.addTransition({ SubState::S_DRYER, Event::StartPressed,  SubState::S_IDLE,   [](){
    Serial.println("SUB1: DOWN");
  }});

  // Sub2: Idle → Active / Active → Idle
  fsmSub2.addTransition({ SubState::S_IDLE,   Event::StartPressed, SubState::S_DRYER, [](){
    Serial.println("SUB2: UP");
  }});
  fsmSub2.addTransition({ SubState::S_DRYER, Event::StartPressed,  SubState::S_IDLE,   [](){
    Serial.println("SUB2: DOWN");
  }});
}

// The FreeRTOS task
static void vStateMachineTask(void* pvParameters) {
  // Initialize
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  setupStateMachines();
  Serial.println("FSM Task started");

  while (true) {
    // 1) On each button press, toggle Start/Stop on all machines
    if (readStart()) {
      if (fsmGlobal.getState() == GlobalState::Idle) {
        broadcast(Event::StartPressed, fsmGlobal, fsmSub1, fsmSub2);}
      if (fsmGlobal.getState() == GlobalState::Running) {
      broadcast(Event::StartPressed, fsmGlobal, fsmSub1, fsmSub2);}}
    
    if (readReset()) {
        broadcast(Event::ResetPressed, fsmGlobal, fsmSub1, fsmSub2);}

    // 2) Always run the global FSM (which runs subs when On)
    fsmGlobal.run();

    // 3) Yield to other tasks & reset watchdog
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void createStateMachineTask() {
  xTaskCreatePinnedToCore(
    vStateMachineTask,
    "StateMachineTask",
    4096,
    nullptr,
    1,
    nullptr,
    1
  );
}