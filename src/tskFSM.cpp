#include "tskFSM.h"
#include <Arduino.h>
#include <StateMachine.h>
#include <Events.h>
#include "global.h"

// Your global string status array (2 shoes, up to 4 chars + null)

// Button definitions
static constexpr int START_PIN = 35;
static constexpr int RESET_PIN = 34;

// Instantiate Global + two Sub-FSMs
static StateMachine<GlobalState, Event> fsmGlobal(GlobalState::Idle);
static StateMachine<SubState,   Event> fsmSub1  (SubState::S_IDLE);
static StateMachine<SubState,   Event> fsmSub2  (SubState::S_IDLE);

//------------------------------------------------------------------------------
// 50 ms software debounce helpers
//------------------------------------------------------------------------------
static bool readStart() {
  static bool    last = HIGH;
  static uint32_t t0  = 0;
  bool raw = digitalRead(START_PIN);
  if (raw != last && (millis() - t0) > 300) {
    t0   = millis();
    last = raw;
    if (raw == LOW) return true;
  }
  return false;
}

static bool readReset() {
  static bool    last = HIGH;
  static uint32_t t0  = 0;
  bool raw = digitalRead(RESET_PIN);
  if (raw != last && (millis() - t0) > 300) {
    t0   = millis();
    last = raw;
    if (raw == LOW) return true;
  }
  return false;
}

//------------------------------------------------------------------------------
// Variadic broadcast: send the same event into N FSMs
//------------------------------------------------------------------------------
template<typename StateT, typename EventT, typename... Fs>
static void broadcast(EventT ev,
                      StateMachine<StateT, EventT>& first,
                      Fs&... rest) {
  first.handleEvent(ev);
  if constexpr(sizeof...(rest) > 0) {
    broadcast(ev, rest...);
  }
}

//------------------------------------------------------------------------------
// Configure all transitions and run-callbacks
//------------------------------------------------------------------------------
static void setupStateMachines() {
  // Global Idle → Running
  fsmGlobal.addTransition({
    GlobalState::Idle, Event::StartPressed, GlobalState::Running,
    [](){ Serial.println("GLOBAL: Running"); }
  });

  // Global Running → Idle
  fsmGlobal.addTransition({
    GlobalState::Running, Event::ResetPressed, GlobalState::Idle,
    [](){ Serial.println("GLOBAL: Idle"); }
  });

  // When in Running, poll both sub-FSMs each cycle
  fsmGlobal.setRun(GlobalState::Running, [](){
    fsmSub1.run();
    fsmSub2.run();
  });

  // Sub1: Idle → S_WET / S_DRY on init events
  fsmSub1.addTransition({ SubState::S_IDLE, Event::Shoe0InitWet, SubState::S_WET,
    [](){ Serial.println("SUB1: entered S_WET"); }
  });
  fsmSub1.addTransition({ SubState::S_IDLE, Event::Shoe0InitDry, SubState::S_DRY,
    [](){ Serial.println("SUB1: entered S_DRY"); }
  });

  // Sub1: example StartPressed path
  fsmSub1.addTransition({ SubState::S_WET,   Event::StartPressed, SubState::S_COOLING,
    [](){ Serial.println("SUB1: Wet→Cooling"); }
  });
  fsmSub1.addTransition({ SubState::S_COOLING,Event::StartPressed, SubState::S_DRY,
    [](){ Serial.println("SUB1: Cooling→Dry"); }
  });
  fsmSub1.addTransition({ SubState::S_DRY,   Event::StartPressed, SubState::S_DONE,
    [](){ Serial.println("SUB1: Dry→Done"); }
  });
    fsmSub1.addTransition({ SubState::S_DONE,   Event::ResetPressed, SubState::S_IDLE,
    [](){ Serial.println("SUB1: Reset"); }
  });


  // Sub2: Idle → S_WET / S_DRY on init events
  fsmSub2.addTransition({ SubState::S_IDLE, Event::Shoe1InitWet, SubState::S_WET,
    [](){ Serial.println("SUB2: entered S_WET"); }
  });
  fsmSub2.addTransition({ SubState::S_IDLE, Event::Shoe1InitDry, SubState::S_DRY,
    [](){ Serial.println("SUB2: entered S_DRY"); }
  });

  // Sub2: example StartPressed path
  fsmSub2.addTransition({ SubState::S_WET,   Event::StartPressed, SubState::S_COOLING,
    [](){ Serial.println("SUB2: Wet→Cooling"); }
  });
  fsmSub2.addTransition({ SubState::S_COOLING,Event::StartPressed, SubState::S_DRY,
    [](){ Serial.println("SUB2: Cooling→Dry"); }
  });
  fsmSub2.addTransition({ SubState::S_DRY,   Event::StartPressed, SubState::S_DONE,
    [](){ Serial.println("SUB2: Dry→Done"); }
  });
  fsmSub2.addTransition({ SubState::S_DONE,   Event::ResetPressed, SubState::S_IDLE,
    [](){ Serial.println("SUB2: Reset"); }
  });
  
}

//------------------------------------------------------------------------------
// FreeRTOS task that drives all FSMs
//------------------------------------------------------------------------------
static void vStateMachineTask(void* pvParameters) {
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  setupStateMachines();
  Serial.println("FSM Task started");

  // Arrays to simplify per-shoe init
  StateMachine<SubState, Event>* subs[]   = { &fsmSub1, &fsmSub2 };
  Event                          initWet[] = { Event::Shoe0InitWet,  Event::Shoe1InitWet  };
  Event                          initDry[] = { Event::Shoe0InitDry,  Event::Shoe1InitDry  };

  while (true) {
    if (readStart()) {
      // 1) Fan StartPressed to Global (Idle→Running)
      broadcast(Event::StartPressed, fsmGlobal);

      // 2) For each shoe, pick wet/dry event based on your strings
      for (int idx = 0; idx < 2; ++idx) {
        bool isWet = (strcmp(g_dhtStatus[idx], "wet") == 0);
        subs[idx]->handleEvent(isWet ? initWet[idx] : initDry[idx]);
      }

      // 3) Optionally, drive each Sub-FSM one step from its new state
      broadcast(Event::StartPressed, fsmSub1, fsmSub2);
    }

    if (readReset()) {
      // Reset Global + both subs back to Idle
      broadcast(Event::ResetPressed, fsmGlobal, fsmSub1, fsmSub2);
    }

    // Continuously service the Global FSM (which polls subs)
    fsmGlobal.run();

    // Yield for 50 ms
    vTaskDelay(pdMS_TO_TICKS(300));
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