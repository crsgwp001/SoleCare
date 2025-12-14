#pragma once

#include <events.h>
#include <cstdint>

void createStateMachineTask(void);
// Post an Event into the FSM queue from other tasks
bool fsmExternalPost(Event ev);

// Query current FSM states (for UI)
GlobalState getGlobalState();
SubState getSub1State();
SubState getSub2State();

// Timing accessors for progress calculation (internal to FSM, read-only for UI)
uint32_t getSubWetStartMs(int shoeIdx);
uint32_t getSubCoolingStartMs(int shoeIdx);
uint32_t getCoolingMotorDurationMs(int shoeIdx);
