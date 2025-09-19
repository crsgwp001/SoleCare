#pragma once
#include <stdint.h>

// Events shared by global + sub-FSMs
enum class Event : uint32_t {
  None           = 0,
  Error          = 1 << 1,
  Debug          = 1 << 2,
  ResetPressed   = 1 << 3,
  StartPressed   = 1 << 4,
  // Internal event used to auto-start sub-FSMs (distinct from the physical Start button)
  SubStart       = 1 << 5,
  BatteryLow     = 1 << 5,
  BatteryReady   = 1 << 6,
  ChargeDetected = 1 << 7,
  SensorTimeout  = 1 << 8,
  SubFSMDone     = 1 << 9,
  Shoe0InitWet   = 1 << 10,
  Shoe0InitDry   = 1 << 11,
  Shoe1InitWet   = 1 << 12,
  Shoe1InitDry   = 1 << 13,
  UVTimer0       = 1 << 14,
  UVTimer1       = 1 << 15

};

// Global FSM states
enum class GlobalState : uint8_t {
  Idle = 0,
  Detecting,
  Checking,
  Running,
  Done,
  Error,
  Debug,
  Count
};

// Sub-FSM states
enum class SubState : uint8_t {
    S_IDLE,
    S_WET,
    S_DRYER,
    S_COOLING,
    S_DRY,
    S_DONE
};

constexpr uint8_t  NUM_GLOBAL_STATES = static_cast<uint8_t>(GlobalState::Count);
constexpr uint32_t ALL_STATE_BITS    = (1UL << NUM_GLOBAL_STATES) - 1;