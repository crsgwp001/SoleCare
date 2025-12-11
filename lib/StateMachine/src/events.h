#pragma once

// system includes
#include <stdint.h>

// Events shared by global + sub-FSMs
enum class Event : uint32_t {
  None = 0,
  Error = 1 << 1,
  Debug = 1 << 2,
  ResetPressed = 1 << 3,
  StartPressed = 1 << 4,
  // Internal event used to auto-start sub-FSMs (distinct from the physical Start button)
  SubStart = 1 << 5,
  BatteryLow = 1 << 6,
  BatteryRecovered = 1 << 7,
  ChargeDetected = 1 << 8,
  SensorTimeout = 1 << 9,
  SubFSMDone = 1 << 10,
  Shoe0InitWet = 1 << 11,
  Shoe0InitDry = 1 << 12,
  Shoe1InitWet = 1 << 13,
  Shoe1InitDry = 1 << 14,
  UVTimer0 = 1 << 15,
  UVTimer1 = 1 << 16,
  DryCheckFailed = 1 << 17
};

// Global FSM states
enum class GlobalState : uint8_t {
  Idle = 0,
  Detecting,
  Checking,
  Running,
  Done,
  LowBattery,
  Error,
  Debug,
  Count
};

// Sub-FSM states
enum class SubState : uint8_t { S_IDLE, S_WAITING, S_WET, S_COOLING, S_DRY, S_DONE };

constexpr uint8_t NUM_GLOBAL_STATES = static_cast<uint8_t>(GlobalState::Count);
constexpr uint32_t ALL_STATE_BITS = (1UL << NUM_GLOBAL_STATES) - 1;