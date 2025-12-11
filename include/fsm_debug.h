// Lightweight FSM debug helper. Define FSM_DEBUG in your build to enable prints.
#pragma once
#ifdef FSM_DEBUG
#include <Arduino.h>
#include <events.h>
// Variadic macros let us forward formatting args (eg Serial.print(val, digits)).
#define FSM_DBG_PRINT(...)                                                                         \
  do {                                                                                             \
    if (Serial)                                                                                    \
      Serial.print(__VA_ARGS__);                                                                   \
  } while (0)
#define FSM_DBG_PRINTLN(...)                                                                       \
  do {                                                                                             \
    if (Serial)                                                                                    \
      Serial.println(__VA_ARGS__);                                                                 \
  } while (0)
#define FSM_DBG_PRINT_INT(prefix, v)                                                               \
  do {                                                                                             \
    if (Serial) {                                                                                  \
      Serial.print(prefix);                                                                        \
      Serial.println((int)(v));                                                                    \
    }                                                                                              \
  } while (0)
// Optional helpers: human-readable names for events/states. Use like:
// FSM_DBG_PRINTLN(eventName(ev));
inline const char *eventName(Event e) {
  uint32_t v = static_cast<uint32_t>(e);
  if (v == static_cast<uint32_t>(Event::None))
    return "None";
  if (v == static_cast<uint32_t>(Event::Error))
    return "Error";
  if (v == static_cast<uint32_t>(Event::Debug))
    return "Debug";
  if (v == static_cast<uint32_t>(Event::ResetPressed))
    return "ResetPressed";
  if (v == static_cast<uint32_t>(Event::StartPressed))
    return "StartPressed";
  if (v == static_cast<uint32_t>(Event::SubStart))
    return "SubStart";
  if (v == static_cast<uint32_t>(Event::BatteryLow))
    return "BatteryLow";
  if (v == static_cast<uint32_t>(Event::BatteryRecovered))
    return "BatteryRecovered";
  if (v == static_cast<uint32_t>(Event::ChargeDetected))
    return "ChargeDetected";
  if (v == static_cast<uint32_t>(Event::SensorTimeout))
    return "SensorTimeout";
  if (v == static_cast<uint32_t>(Event::SubFSMDone))
    return "SubFSMDone";
  if (v == static_cast<uint32_t>(Event::Shoe0InitWet))
    return "Shoe0InitWet";
  if (v == static_cast<uint32_t>(Event::Shoe0InitDry))
    return "Shoe0InitDry";
  if (v == static_cast<uint32_t>(Event::Shoe1InitWet))
    return "Shoe1InitWet";
  if (v == static_cast<uint32_t>(Event::Shoe1InitDry))
    return "Shoe1InitDry";
  return "Event(unknown)";
}

inline const char *globalStateName(GlobalState s) {
  switch (s) {
  case GlobalState::Idle:
    return "Idle";
  case GlobalState::Detecting:
    return "Detecting";
  case GlobalState::Checking:
    return "Checking";
  case GlobalState::Running:
    return "Running";
  case GlobalState::Done:
    return "Done";
  case GlobalState::LowBattery:
    return "LowBattery";
  case GlobalState::Error:
    return "Error";
  case GlobalState::Debug:
    return "Debug";
  default:
    return "GlobalState(unknown)";
  }
}

inline const char *subStateName(SubState s) {
  switch (s) {
  case SubState::S_IDLE:
    return "S_IDLE";
  case SubState::S_WAITING:
    return "S_WAITING";
  case SubState::S_WET:
    return "S_WET";
  case SubState::S_COOLING:
    return "S_COOLING";
  case SubState::S_DRY:
    return "S_DRY";
  case SubState::S_DONE:
    return "S_DONE";
  default:
    return "SubState(unknown)";
  }
}
#else
#define FSM_DBG_PRINT(msg) ((void)0)
#define FSM_DBG_PRINTLN(msg) ((void)0)
#define FSM_DBG_PRINT_INT(prefix, v) ((void)0)
#endif
