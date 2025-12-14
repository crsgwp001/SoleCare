// global.h — shared global symbols
#pragma once

// Includes
#include "config.h"
#include <Arduino.h>
#include <cmath>
#include <type_traits>

// Ambient AH offset alias (from config)
static constexpr float kAmbAhOffset = AMB_AH_OFFSET;

// Raw temperature and humidity readings (indexed by sensor)
extern volatile float g_dhtTemp[3];
extern volatile float g_dhtHum[3];

// Instantaneous absolute humidity (g/m^3)
extern volatile float g_dhtAH[3];

// Differences relative to sensor 0 (two diffs: sensor1 - sensor0, sensor2 - sensor0)
extern volatile float g_dhtAHDiff[2];

// EMA-filtered absolute humidity and diffs
extern volatile float g_dhtAH_ema[3];
extern volatile float g_dhtAHDiff_ema[2];

// Boolean wet/dry status for sensors 1 and 2 (true = wet)
extern volatile bool g_dhtIsWet[2];

// AH rate-of-change in g/m³/min for sensors 1 and 2 (calculated by motor task)
extern float g_dhtAHRate[2];

// Cached battery voltage (updated during Checking state and at boot)
extern volatile float g_lastBatteryVoltage;

// Battery voltage monitoring
float readBatteryVoltage();
bool isBatteryOk();
bool isBatteryRecovered();

// LED status control
void updateStatusLEDs();

// Enum bit helpers: enable bitwise ops for enum types. These are small helpers
// and only enabled for enum types via SFINAE.
template <typename E>
inline typename std::enable_if<std::is_enum<E>::value, bool>::type hasFlag(E value, E flag) {
  using U = typename std::underlying_type<E>::type;
  return (static_cast<U>(value) & static_cast<U>(flag)) != 0;
}

template <typename E>
inline typename std::enable_if<std::is_enum<E>::value, E>::type operator|(E lhs, E rhs) {
  using U = typename std::underlying_type<E>::type;
  return static_cast<E>(static_cast<U>(lhs) | static_cast<U>(rhs));
}

template <typename E>
inline typename std::enable_if<std::is_enum<E>::value, E>::type operator&(E lhs, E rhs) {
  using U = typename std::underlying_type<E>::type;
  return static_cast<E>(static_cast<U>(lhs) & static_cast<U>(rhs));
}

template <typename E>
inline typename std::enable_if<std::is_enum<E>::value, E>::type operator~(E value) {
  using U = typename std::underlying_type<E>::type;
  return static_cast<E>(~static_cast<U>(value));
}