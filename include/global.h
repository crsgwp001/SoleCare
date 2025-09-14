// global.h
#pragma once

#include "config.h"
#include <Arduino.h>
#include <type_traits>
#include <math.h>
#include <utility>

// raw temperature and humidity readings
extern float g_dhtTemp[3];
extern float g_dhtHum[3];

// instantaneous absolute humidity
extern float g_dhtAH[3];

// differences relative to sensor 0 (two diffs)
extern float g_dhtAHDiff[2];

// EMA-filtered absolute humidity and diffs
extern float g_dhtAH_ema[3];
extern float g_dhtAHDiff_ema[2];

// wet/dry/normal status for sensors 1 and 2
extern char  g_dhtStatus[2][5];

#include <type_traits>

// 1) hasFlag
template<typename E>
inline typename std::enable_if<std::is_enum<E>::value, bool>::type
hasFlag(E value, E flag) {
    typedef typename std::underlying_type<E>::type U;
    return (static_cast<U>(value) & static_cast<U>(flag)) != 0;
}

// 2) bitwise OR
template<typename E>
inline typename std::enable_if<std::is_enum<E>::value, E>::type
operator|(E lhs, E rhs) {
    typedef typename std::underlying_type<E>::type U;
    return static_cast<E>(static_cast<U>(lhs) | static_cast<U>(rhs));
}

// 3) bitwise AND
template<typename E>
inline typename std::enable_if<std::is_enum<E>::value, E>::type
operator&(E lhs, E rhs) {
    typedef typename std::underlying_type<E>::type U;
    return static_cast<E>(static_cast<U>(lhs) & static_cast<U>(rhs));
}

// 4) bitwise NOT
template<typename E>
inline typename std::enable_if<std::is_enum<E>::value, E>::type
operator~(E value) {
    typedef typename std::underlying_type<E>::type U;
    return static_cast<E>(~static_cast<U>(value));
}