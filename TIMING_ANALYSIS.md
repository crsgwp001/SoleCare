# FSM Timing Conflict Analysis

## Overview
This document outlines potential timing conflicts and issues discovered in the SoleCare FSM implementation that could cause state transitions, motor phase overlaps, or synchronization problems.

---

## 🔴 CRITICAL ISSUES

### 1. **Motor Phase Overlap in S_WAITING → S_WET Transition**
**Location:** [src/tskFSM.cpp](src/tskFSM.cpp#L750)

**Status:** REAL ISSUE - Motor/Heater overlap between COOLING (with re-evap) and WET

**The Real Problem:**

The WET lock prevents `S_WET ↔ S_WET` overlap, BUT it doesn't prevent:
- **SUB1 in S_COOLING (re-evap active: motor 85%, heater ON)**
- **SUB2 in S_WET (motor running, heater managed)**
- **BOTH running motors and heaters = power/thermal disaster** ⚠️

**Scenario:**
1. SUB1 finishes WET phase → releases WET lock → enters S_COOLING
2. SUB2 acquires WET lock → enters S_WET with motor + heater
3. SUB1's COOLING dry-check fails → triggers **re-evap**
4. Re-evap runs with **motor at 85% + heater ON**
5. **Both shoes now operating:** SUB1 (re-evap) + SUB2 (WET) = simultaneous motors + heaters

**Power Impact:**
```
SUB2 WET phase:
  - Motor: 60-100% (PID controlled)
  - Heater: ON (trend-gated)

SUB1 RE-EVAP phase:
  - Motor: 85% (fixed)
  - Heater: ON (fixed)

Combined: 2 motors (145-185% total) + 2 heaters
vs designed: Single motor + single heater
```

**Why `coolingMotorPhaseActive()` Check Isn't Enough:**

During re-evap in S_COOLING:
```cpp
coolingMotorPhaseActive(0) returns TRUE if:
  - g_subCoolingStartMs[0] != 0    ✅ (cooling started)
  - g_subCoolingStabilizeStartMs[0] == 0  ✅ (motor phase active, not stabilizing)
```

So the check **should** work... BUT there might be a window or timing issue where SUB2 reads the state before flags are updated.

**Risk Level:** HIGH - System not designed for dual motor + dual heater operation

**The Guard IS Necessary:**

The suggested defensive fix actually solves the real problem - preventing SUB2 from entering S_WET while SUB1 is anywhere in COOLING (including re-evap):

```cpp
fsmSub1.setRun(SubState::S_WAITING, []() {
    if (g_waitingEventPosted[0])
      return;
    
    // Prevent entering WET if other shoe is in COOLING
    // (which could have motor+heater active, including re-evap)
    if (coolingMotorPhaseActive(1))
      return;
    
    // Defensive: also prevent if other shoe just entered COOLING
    // but re-evap flags haven't been set yet (timing safety)
    if (fsmSub2.getState() == SubState::S_COOLING)  // ← Added this!
      return;
    
    // Now safe to check WET lock and acquire
    if (g_wetLockOwner == -1) {
      // ... rest of lock acquisition ...
    }
});
```

**Better Fix:** Check for ANY active motor (WET or COOLING with re-evap):
```cpp
if (coolingMotorPhaseActive(1) || fsmSub2.getState() == SubState::S_WET || fsmSub2.getState() == SubState::S_COOLING)
    return;  // Don't start if other shoe has motor running in any state
```

**Risk Level:** HIGH - Not a theoretical issue, real power/thermal overlap

---

### 2. **Race Condition: UV Timer Completion vs State Transition**
**Location:** [src/tskFSM.cpp](src/tskFSM.cpp#L650-L720) (DRY state entries)

**Problem:**
```cpp
{SubState::S_DRY,
 []() {
     FSM_DBG_PRINTLN("SUB2 ENTRY: DRY");
     motorStop(1);
     // Check if both shoes in DRY to start UV
     if (fsmSub1.getState() == SubState::S_DRY && !g_uvComplete[0] && !uvIsStarted(0)) {
         FSM_DBG_PRINTLN("SUB2 ENTRY: DRY -> Both shoes dry, starting single UV");
         uvStart(0, 0);
     }
 }
}
```

**Issue:** 
- UV timer is started from entry callback **without synchronization**
- If `uvStart()` is called twice rapidly (race condition), UV might be restarted, cutting drying time short
- No atomic check-and-set of `g_uvComplete[0]`

**Risk Level:** HIGH - UV drying could be incomplete

**Recommended Fix:**
- Use atomic operations or a mutex for UV state
- Add immediate flag set after `uvStart()` to prevent re-entry

---

### 3. **Heater Warmup Phase: Incomplete Timing Boundary**
**Location:** [src/tskFSM.cpp](src/tskFSM.cpp#L800-L820)

**Problem:**
```cpp
uint32_t warmupElapsed = now - g_heaterWarmupStartMs[0];
uint32_t targetWarmupMs = HEATER_WARMUP_MIN_MS;  // 30s default

// End warmup immediately if threshold temp reached
if (!isnan(shoeTemp) && shoeTemp >= HEATER_WET_TEMP_THRESHOLD_C) {
    g_heaterWarmupDone[0] = true;
    motorStart(0);  // PID initialization
    // ... reset tracking ...
} else if (warmupElapsed < targetWarmupMs) {
    return;  // Skip rest of WET logic
} else {
    // Time-based completion fallback
    g_heaterWarmupDone[0] = true;
    motorStart(0);
    // ...
}
```

**Issue:**
- **Time boundary gap**: Between `targetWarmupMs` expiry and the `else` clause executing, there's no state transition
- If temperature is stuck below threshold at exactly `30s`, the code might:
  - Not return (fail `warmupElapsed < targetWarmupMs`)
  - Fall through to `else` → call `motorStart(0)` twice
  - Multiple PID initializations could corrupt control state

**Risk Level:** MEDIUM - PID initialization repeated, causing erratic motor duty

**Recommended Fix:**
```cpp
if (!isnan(shoeTemp) && shoeTemp >= HEATER_WET_TEMP_THRESHOLD_C) {
    // Temp threshold met
    g_heaterWarmupDone[0] = true;
    motorStart(0);
} else if (warmupElapsed >= targetWarmupMs) {
    // Time threshold met - explicit transition
    g_heaterWarmupDone[0] = true;
    motorStart(0);
} else {
    // Still warming up
    return;
}
```

---

### 4. **COOLING Phase: Temperature-Based Motor Extension Without Upper Bound**
**Location:** [src/tskFSM.cpp](src/tskFSM.cpp#L1540-L1570)

**Problem:**
```cpp
if (!isnan(tempC0) && tempC0 > targetC0) {
    if (motorElapsed < g_coolingMotorDurationMs[0] + COOLING_TEMP_EXTEND_MAX_MS) {
        // Adjust motor duty and RETURN
        // ... (no transition out of COOLING) ...
        return;
    }
    // Max extension reached: force stabilization
}
```

**Issue:**
- `COOLING_TEMP_EXTEND_MAX_MS = 120s` is the **only upper bound** for motor phase
- If shoe temperature **never drops below target**, motor runs for entire `DRY_COOL_MS_BASE (90s) + 120s = 210s`
- No stabilization timeout if temperature stalls
- Could cause FreeRTOS watchdog timeout (typically 5-60s per task)

**Risk Level:** CRITICAL - Watchdog timeout → ESP32 reboot possible

**Current Logic:**
```
COOLING phase timing:
Motor runs: 0-90s (base)
If hot: 0-210s (base + extension max)
Stabilization: ? (unconstrained if temp stays high)
Total worst case: >300s
```

**Recommended Fix:**
```cpp
#define COOLING_MOTOR_ABSOLUTE_MAX_MS = 240u * 1000u  // 4 min hard limit

if (motorElapsed >= COOLING_MOTOR_ABSOLUTE_MAX_MS) {
    FSM_DBG_PRINTLN("COOLING: Hard motor timeout (safety watchdog)");
    motorStop(idx);
    g_subCoolingStabilizeStartMs[idx] = millis();
    return;
}
```

---

### 5. **WET Phase: Missing Timeout for Stalled Peak Detection**
**Location:** [src/tskFSM.cpp](src/tskFSM.cpp#L900-L1100)

**Problem:**
```cpp
// Adaptive peak detection times:
// Barely wet: 60s → peak
// Moderate: 120s → peak
// Very wet: 180s → peak
// Soaked: 240s → peak

// BUT: If peak never detected (e.g., dry environment):
// No upper bound on WET phase!
```

**Issue:**
- `g_wetMinDurationMs[idx]` sets **minimum** WET time
- No **maximum** WET time if peak never detected
- If AH diff decreases slowly (no clear peak), WET phase continues indefinitely
- Heater stays ON → thermal runaway possible
- Motor runs continuously → power drain

**Risk Level:** HIGH - Indefinite heating/motor runtime

**Current WET Durations:**
| Wetness | Min | Max |
|---------|-----|-----|
| Barely | 180s | ∞ |
| Moderate | 360s | ∞ |
| Very | 480s | ∞ |
| Soaked | 600s | ∞ |

**Recommended Fix:**
```cpp
// Add maximum WET phase limits to config.h:
constexpr uint32_t WET_BARELY_WET_MAX_MS = 300u * 1000u;      // 5 min max
constexpr uint32_t WET_MODERATE_MAX_MS = 540u * 1000u;        // 9 min max
constexpr uint32_t WET_VERY_WET_MAX_MS = 720u * 1000u;        // 12 min max
constexpr uint32_t WET_SOAKED_MAX_MS = 900u * 1000u;          // 15 min max

// In WET run callback:
if (wetElapsed >= g_wetMaxDurationMs[idx]) {
    FSM_DBG_PRINT("WET: Timeout (no peak detected after ");
    FSM_DBG_PRINT(wetElapsed/1000); FSM_DBG_PRINTLN("s) -> COOLING");
    heaterRun(idx, false);
    motorStop(idx);
    startCoolingPhase(idx, false);
    return;
}
```

---

### 6. **Re-Evaporation Short Cycle: No Guard Against Infinite Loop**
**Location:** [src/tskFSM.cpp](src/tskFSM.cpp#L1460-L1500)

**Problem:**
```cpp
if (g_inReEvap[0]) {
    // ... (heater ON, motor at duty) ...
    
    bool timeout = (elapsed >= RE_EVAP_MAX_MS);           // 60s max
    bool risePassed = (elapsed >= minTime) && (d - g_reEvapMinDiff[0] > riseThresh);
    
    if (timeout || risePassed) {
        FSM_DBG_PRINT("SUB1: RE-EVAP done ...");
        g_inReEvap[0] = false;
        startCoolingPhase(0, true);  // Restart COOLING (retry=true)
        return;
    }
    return;  // Stay in re-evap
}
```

**Issue:**
- If re-evap never reaches rise threshold **and** timeout fires, restarts COOLING with `isRetry=true`
- If COOLING transitions back to WET (via `DryCheckFailed`), could trigger re-evap **again**
- Potential infinite loop: `COOLING → WET → COOLING → WET → ...`
- No retry count limit across cycles

**Risk Level:** MEDIUM - Battery drain from infinite cycling

**Recommended Fix:**
```cpp
// In global state tracking:
static uint8_t g_reEvapRetryCount[2] = {0, 0};  // Limit retries per cycle
constexpr int MAX_RE_EVAP_RETRIES = 2;

// In COOLING handler:
if (timeout) {
    if (g_coolingRetryCount[idx] >= MAX_RE_EVAP_RETRIES) {
        FSM_DBG_PRINTLN("Max re-evap retries reached -> forcing DRY");
        motorStop(idx);
        fsmSubX.handleEvent(Event::SubStart);  // Force to DRY
        return;
    }
    startCoolingPhase(idx, true);
}
```

---

### 7. **Done State: Timeout Uses DONE_TIMEOUT_MS (10s, not 60s)**
**Location:** [src/tskFSM.cpp](src/tskFSM.cpp#L2145-2155), [include/config.h](include/config.h#L16)

**Finding:**
✅ **WORKING CORRECTLY** - Auto-reset IS implemented and working.

The timeout uses `DONE_TIMEOUT_MS` from config:
```cpp
constexpr uint32_t DONE_TIMEOUT_MS = 10u * 1000u;  // 10 seconds, NOT 60
```

Actual implementation in main loop:
```cpp
if (g_doneStartMs != 0) {
  uint32_t now = millis();
  if ((uint32_t)(now - g_doneStartMs) >= DONE_TIMEOUT_MS) {
    FSM_DBG_PRINTLN("GLOBAL: Done timeout -> Reset to Idle");
    fsmPostEvent(Event::ResetPressed, /*broadcastAll=*/false);
    g_doneStartMs = 0;
  }
}
```

**Root Cause of 5s vs Docs:**
- You're seeing ~5 seconds in testing
- Config says 10 seconds
- Old documentation says 60 seconds (incorrect!)
- The **implementation works**, documentation is just outdated/wrong

**Risk Level:** LOW - Implementation is correct, docs are misleading

**Fix:** Update misleading comment in config to clarify actual timeout value

---

## 🟡 MEDIUM PRIORITY ISSUES

### 8. **Stabilization Phase: Inconsistent Sample Timing**
**Location:** [src/tskFSM.cpp](src/tskFSM.cpp#L1580-1620)

**Problem:**
- Stabilization samples AH diff every `~1s` (FSM loop runs every 50ms)
- If FSM loop stalls, sample intervals become irregular
- Leads to inconsistent decline detection

**Fix:** Implement explicit timing in sampling:
```cpp
if ((now - g_coolingStabilizeLastSampleMs[idx]) >= 1000) {
    g_coolingStabilizeLastSampleMs[idx] = now;
    // Take sample...
}
```

---

### 9. **Sensor Equalization: Only Checks on Global Entry**
**Location:** [src/tskFSM.cpp](src/tskFSM.cpp#L33)

**Problem:**
```cpp
constexpr uint32_t SENSOR_EQUALIZE_MS = 6u * 1000u;  // 6s
// But where is the timeout handler?
```

**Issue:** 6s timeout for sensor equalization may not be enforced if event never fires

---

### 10. **Battery Voltage Check: Only in Checking State**
**Location:** [src/tskFSM.cpp](src/tskFSM.cpp#L1910)

**Problem:**
- Battery voltage only checked in Checking state
- No periodic check during Running state
- If battery dies mid-cycle, system won't detect it until too late

**Fix:** Add periodic battery monitoring in Running state

---

## 📊 Timing Constraint Summary

| Phase | Min Duration | Max Duration | Current Max | Issue |
|-------|--------------|--------------|-------------|-------|
| Warmup | 30-50s | 50s | 50s | ✅ OK |
| WET | 180-600s | ∞ | **UNBOUNDED** | ⚠️ No max |
| COOLING Motor | 90-180s | 210s | 210s | ✅ OK (marginal) |
| COOLING Stab | 90s | 90s | 90s | ✅ OK |
| Re-evap | 0-60s | 60s | 60s | ✅ OK |
| UV Dry | 57s | 57s | 57s | ✅ OK |
| Done | Auto-reset | 60s | **UNCHECKED** | ⚠️ Not enforced |

---

## 🔧 Recommended Priority Fixes (Revised Again)

1. **Add motor overlap guard** (Issue #1) - CRITICAL (Prevents COOLING/re-evap + WET dual operation)
2. **Enforce COOLING motor hard timeout** (Issue #4) - CRITICAL  
3. **Add WET phase maximum duration** (Issue #5) - HIGH
4. **Fix heater warmup boundary** (Issue #3) - HIGH
5. **Implement Done auto-reset timeout** (Issue #7) - MEDIUM
6. **Add UV state atomic protection** (Issue #2) - MEDIUM
7. **Limit re-evap retries** (Issue #6) - MEDIUM

---

## Testing Recommendations

1. **Run extended drying cycles** (>30 min) to verify no infinite loops
2. **Simulate slow evaporation** (manually reduce AH rate) to verify WET timeout
3. **Monitor ESP32 temperature** during COOLING to ensure no watchdog resets
4. **Check for simultaneous motor operation** using current sensor
5. **Validate UV timing** by measuring actual drying time vs expected

---

## Configuration Changes Needed

Add to `include/config.h`:
```cpp
// WET phase maximum durations (safety limits)
constexpr uint32_t WET_BARELY_WET_MAX_MS = 300u * 1000u;
constexpr uint32_t WET_MODERATE_MAX_MS = 540u * 1000u;
constexpr uint32_t WET_VERY_WET_MAX_MS = 720u * 1000u;
constexpr uint32_t WET_SOAKED_MAX_MS = 900u * 1000u;

// COOLING motor absolute maximum (watchdog safety)
constexpr uint32_t COOLING_MOTOR_ABSOLUTE_MAX_MS = 240u * 1000u;

// Re-evap retry limit
constexpr int MAX_RE_EVAP_RETRIES = 2;

// Done state timeout enforcement
constexpr uint32_t DONE_STATE_RESET_MS = 60u * 1000u;
```
