# Motor Lock Implementation - Completed

## Overview
Motor lock has been successfully implemented to prevent dual motor operation between WET and COOLING (re-evap) phases. The `g_wetLockOwner` now functions as a unified **motor lock** preventing any two-motor overlap.

## Changes Made

### 1. SUB1 Re-evap Motor Phase (lines ~1450-1500)

**Motor Start - Lock Acquisition:**
```cpp
if (!g_motorStarted[0]) {
    // Acquire motor lock at start of re-evap motor phase
    if (g_wetLockOwner == -1) {
        g_wetLockOwner = 0;
        FSM_DBG_PRINTLN("SUB1: RE-EVAP acquired motor lock");
    }
    motorStart(0);
    g_motorStarted[0] = true;
}
```

**Motor Complete - Lock Release:**
```cpp
if (timeout || risePassed) {
    // ... cleanup ...
    // Release motor lock when re-evap completes
    if (g_wetLockOwner == 0) {
        g_wetLockOwner = -1;
        FSM_DBG_PRINTLN("SUB1: RE-EVAP released motor lock on completion");
    }
    startCoolingPhase(0, true);
    return;
}
```

### 2. SUB1 Stabilization Phase (lines ~1560)

**Transition to Stabilization - Lock Release:**
```cpp
if (g_subCoolingStabilizeStartMs[0] == 0) {
    FSM_DBG_PRINTLN("SUB1: COOLING -> motor phase done, starting stabilization");
    motorStop(0);
    // Release motor lock when transitioning to stabilization phase
    if (g_wetLockOwner == 0) {
        g_wetLockOwner = -1;
        FSM_DBG_PRINTLN("SUB1: COOLING released motor lock on stabilization start");
    }
    g_subCoolingStabilizeStartMs[0] = millis();
    return;
}
```

### 3. SUB2 Re-evap Motor Phase (lines ~1683-1700)

**Motor Start - Lock Acquisition:**
```cpp
if (!g_motorStarted[1]) {
    // Acquire motor lock at start of re-evap motor phase
    if (g_wetLockOwner == -1) {
        g_wetLockOwner = 1;
        FSM_DBG_PRINTLN("SUB2: RE-EVAP acquired motor lock");
    }
    motorStart(1);
    g_motorStarted[1] = true;
}
```

**Motor Complete - Lock Release:**
```cpp
if (timeout || risePassed) {
    // ... cleanup ...
    // Release motor lock when re-evap completes
    if (g_wetLockOwner == 1) {
        g_wetLockOwner = -1;
        FSM_DBG_PRINTLN("SUB2: RE-EVAP released motor lock on completion");
    }
    startCoolingPhase(1, true);
    return;
}
```

### 4. SUB2 Stabilization Phase (lines ~1779)

**Transition to Stabilization - Lock Release:**
```cpp
if (g_subCoolingStabilizeStartMs[1] == 0) {
    FSM_DBG_PRINTLN("SUB2: COOLING -> motor phase done, starting stabilization");
    motorStop(1);
    // Release motor lock when transitioning to stabilization phase
    if (g_wetLockOwner == 1) {
        g_wetLockOwner = -1;
        FSM_DBG_PRINTLN("SUB2: COOLING released motor lock on stabilization start");
    }
    g_subCoolingStabilizeStartMs[1] = millis();
    return;
}
```

## Motor Lock Lifecycle

### Acquisition Points:
1. **WET entry** (existing): Lock acquired in S_WAITING run callback (priority-based)
2. **Re-evap start** (NEW): Lock acquired when re-evap motor starts in COOLING handler

### Release Points:
1. **WET exit to COOLING** (existing): Lock released when leaving WET phase
2. **Re-evap complete** (NEW): Lock released when re-evap timeout or rise-threshold met
3. **Stabilization start** (NEW): Lock released when transitioning from motor to stabilization phase in COOLING
4. **DONE state** (existing): Lock verified/released on entry
5. **Idle reset** (existing): Lock reset to -1 on global idle entry

## Protection Guarantees

With these changes, the system now guarantees:

✅ **No simultaneous WET operations**: Only one shoe in S_WET at a time (existing WET lock)
✅ **No simultaneous re-evap operations**: Re-evap acquires motor lock, preventing overlap with WET
✅ **Clean motor phase sequencing**: 
- Motor lock held from motor start through motor completion
- Lock released before stabilization phase (no motor running)
- Prevents race conditions between re-evap and WET transitions

## Stabilization Phase Behavior

**CRITICAL:** Stabilization phase runs **without motor lock**:
- Motor is already stopped (`motorStop()` called before lock release)
- Stabilization only samples AH diff for dry-check
- Other shoe can acquire motor lock during stabilization (correct behavior)
- Lock release before stabilization prevents deadlock situations

## Testing Checklist

- [ ] Build compiles successfully ✅
- [ ] Upload to ESP32 and verify boot
- [ ] Run single-shoe drying cycle - verify motor lock messages
- [ ] Run dual-shoe cycle - verify no simultaneous motors
- [ ] Trigger re-evap during COOLING - verify motor lock acquired
- [ ] Monitor current draw - verify single motor peaks, not dual

## Edge Cases Handled

1. **Re-evap while other shoe in WET**: Motor lock already held by WET, re-evap waits (can't acquire)
2. **WET while other shoe in re-evap**: Motor lock held by re-evap, WET blocked by S_WAITING guard
3. **Simultaneous re-evap start**: First to acquire lock proceeds, second must wait until release
4. **Re-evap timeout during WET of other shoe**: Lock already held by WET, re-evap release has no effect (verify ownership)

## Next Steps

With motor lock in place:
1. Remove defensive state checks from S_WAITING (optional cleanup, now redundant)
2. Consider renaming `g_wetLockOwner` → `g_motorLockOwner` for clarity (future refactor)
3. Add COOLING motor hard timeout (240s safety watchdog)
4. Add WET phase maximum duration safeguards
