# Motor Lock Refactoring Proposal

## Current Problem
- Lock is named `g_wetLockOwner` but its actual purpose is preventing **simultaneous motor operation**
- Re-evap doesn't acquire the lock, so it can run while another shoe is in S_WET
- Defensive state checks are needed to prevent the overlap

## Proposed Solution
Rename `g_wetLockOwner` → `g_motorLockOwner` and have **re-evap acquire it too**

## Implementation Details

### 1. Rename Global Variable
```cpp
// OLD:
static int g_wetLockOwner = -1;  // -1 = free, 0 = SUB1, 1 = SUB2

// NEW:
static int g_motorLockOwner = -1;  // -1 = free, 0 = SUB1, 1 = SUB2
// Tracks which shoe owns exclusive motor (used by S_WET and re-evap phases)
```

### 2. Update S_WET Entry (no change needed)
```cpp
fsmSub1.setRun(SubState::S_WAITING, []() {
    if (g_waitingEventPosted[0])
      return;
    
    // Simplified: just check if lock is free (works for both WET and re-evap)
    if (g_motorLockOwner == -1) {
      // Priority-based acquisition...
      g_motorLockOwner = 0;
      fsmSub1.handleEvent(Event::SubStart);
    }
});
```

### 3. Update S_WET Exit (rename only)
```cpp
[]() {  // S_WET exit callback
    if (g_motorLockOwner == 0) {
      g_motorLockOwner = -1;  // Release motor lock
    }
}
```

### 4. **New: Re-Evap Acquires Motor Lock**
```cpp
// In S_COOLING handler, when re-evap is triggered:
if (stillWet) {
    // Acquire motor lock to prevent other shoe from entering S_WET
    g_motorLockOwner = 0;  // SUB1 claiming motor during re-evap
    g_inReEvap[0] = true;
    g_reEvapStartMs[0] = millis();
    // ... rest of re-evap init ...
    return;
}
```

### 5. **New: Re-Evap Releases Motor Lock**
```cpp
// In re-evap cleanup:
if (timeout || risePassed) {
    g_inReEvap[0] = false;
    // Release motor lock since we're done with motor
    g_motorLockOwner = -1;
    // Start normal COOLING (no motor, just temperature monitoring)
    startCoolingPhase(0, true);
    return;
}
```

### 6. Simplify S_WAITING Guard
```cpp
fsmSub1.setRun(SubState::S_WAITING, []() {
    if (g_waitingEventPosted[0])
      return;
    
    // Single check: is motor available?
    if (g_motorLockOwner == -1) {
      // Lock acquisition logic (both shoes use same mechanism)
      if (fsmSub2.getState() == SubState::S_WAITING) {
        // Priority-based acquisition
        float diff0 = g_dhtAHDiff[0];
        float diff1 = g_dhtAHDiff[1];
        if (diff0 >= diff1) {
          g_motorLockOwner = 0;
          g_waitingEventPosted[0] = true;
          fsmSub1.handleEvent(Event::SubStart);
        }
      } else {
        g_motorLockOwner = 0;
        g_waitingEventPosted[0] = true;
        fsmSub1.handleEvent(Event::SubStart);
      }
    }
});
```

## Benefits

| Aspect | Before | After |
|--------|--------|-------|
| **Architecture** | State checks + lock | Unified motor lock |
| **Re-evap overlap prevention** | Defensive guards needed | Automatic via lock |
| **Code clarity** | Lock called "wet" but controls motor | Lock properly named "motor" |
| **Mutual exclusion** | Weak (gaps during transitions) | Strong (lock enforces it) |
| **Extensibility** | Hard to add more phases | Easy to add phases that need motor |

## Flow with Motor Lock

```
SUB1 in S_WET:
  ├─ Acquires motor lock (g_motorLockOwner = 0)
  ├─ Motor + heater running
  └─ SUB2 blocked from S_WAITING (lock held)

SUB1 exits WET → enters COOLING:
  ├─ Releases motor lock
  ├─ SUB2 can now acquire lock and enter S_WET
  └─ Dry-check fails → re-evap triggered

SUB1 re-evap (in S_COOLING):
  ├─ Re-acquires motor lock (g_motorLockOwner = 0)
  ├─ Motor 85% + heater ON
  └─ SUB2 BLOCKED from S_WAITING (lock held by re-evap!)

Re-evap complete:
  ├─ Releases motor lock
  └─ SUB2 can now acquire lock
```

## Migration Checklist

- [ ] Rename `g_wetLockOwner` → `g_motorLockOwner` (all 20+ references)
- [ ] Update comment to clarify purpose: "Prevents simultaneous motor operation"
- [ ] Add motor lock acquisition to re-evap init
- [ ] Add motor lock release to re-evap completion
- [ ] Remove defensive state checks from S_WAITING (simplified!)
- [ ] Update CHANGELOG documenting the refactoring
- [ ] Test sequential operation: WET → COOLING re-evap → dry-check → next shoe

## Code Changes Summary

**Files to modify:**
- `src/tskFSM.cpp` (20+ locations for rename + re-evap lock handling)

**Lines to add:**
- ~5 lines for motor lock acquire in re-evap init
- ~5 lines for motor lock release in re-evap cleanup

**Lines to remove:**
- ~3 lines of defensive `|| fsmSub2.getState() == SubState::S_COOLING` checks

**Net change:** ~7 new lines, cleaner architecture
