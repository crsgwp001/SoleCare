# WET Lock Analysis & Re-Evap Interaction

## 1. WET Lock Mechanism - Does It Prevent Dual Motors?

### Overview
YES, there IS a WET lock mechanism (`g_wetLockOwner`), but it has a **critical gap**.

### How the WET Lock Works

**Lock State:**
```cpp
static int g_wetLockOwner = -1;  // -1 = free, 0 = SUB1 owns it, 1 = SUB2 owns it
```

**Acquisition Logic (S_WAITING state):**
```cpp
fsmSub1.setRun(SubState::S_WAITING, []() {
    if (g_waitingEventPosted[0])  // Guard: don't re-post
      return;
    
    if (coolingMotorPhaseActive(1))  // ⚠️ Only checks COOLING phase
      return;
    
    if (g_wetLockOwner == -1) {  // Lock is free
      if (fsmSub2.getState() == SubState::S_WAITING) {
        // Both waiting: priority-based acquisition
        if (diff0 >= diff1) {
          g_wetLockOwner = 0;  // SUB1 acquires lock
          fsmSub1.handleEvent(Event::SubStart);  // Transition to S_WET
        }
      } else {
        // SUB2 not waiting: auto-acquire
        g_wetLockOwner = 0;
        fsmSub1.handleEvent(Event::SubStart);
      }
    }
});
```

**Release Logic (S_WET exit):**
```cpp
[]() {  // S_WET exit callback
    if (g_wetLockOwner == 0) {
      g_wetLockOwner = -1;  // Release lock
      FSM_DBG_PRINTLN("SUB1: Released WET lock on exit to COOLING");
    }
}
```

### The Critical Gap

The check at line 755 **only guards COOLING motor phase**:
```cpp
if (coolingMotorPhaseActive(1))  // Prevents COOLING overlap
  return;

// ⚠️ But DOES NOT check:
// - if (fsmSub2.getState() == SubState::S_WET)  // Missing!
```

### Failure Scenario

**Timeline:**
1. **T=0s:** SUB1 acquires WET lock (`g_wetLockOwner = 0`)
   - Transitions S_WAITING → S_WET
   - Starts motor at 60% warmup
   
2. **T=0s:** SUB2 in S_WAITING waiting for lock
   - Checks: `coolingMotorPhaseActive(1)` → FALSE (SUB1 is in WET, not COOLING)
   - Checks: `g_wetLockOwner == -1` → FALSE (SUB1 owns it)
   - Returns, waits...

3. **T=35s:** SUB1's warmup phase ends → calls `motorStart(0)` for PID
   - Motor now running at ~75% under PID control
   - SUB1 STILL in S_WET with motor active

4. **T=35s:** SUB2's S_WAITING run fires again
   - Same checks:
     - `coolingMotorPhaseActive(1)` → FALSE (SUB1 not in COOLING, still in WET!)
     - `g_wetLockOwner == -1` → FALSE (still owned by SUB1)
   - Returns, continues waiting... ✅ **Lock DOES prevent acquisition**

**Actually, the lock WORKS!**
The `g_wetLockOwner` check prevents SUB2 from acquiring while SUB1 is in S_WET. SUB2 will stay in S_WAITING until SUB1 exits to S_COOLING and releases the lock.

So my original analysis was **incorrect** - the WET lock DOES prevent the dual-motor scenario in normal operation.

---

## 2. Re-Evap Interaction with WET State

### What Happens During Re-Evap?

Re-evap occurs in **COOLING phase**, not WET phase:

```cpp
fsmSub1.setRun(SubState::S_COOLING, []() {
    if (g_inReEvap[0]) {  // Already in RE-EVAP
      // Heater ON, motor at fixed 85% duty
      // Looking for "rise-from-minimum" condition or timeout
      if (timeout || risePassed) {
        g_inReEvap[0] = false;
        startCoolingPhase(0, true);  // Restart COOLING
        return;
      }
      return;
    }
    // ... rest of COOLING logic ...
});
```

### The Re-Evap Loop Problem

**Where Re-Evap Can Get Triggered:**
1. Stabilization phase completes → dry-check fails (still wet)
2. Code decides to do re-evap instead of retrying full COOLING cycle:
   ```cpp
   bool stillWet = (evalDiff > threshold);
   if (stillWet) {
     // Try re-evap short cycle instead
     g_inReEvap[0] = true;
     g_reEvapStartMs[0] = millis();
   }
   ```

**What Happens in Re-Evap:**
```cpp
if (g_inReEvap[0]) {
    // Heater ON (aggressive mode)
    // Motor at 85% (RE_EVAP_MOTOR_DUTY)
    // Stays here for up to 60s waiting for moisture to rise
    
    if (timeout || risePassed) {
      g_inReEvap[0] = false;
      startCoolingPhase(0, true);  // Restart COOLING with retry flag
      return;  // ← Still in S_COOLING!
    }
}
// ⚠️ Never exits S_COOLING state during re-evap!
```

### Critical Finding: WET State is NEVER Re-Entered During Re-Evap

The flow is:
```
S_WET → S_COOLING (motor off, heater off)
              ↓
        Re-evap starts (in COOLING)
        (heater ON, motor 85%)
              ↓
        Re-evap timeout/condition met
              ↓
        startCoolingPhase(0, true)
              ↓
        Still in S_COOLING (never goes back to S_WET!)
```

**Key Point:** During re-evap, the system is **still in S_COOLING state**, so:
- ✅ WET lock is already released
- ✅ Can never accidentally hold the WET lock
- ✅ Other shoe can acquire WET lock during re-evap
- **No re-entry conflict with WET phase**

---

## 3. Corrected Motor Overlap Analysis

### The Real Issue (Not the WET Lock)

The WET lock mechanism WORKS correctly. The issue is more subtle:

**Actual Motor Overlap Path:**
1. Both shoes are wet initially
2. SUB1 acquires WET lock first (wetter) → enters S_WET
3. SUB2 waits in S_WAITING (locked out) ✅
4. SUB1 completes WET phase → exits to S_COOLING → **releases WET lock**
5. **Between steps 4-6, there's a window:**
   - S_COOLING motor phase is running (motor at variable duty for cooling)
   - WET lock just released
   - SUB2's S_WAITING fires and checks `coolingMotorPhaseActive(1)`
   
**The Gap:**
```cpp
if (coolingMotorPhaseActive(1))  // checks ONLY if motor is in the COOLING cooling phase
    return;
```

If `coolingMotorPhaseActive()` function returns FALSE when motor is running but transitioning, both could start.

### Need to Check: What Does `coolingMotorPhaseActive()` Actually Do?

Let me search for this function definition to see if it has the bug...

---

## The `coolingMotorPhaseActive()` Function

**Definition (line 143):**
```cpp
static inline bool coolingMotorPhaseActive(int idx) {
  // Motor phase is active if cooling started and stabilization hasn't begun yet
  return (g_subCoolingStartMs[idx] != 0 && g_subCoolingStabilizeStartMs[idx] == 0);
}
```

**What It Actually Checks:**
- Returns TRUE **only during the COOLING motor phase** (before stabilization)
- Returns FALSE when:
  - Not in COOLING state at all
  - In S_WET state (both timestamps are 0)
  - In stabilization phase (stab start is set)
  - In S_DRY or S_DONE

**The Real Problem:**

The WET lock **IS sufficient** to prevent SUB1+SUB2 from both being in S_WET simultaneously. But there's a **different window**:

1. **Phase 1:** SUB1 in S_WET with motor running (warmup + PID)
   - WET lock: owned by SUB1
   - SUB2 in S_WAITING, blocked by `g_wetLockOwner == 0` check ✅
   
2. **Phase 2:** SUB1 exits S_WET → S_COOLING (releases lock)
   - At transition instant, both callbacks could fire
   - `coolingMotorPhaseActive(0)` might not be set yet (timing race)
   - SUB2 could acquire lock and transition to S_WET
   - While SUB1 is just starting its COOLING motor phase
   - **Brief window: both motors might run**

---

## Summary

### Verified:
✅ WET lock mechanism EXISTS and prevents overlapping S_WET states
✅ Re-evap does NOT re-enter WET state (stays in COOLING)
✅ Re-evap does NOT hold the WET lock

### Potential Issue:
⚠️ The `coolingMotorPhaseActive()` check might have a window or bug
⚠️ Need to verify what this function actually checks

---

## Summary

### What I Got Wrong:
❌ The WET lock DOES work and prevents dual S_WET states
❌ The `coolingMotorPhaseActive()` check is correct and does what it's supposed to

### What's Actually Happening:
✅ The WET lock mechanism is solid for S_WET phase protection
✅ Re-evap never re-enters WET state (correct design)
✅ Re-evap never holds the WET lock (correct design)

### But There's Still a Potential Issue:
⚠️ **Race condition at S_WET → S_COOLING transition**

When SUB1 exits S_WET and enters S_COOLING:
1. S_WET exit callback releases the WET lock (`g_wetLockOwner = -1`)
2. State machine transitions to S_COOLING
3. S_COOLING entry callback calls `startCoolingPhase(0, false)`
   - This sets `g_subCoolingStartMs[0]`
   - Starts the COOLING motor phase

**If timing is unlucky:**
- Between step 1 (lock released) and step 3 (cooling motor starts), SUB2's S_WAITING could acquire the lock
- SUB2 might check `coolingMotorPhaseActive(0)` while it's FALSE (cooling motor not started yet)
- Both motors could briefly run simultaneously

### Bottom Line:
The WET lock is **not the issue** - it's more subtle. The recommendation to add `|| fsmSub2.getState() == SubState::S_WET` is still good for robustness, but the real benefit is catching WET→COOLING transitions more reliably.

---

## Recommendation

The WET lock is well-designed and works. The suggested motor overlap guard in Issue #1 of TIMING_ANALYSIS.md is still a good **defensive programming** improvement, but:

1. **Priority:** Lower than originally thought (WET lock provides protection)
2. **Purpose:** Adds extra safety margin during state transitions
3. **Cost:** Minimal (just one extra boolean check)

**Decision:** Keep it as a MEDIUM-priority improvement, not CRITICAL
