# SoleCare FSM Architecture v1.1.0

## Overview
Dual-layer hierarchical state machine managing shoe drying with PID-controlled adaptive airflow and intelligent state transitions based on humidity dynamics.

**Architecture**: Global FSM (main control flow) + 2 independent Sub-FSMs (per-shoe drying logic)

---

## Global States & Transitions

### **Idle** (Initial State)
- **Entry Actions**:
  - Stop all motors, heaters, UVs
  - Release WET lock
  - Reset substates to S_IDLE
  - Clear all timing counters
  - Status LED ON, Error LED OFF
- **Transitions**:
  - StartPressed → Detecting (when Start button pressed)
  - ResetPressed → Idle (catch-all reset from any state)
- **Exit Conditions**: User presses Start button

### **Detecting** (Sensor Equalization)
- **Duration**: SENSOR_EQUALIZE_MS (default: 5s)
- **Purpose**: Let DHT sensors stabilize before checking moisture
- **Transitions**:
  - SensorTimeout → Checking (after 5s automatically)
  - ResetPressed → Idle
- **Battery Check**: If low battery detected → LowBattery state

### **Checking** (Battery & UI Check)
- **Purpose**: Battery voltage check before running motor
- **Run Logic**: Check g_batteryLowFlag
- **Transitions**:
  - StartPressed → Running (battery OK)
  - ResetPressed → Idle
  - Auto-advance to Running if battery recovered

### **Running** (Active Drying)
- **Entry Actions**:
  - Initialize substates based on moisture detection:
    - Shoe0InitWet/InitDry → Sub1
    - Shoe1InitWet/InitDry → Sub2
  - Clear WET lock (g_wetLockOwner = -1)
  - Status LED OFF, Error LED starts blinking (500ms)
- **Substate Initialization**:
  - **Wet shoe** → S_WAITING (queue for WET lock)
  - **Dry shoe** → S_DRY (go straight to UV)
- **Transitions**:
  - SubFSMDone (both subs reach S_DONE) → Done
  - ResetPressed → Idle
- **LED Status**: Error LED blinking = motor/UV active

### **Done** (Drying Complete)
- **Entry Actions**:
  - Stop all motors, heaters, UVs
  - Keep substates in final state (S_DONE)
  - Status LED starts blinking (500ms)
  - Start auto-reset timer (g_doneStartMs)
- **Auto-Reset**: After DONE_TIMEOUT_MS (10s) → posts ResetPressed (global only)
- **Exit**: Auto-reset triggers → Idle
- **Manual**: User can press Reset anytime

### **LowBattery** (Battery Fault)
- **Entry**: Voltage < threshold
- **Exit**: Charge battery, press Reset
- **Status**: Error LED solid ON

### **Error** (Fatal Error)
- **Status**: Error LED solid ON
- **Recovery**: Reset required

---

## Substate Machine (Per Shoe)

Each shoe (Sub1, Sub2) has independent state flow:

### **S_IDLE**
- Initial state for each substate machine
- Entry: Set to initial state
- Transitions:
  - Shoe0InitWet / Shoe1InitWet → S_WAITING
  - Shoe0InitDry / Shoe1InitDry → S_DRY
  - ResetPressed → S_IDLE (catch-all)

### **S_WAITING** (Queue for Heater)
- **Purpose**: Acquire sequential WET lock before heating
- **Lock Logic**: Only one shoe can be in S_WET at a time
- **Run Callback**: Check if lock is free
  - If free and other shoe not waiting: acquire lock
  - If both waiting: priority to wetter shoe (higher AH diff)
  - Post SubStart event to transition to S_WET
- **Transitions**:
  - SubStart (lock acquired) → S_WET
  - DryCheckFailed → S_WAITING (re-queue after failed dry-check)
  - ResetPressed → S_IDLE

### **S_WET** (Active Heating & Evaporation)
- **Timeline**:
  1. **0-5s**: Heater only (HEATER_WARMUP_MS)
  2. **5-30s+**: Motor starts at 100% duty
  3. **30s+**: PID monitoring begins
  4. **30-600s**: Exit when AH acceleration declines (peak evaporation detected)
  5. **Fallback**: Motor safety timeout at 600s (MOTOR_SAFETY_MS)

- **Entry Actions**:
  - Start heater immediately
  - Initialize timing counters
  - Reset PID controller
  - Clear AH rate tracking

- **Run Callback**:
  - Check if HEATER_WARMUP_MS elapsed → start motor at 100%
  - Sample AH diff every 2 seconds (after 30s warmup)
  - Calculate rate-of-change of AH diff
  - Detect peak: when rate becomes negative (declining) → SubStart
  - Exit to COOLING

- **Exit Actions**:
  - Reset PID (g_pidInitialized = false, motorPID.reset())
  - Release WET lock (no longer needed)
  - Stop motor, heater

- **Transitions**:
  - SubStart (evaporation peaks) → S_COOLING
  - ResetPressed → S_IDLE

### **S_COOLING** (Passive Cooling & Stabilization)
- **Two-Phase Design**:
  
  **Phase 1: Motor Cooling (0-60s)**
  - Run motor at 80% duty
  - Heater OFF
  - Purpose: Remove surface moisture
  
  **Phase 2: Stabilization (60-180s)**
  - Motor OFF
  - Let humidity sensor stabilize
  - Purpose: Get accurate final AH reading
  
- **Run Callback**:
  - Check motorElapsed < DRY_COOL_MS (60s)
    - If true: return (still cooling)
  - If stabilizeStartMs == 0: stop motor, set stabilization start
  - Check stabilizeElapsed < DRY_STABILIZE_MS (120s)
    - If true: return (still stabilizing)
  - Stabilization complete: perform dry-check

- **Dry-Check Logic**:
  - Read g_dhtAHDiff[i]
  - If diff > AH_DRY_THRESHOLD (0.5f): still wet
    - Post DryCheckFailed → go to S_WAITING (re-queue)
  - If diff ≤ AH_DRY_THRESHOLD: dry enough
    - Post SubStart → go to S_DRY

- **Transitions**:
  - SubStart (dry detected) → S_DRY
  - DryCheckFailed (still wet) → S_WAITING
  - ResetPressed → S_IDLE

### **S_DRY** (UV Exposure)
- **Entry Actions**:
  - Stop motor
  - Check if both shoes in S_DRY
  - Start single UV on GPIO14 (when both ready)
  - Wait for UV timer

- **Run**: Waits for UVTimer0 event

- **Transitions**:
  - SubStart (UV complete) → S_DONE
  - DryCheckFailed → S_WAITING (if UV timer expires early)
  - ResetPressed → S_IDLE

### **S_DONE** (Cycle Complete)
- **Entry Actions**:
  - Mark substate complete (g_subDoneMask)
  - Release WET lock (final cleanup)
- **Transitions**:
  - SubStart → S_DONE (idle in done state)
  - ResetPressed → S_IDLE

---

## Key Mechanisms

### WET Lock (Sequential Heater Access)
```
Global: g_wetLockOwner (-1 = free, 0 = sub1, 1 = sub2)

S_WAITING run():
  if (lock free):
    if (both waiting):
      give to wetter shoe
    else:
      give to waiting shoe
    post SubStart → transition to S_WET

S_WET exit:
  release lock (g_wetLockOwner = -1)
```

### AH-Based WET Exit (Peak Evaporation Detection)
```
During S_WET (after 30s warmup):
  every 2 seconds:
    sample AH_diff
    if (samples ≥ 3):
      calculate: rateChange = current - previous
      if (rateChange < -0.01):  # declining
        exit to COOLING
```

### PID Motor Control (Phase 1: P-only)
```
Configuration:
  Kp = 0.2 (proportional gain)
  Ki = 0.05 (integral gain, Phase 1)
  Setpoint = 0.4 g/m³/min
  Output: 50-100% (0.5-1.0 normalized)
  Sample: 2-second interval

Timeline in S_WET:
  0-30s: Fixed 75% duty
  30s+: PID adaptive 50-100% based on AH rate error

Output formula:
  midpoint = (0.5 + 1.0) / 2 = 0.75
  range = 1.0 - 0.5 = 0.5
  out = midpoint + (P + I + D) * range/2
  out = clamp(out, 0.5, 1.0)
```

### Hysteresis Thresholds
```
Entry to WET:   AH_diff > 1.0f (wet detected)
Exit to DRY:    AH_diff < 0.5f (dry enough)
Gap:            0.5 unit hysteresis
Purpose:        Prevent state oscillation
```

### Global Idle Reset
```
On entry to Idle:
  1. Stop motors (0, 1)
  2. Stop heaters (0, 1)
  3. Stop UVs (0, 1)
  4. Release locks
  5. Reset flags (uvComplete, motorStarted, etc.)
  6. Post ResetPressed to subs → all go to S_IDLE
```

---

## Timing Summary (Milliseconds)

| Phase | Duration | Purpose |
|-------|----------|---------|
| Sensor Equalize | 5s | DHT stabilization |
| Heater Warmup | 5s | Heat shoe before motor |
| PID Warmup | 30s | Fixed duty before PID |
| Motor Cooling | 60s | Active evaporation |
| Stabilization | 120s | Passive humidity equalization |
| Motor Safety | 600s | Hard stop if stuck |
| Done Auto-Reset | 10s | Auto-return to Idle |
| PID Sample | 2s | Control loop frequency |

---

## Event Flow Example: Single Wet Shoe

```
[Idle] 
  → Start button
[Detecting] 
  → Wait 5s
[Checking]
  → Battery OK
[Running]
  → Sub1: Shoe0InitWet (AH_diff > 1.0)
  → Sub1: S_WAITING (queue for lock)
  → Sub1: SubStart (lock acquired)
  → Sub1: S_WET
    ├─ 0-5s: heater only
    ├─ 5-30s: motor 100%, PID warming
    ├─ 30s+: motor adaptive 50-100%, detect peak
    └─ Peak detected: SubStart event
  → Sub1: S_COOLING
    ├─ 0-60s: motor 80%
    ├─ 60-180s: stabilize
    └─ Dry-check: AH_diff < 0.5f
  → Sub1: S_DRY
    ├─ Start single UV (both in DRY)
    └─ Wait for UV timer
  → Sub1: S_DONE
[Done]
  → Auto-reset after 10s
[Idle]
```

---

## Dual-Shoe Parallel Example

```
Sub1 (wet):                    Sub2 (wet):
S_WAITING                      S_WAITING (blocks)
↓ (lock available)             ↓
S_WET (heating)                S_WAITING (higher priority? or equal?)
↓ (30s+)                       ↓
S_COOLING                      S_WAITING → S_WET (lock released)
↓ (180s)                       ↓ (30s+)
S_DRY                          S_COOLING
↓ (both in DRY now)            ↓ (180s)
[UV starts]                    ↓ (both in DRY now)
S_DONE ← ← ← ← ← ← ← ← ← ← S_DONE
[Done state reached]
```

---

## Not Yet Implemented (Phase 2+)

### Dual-Phase Setpoint Switching
```
Phase 2A: Aggressive evaporation
  Setpoint = 0.4 g/m³/min (current)
  Exit: when rate peaks

Phase 2B: Gentle stabilization
  Setpoint = -0.1 g/m³/min (dehumidifying)
  Removes last moisture slowly
  Exit: when stable
```

### Integral & Derivative Terms
- Currently Ki = 0.05 (minimal)
- Kd = 0.0 (disabled)
- Enable after Kp stabilization

### Adaptive Thresholds
- Current fixed: WET=1.0, DRY=0.5
- Phase 2: Learn thresholds from sensor history

---

## Tuning Parameters (v1.1.0)

### Critical for Optimization
1. **PID_KP** (0.2): Reduce if oscillating, increase if sluggish
2. **PID_KI** (0.05): Increase if never reaches setpoint
3. **AH_WET_THRESHOLD** (1.0f): Lower if false positives on dry shoes
4. **AH_DRY_THRESHOLD** (0.5f): Increase if shoes re-queue too often
5. **DRY_STABILIZE_MS** (120s): Increase if readings unstable
6. **AH_ACCEL_WARMUP_MS** (30s): Reduce if sensor stable earlier

### Test Points
- Watch CSV logs for PID output range (should use 50-100%)
- Monitor AH diff samples for trend clarity
- Check state dwell times in serial output
- Verify no state oscillation (bouncing)

---

## Status: v1.1.0
✅ **Working** - Early build, not optimized
- FSM core logic stable
- PID control functional but gains preliminary
- All major transitions tested
- Ready for hardware validation

⚠️ **Known Limitations**
- No adaptive thresholds
- Single setpoint only
- P+I only (no derivative)
- CSV logging may add latency

🔧 **Next Steps**
- Characterize actual system response
- Tune PID gains for target drying speed
- Adjust AH thresholds for shoe types
- Implement Phase 2 dual-setpoint control
