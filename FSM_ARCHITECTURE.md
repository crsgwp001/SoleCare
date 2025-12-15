# SoleCare FSM Architecture v1.3.0

## Overview
Dual-layer hierarchical state machine managing shoe drying with dual-phase PID-controlled adaptive airflow, moving-average peak detection, adaptive cooling tiers, and trend-based lenient dry-checking.

**Architecture**: Global FSM (main control flow) + 2 independent Sub-FSMs (per-shoe drying logic)

**Major Updates (v1.3.0)**:
- Dual-phase PID setpoints (aggressive evaporation → gentle stabilization)
- Moving-average peak detection with 60s post-peak buffer
- Three-tier adaptive COOLING based on moisture level
- Trend-aware lenient dry-check threshold
- One cooling retry before re-queueing to WET

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
  1. **0-10s**: Heater only (HEATER_WARMUP_MS)
  2. **10-30s**: Motor starts at 75% fixed duty (PID warmup)
  3. **30-120s**: PID Phase 2A (aggressive evaporation, setpoint=0.45 g/m³/min)
  4. **120s+**: PID Phase 2B when rate < 0.25 (stabilization, setpoint=0.08 g/m³/min)
  5. **Exit**: Moving-average peak detected + 60s minimum WET + 75s post-peak buffer
  6. **Fallback**: Motor safety timeout at 600s (MOTOR_SAFETY_MS)

- **Entry Actions**:
  - Start heater immediately
  - Initialize timing counters (g_subWetStartMs)
  - Reset PID controller
  - Clear AH rate tracking & moving-average buffers (8 samples)
  - Reset cooling retry count to 0

- **Run Callback (Peak Detection)**:
  - Check if HEATER_WARMUP_MS elapsed → start motor at 75% fixed
  - After 30s: PID takes over with dual-phase setpoints
  - Sample AH rate every few seconds, store in circular buffer (8 samples)
  - Calculate moving averages: last 3 samples vs prior 3 samples
  - Detect peak: 3 consecutive declining averages
  - Enforce 6-minute minimum + 75s post-peak buffer before exit
  - Exit to COOLING when conditions met

- **PID Dual-Phase Logic**:
  - **Phase 2A (Evaporation)**: Target 0.45 g/m³/min, duty 50-100%
    - Aggressive drying, maximize moisture removal
  - **Phase 2B (Stabilization)**: Target 0.08 g/m³/min, duty 50-100%
    - Switch when rate < 0.25 after 120s in Phase 2A
    - Gentle final drying, prevents over-drying

### **S_COOLING** (Motor-Only Finish & Stabilization + Optional Re-Evap)
- **Two-Phase Design**:
  
  **Phase 1: Adaptive Motor Cooling (90-210s, Ambient-Coupled)**
  - Three-tier strategy based on moisture level:
    - **Nearly dry** (diff ≤ 1.2): 90s @ 80% duty
    - **Moderate** (1.2 < diff ≤ 2.0): 150s @ 90% duty
    - **Very wet** (diff > 2.0): 180s @ 100% duty
  - **Ambient-coupled motor control**: Modulate duty by temp-to-ambient delta; extend motor phase until shoe reaches ambient + 0.5°C
  - Heater OFF (motor-only drying)
  - Purpose: Remove remaining moisture, prevent heat-inflated AH readings
  
  **Phase 2: Stabilization (90s)**
  - Motor OFF
  - Let humidity sensor stabilize
  - Sample AH diff every 15s (6 samples total)
  - Temperature must be ≤ ambient + 0.5°C (safety gate)
  - Purpose: Get accurate final AH reading & track decline trend
  
- **Entry Actions**:
  - Turn off heater
  - Evaluate current moisture level (g_dhtAHDiff)
  - Set motor duty (80/90/100%) and duration (90/150/180s) based on tier
  - Start cooling timer
  - Reset trend tracking buffers

- **Run Callback**:
  - **Motor Phase**: Check motorElapsed < coolingMotorDurationMs
    - **Temperature guard**: If shoe temp > ambient + 0.5°C, extend motor until cooler
    - Else: stop motor, set stabilization start
  - **Stabilization Phase**: Sample AH diff every 15s
    - Store in circular buffer (6 samples)
    - Check stabilizeElapsed < DRY_STABILIZE_MS (90s)
    - Verify temperature ≤ ambient + 0.5°C (safety gate)
    - If true: return (still stabilizing)
  - **Dry-Check**: After stabilization complete

- **Adaptive Dry-Check Logic**:
  - Analyze trend: Check if last 4 samples show consistent decline
    - Declining = 2+ out of 3 recent transitions show decrease
  - Select threshold:
    - **Strict** (0.5): Used if moisture stable/increasing
    - **Lenient** (0.7): Used if moisture consistently declining
  - Evaluate: diff vs selected threshold
  - **If still wet**:
    - Immediately invoke RE-EVAP short cycle (no cooling retries)
  - **If dry**: post SubStart → S_DRY

- **RE-EVAP Short Cycle (Subphase, On First Dry-Check Failure)**:
  - **Entry**: Still wet after COOLING stabilization
    - Turn on heater + motor at 80% duty
    - Set max duration: 30-60s (by moisture tier)
    - Use lenient rise-from-min detection (threshold 0.15 g/m³/min)
  - **Run**:
    - Track AH diff min since re-evap start
    - If diff rises 0.15+ above min: likely peaked (moisture re-evaporated)
    - Or if timeout: exit and continue
  - **Exit**:
    - Return to COOLING Phase 1 (motor-only, ambient-coupled)
    - Repeat stabilization → dry-check
  - **Final Failure**:
    - If still wet after re-evap: post DryCheckFailed → S_WAITING (re-queue to WET)

- **Transitions**:
  - SubStart (dry detected) → S_DRY
  - RE-EVAP (first dry-check failure) → Re-Evap subphase (heater+motor brief)
  - After Re-Evap → back to COOLING Phase 1
  - DryCheckFailed (after re-evap still wet) → S_WAITING (re-queue to WET)
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
      give to wetter shoe2: Dual Setpoints)
```
Configuration (v1.3.0):
  Kp = 0.15 (proportional gain, increased for faster response)
  Ki = 0.03 (integral gain, increased for better error correction)
  Kd = 0.08 (derivative gain, increased for damping)
  Output: 50-100% (0.5-1.0 normalized)
  Sample: 3-second interval

Dual-Phase Setpoints:Moving-Average Peak Detection)
```
During S_WET (after warmup):
  Continuously:
    sample AH rate-of-change
    store in circular buffer (8 samples per shoe)
    
  When buffer has ≥5 samples:
    calculate: avg_last3 = mean(samples[n-2:n])
    calculate: avg_prior3 = mean(samples[n-5:n-3])
    
    if (avg_last3 < avg_prior3):
      increment consecutive_negative_count
      if (consecutive_negative_count >= 3):
        mark peak_detected = true
        record peak_detected_timestamp
    else:
      reset consecutive_negative_count = 0
  
  Exit conditions (all must be true):
    1. peak_detected == true
    2. wet_elapsed >= WET_MIN_DURATION_MS (360s = 6 min)
    3. (now - peak_timestamp) >= WET_PEAK_BUFFER_MS (75s)
  
  Then: transitionts over-evaporation
    - Maintains slight positive drying rate

Timeline in S_WET:
  0-10s: Heater warmup
  10-30s: Fixed 75% duty (PID warmup)
  30-120s+: PID Phase 2A (aggressive, setpoint 0.45)
  120s+: PID Ph& Adaptive Thresholds
```
Entry to WET:         AH_diff > 1.0f (wet detected)
Exit to DRY (strict): AH_diff < 0.5f (dry enough, stable moisture)
Exit to DRY (lenient): AH_diff < 0.7f (dry enough, declining moisture)
Gap:                  0.5-0.7 unit hysteresis
Purpose:              Prevent state oscillation, allow earlier exit when trending dry

Adaptive Dry-Check Selection:
  During stabilization: sample AH_diff every 15s (6 samples)
  Analyze trend: check if 2+ out of 3 recent transitions show decline
  If declining: use lenient threshold (0.7)
  If stable/increasing: use strict threshold (0.5)
  
Cooling Tier Thresholds:
  Nearly dry:  diff ≤ 1.2  → 90s @ 80%
  Moderate:    1.2 < diff ≤ 2.0 → 150s @ 90%
  Very wet:    diff > 2.0  → 180s @ 100%
  pid_output = Kp*error + Ki*integral + Kd*derivative
  out = midpoint + pid_output  # declining
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

Output formula: - v1.3.0

| Phase | Duration | Purpose |
|-------|----------|---------|
| Sensor Equalize | 5s | DHT stabilization |
| Heater Warmup | 10s | Heat shoe before motor |
| PID Warmup | 30s | Fixed 75% duty before PID |
| WET Minimum | 360s (6 min) | Minimum heater-on evaporation time |
| WET Peak Buffer | 75s | Extra drying after peak detected |
| **Cooling Tiers** | | **Adaptive motor-only cooling** |
| └─ Nearly Dry | 90s @ 80% | Light finish (diff ≤ 1.2) |
| └─ Moderate | 150s @ 90% | Standard cooling (1.2 < diff ≤ 2.0) |
| └─ Very Wet | 180s @ 100% | Aggressive cooling (diff > 2.0) |
| Stabilization | 90s | Sensor stabilization + trend tracking |
| Cooling Retry | Max 150s @ 100% | One retry before re-queue |
| Motor Safety | 600s | Hard stop if stuck in WET |
| Done Auto-Reset | 10s | Auto-return to Idle |
| PID Sample | 3s | Control loop frequency |
| Diff Sample (Cooling) | 15s | Trend analysis during stabilization
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
``` (v1.3.0)

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
  → Sub1: S_WET (Aggressive Evaporation)
    ├─ 0-10s: heater only (warmup)
    ├─ 10-30s: motor 75% fixed, heater on
    ├─ 30-120s: PID Phase 2A (setpoint 0.45), heater on
    ├─ 120s+: PID Phase 2B when rate < 0.25 (setpoint 0.08)
    ├─ Detect: 3 consecutive declining moving averages
    ├─ Wait: 6 min minimum + 75s post-peak buffer
    └─ Peak + buffers satisfied: SubStart event
  → Sub1: S_COOLING (Motor-Only Finish + Stabilization)
    ├─ Evaluate diff: 2.5 → Very Wet tier
    ├─ 0-180s: motor 100%, ambient-coupled (extend if temp > ambient+0.5C)
    ├─ 180-270s: stabilize (sample every 15s, temp guard)
    ├─ Analyze trend: declining (lenient threshold)
    └─ Dry-check: AH_diff = 0.65 < 0.7 (lenient) → PASS
  → Sub1: S_DRY
    ├─ Start single UV (when both in DRY)
    └─ Wait for UV timer
  → Sub1: S_DONE
[Done]
  → Auto-reset after 10s
[Idle]
```

**Alternative Flow: Immediate Re-Evap on First Dry-Check Failure**
```
  → Sub1: S_COOLING (first pass)
    └─ Dry-check: AH_diff = 0.9 > 0.7 → FAIL
  → Sub1: RE-EVAP short cycle (no cooling retries)
    ├─ 0-30s: heater + motor 80% (lenient rise detection)
    ├─ Monitor: diff min tracking, rise detection
    └─ Peak detected or timeout: exit re-evap
  → Sub1: S_COOLING (back to Phase 1, ambient-coupled)
    ├─ 0-180s: motor 100% (aggressive tier)
    ├─ 180-270s: stabilize
    └─ Dry-check: AH_diff = 0.45 < 0.5 → PASS
  → Sub1: S_DRY (continue)
```

**Worst Case: Re-queue to WET After Re-Evap**
```
  → Sub1: S_COOLING (first pass)
    └─ Dry-check: AH_diff = 1.2 → FAIL
  → Sub1: RE-EVAP short cycle
    └─ Still wet after re-evap
  → Sub1: S_COOLING (Phase 1 retry)
    └─ Dry-check: AH_diff = 1.0 → FAIL (still wet)
  → Sub1: S_WAITING (re-queue as last resort)
  → Sub1: S_WET (additional heating cycle)
  → Sub1: S_COOLING → S_DRY → S_DONE
```ng]
  → Sub1: Shoe0InitWet (AH_diff > 1.0)
  → Sub1: S_WAITING (queue for lock)
  → Sub1: SubStart (lock acquired)
  → Sub1: S_WET
    ├─ 0-5s: heater only
    ├─ 5-30s: motor 100%, PID warming
    ├─ 30s+: motor adaptive 50-100%, detect peak
    └─ Peak detected: SubStart event
  →Implemented Features (v1.3.0)

### ✅ Dual-Phase PID Setpoint Switching
```
Phase 2A: Aggressive evaporation
  Setpoint = 0.45 g/m³/min
  Active: 30-120s+ into WET phase
  Exit: when rate < 0.25 g/m³/min (after 120s minimum)

Phase 2B: Gentle stabilization
  Setpoint = 0.08 g/m³/min
  Maintains slight positive drying rate
  Exit: when moving-average peak detected
```

### ✅ Full PID Control (P+I+D)
- Kp = 0.15 (increased for faster response)
- Ki = 0.03 (better steady-state error correction)
- Kd = 0.08 (improved damping and smoothness)

### ✅ Adaptive Dry-Check Thresholds
- Trend-aware: analyzes 6 samples during stabilization
- Strict threshold (0.5): stable/increasing moisture
- Lenient threshold (0.7): consistently declining moisture
- Reduces unnecessary retries when shoe is clearly drying
3.0)

### Critical for Optimization
1. **PID_KP** (0.15): Faster response, reduce if oscillating heavily
2. **PID_KI** (0.03): Better steady-state, reduce if windup occurs
3. **PID_KD** (0.08): Damping/smoothness, reduce if too sluggish
4. **TARGET_AH_RATE_EVAP** (0.45): Phase 2A aggressive target
5. **TARGET_AH_RATE_STABLE** (0.08): Phase 2B gentle target
6. **AH_RATE_PHASE_THRESHOLD** (0.25): Evap→Stable switch point
7. **WET_MIN_DURATION_MS** (360s): Minimum heater-on time
8. **WET_PEAK_BUFFER_MS** (75s): Post-peak safety margin
9. **AH_DRY_THRESHOLD** (0.5f): Strict dry threshold
10. **AH_DRY_THRESHOLD_LENIENT** (0.7f): Lenient for declining trends
11. **Cooling thresholds** (1.2, 2.0): Tier boundaries

### Test Points
- Monitor PID phase switches in logs (2A → 2B transition)
- Verify moving-average peak detection timing
- Check cooling tier selection based on actual moisture
- Watch dry-check decisions (strict vs lenient)
- Confirm single-pass drying success rate
- Validate retry logic triggers appropriately

### Optimization Tips
- If shoes too wet after cooling: increase WET_MIN_DURATION_MS
- If over-drying: reduce TARGET_AH_RATE_EVAP, increase peak buffer
- If retries too frequent: adjust lenient threshold (0.7 → 0.8)
- If WET phase too long: reduce WET_MIN_DURATION_MS (but not below 4 min)

---

## Status: v1.4.2
✅ **Production Ready** - COOLING streamlined with immediate re-evap
- **No cooling retries**: On first dry-check failure, immediately invoke re-evap short cycle
- **Ambient-coupled COOLING**: Motor duty scales by temperature delta; extends phase until shoe cools to ambient+0.5°C
- **Temperature-gated stabilization**: Safety gate prevents false dry-checks on hot shoes
- **Re-Evap subphase**: Heater+motor burst with lenient rise detection; returns to COOLING if still wet
- **Dual-phase peak detection**: Rise-from-min and rate-decline detection in WET phase
- **Adaptive dry-checks**: Lenient vs strict thresholds based on moisture trending
- All major transitions validated; ready for field testing

🔧 **Key Improvements Over v1.3.0**
- Removed cooling retry loop (simpler, faster iteration)
- Immediate re-evap on first dry-check failure (more responsive to persistent moisture)
- Ambient-coupled motor control (prevents temperature-inflated AH readings)
- Temperature safety gate in stabilization (robust dry-checking)
- Faster cycle times without retry penalty

⚠️ **Known Considerations**
- Re-evap duration (30-60s) may need tuning per moisture tier
- Lenient rise threshold (0.15 g/m³/min) can be adjusted if re-evap exits prematurely
- Very extreme humidity may still require multiple WET cycles

🎯 **Next Steps**
- Upload and monitor real drying sessions; capture logs focusing on COOLING→RE-EVAP→COOLING flow
- Validate ambient-target temperature control and re-evap effectiveness
- Fine-tune re-evap durations and rise thresholds based on field data

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
