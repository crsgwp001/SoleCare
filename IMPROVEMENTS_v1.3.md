# v1.3.0 - Enhanced WET/COOLING Logic with Adaptive Cooling

**Build Status**: ✅ SUCCESS (344661 bytes, 26.3% flash)

## Overview
Major improvements to WET phase robustness and COOLING phase effectiveness addressing premature WET exits during high motor noise environments and insufficient cooling for very wet shoes.

---

## Changes Implemented

### 1. **Increase Peak Detection Threshold** 
- **Issue**: WET phase exiting after only ~100 seconds due to MIN_CONSECUTIVE_NEGATIVE=2 being too sensitive to noise spikes
- **Fix**: Increased `MIN_CONSECUTIVE_NEGATIVE` from 2 to 3
- **Effect**: Requires 3 consecutive declining rate samples instead of 2, filtering out single noise spikes
- **Location**: `src/tskFSM.cpp` line 73

### 2. **Post-Peak Evaporation Buffer**
- **Issue**: After detecting peak evaporation, shoe needs extra drying time before transitioning to COOLING
- **Fix**: Added 60-second buffer after peak detection
- **Details**:
  - New flag: `g_peakDetected[2]` tracks if peak has been detected
  - New timestamp: `g_peakDetectedMs[2]` marks when peak was first detected
  - New constant: `WET_PEAK_BUFFER_MS = 60000` (60 seconds)
  - After peak detected, system waits 60 seconds before checking minimum duration
- **Effect**: Shoe gets 60 extra seconds of drying time after peak evaporation detected
- **Location**: `include/config.h`, `src/tskFSM.cpp` lines 535-595, 635-705

### 3. **Minimum WET Phase Duration**
- **Issue**: System was exiting WET too early, not allowing sufficient drying time
- **Fix**: Added mandatory 5-minute minimum before any exit, even after peak+buffer
- **Details**:
  - New constant: `WET_MIN_DURATION_MS = 300000` (5 minutes)
  - Prevents exit until both conditions met:
    - Peak detected + buffer elapsed, AND
    - Minimum 5 minutes of total WET time elapsed
  - Falls back to 5-minute timeout even if peak detection fails
- **Effect**: WET phase guaranteed minimum runtime of 5 minutes for thorough drying
- **Location**: `include/config.h`, `src/tskFSM.cpp` peak detection logic

### 4. **Adaptive COOLING Based on Moisture Level**
- **Issue**: 80% motor duty insufficient for very wet shoes (diff > 2.0)
- **Fix**: Implemented adaptive COOLING with two tiers:
  - **Tier 1 (diff > 2.0)**: Very wet shoes get 100% motor duty + extended 180s motor phase
  - **Tier 2 (diff ≤ 2.0)**: Moderately wet shoes get standard 80% duty + 120s motor phase
- **Details**:
  - New constants:
    - `DRY_COOL_MS_BASE = 120000` (base motor duration for standard cooling)
    - `DRY_COOL_MS_WET = 180000` (extended motor duration for wet shoes)
  - New tracking: `g_coolingMotorDurationMs[2]` per-shoe adaptive duration
  - Entry callback checks current diff and sets duty/duration accordingly
  - Run callback uses adaptive duration instead of fixed DRY_COOL_MS
- **Effect**: Very wet shoes get 50% more cooling time + stronger air circulation
- **Locations**: 
  - `include/config.h` new constants
  - `src/tskFSM.cpp` COOLING entry (lines 343-370, 435-462)
  - `src/tskFSM.cpp` COOLING run (lines 798-802, 856-860)

---

## Complete Timeline: WET → COOLING → DRY

### For Very Wet Shoe (diff = 4.357)
1. **WAITING → WET**: Heater on, motor off
2. **WET (0-10s)**: Heater warmup period
3. **WET (10+s)**: Motor starts at 100%, samples rate every 2 seconds
4. **WET (180+s)**: Rate starts declining (peak detected), enters 60-second buffer
5. **WET (240+s)**: Buffer expires, minimum 5-minute check
6. **WET (300s)**: Minimum duration reached → **Exit to COOLING**
7. **COOLING (0-180s)**: Motor at 100% (adaptive for diff > 2.0), heater off
8. **COOLING (180-300s)**: Stabilization phase, motor off
9. **COOLING (300s)**: Dry-check
   - If diff now ≤ 0.5 → DRY (success!)
   - If diff still > 0.5 → WAITING (retry)

### For Moderately Wet Shoe (diff = 1.2)
1. **WET (0-10s)**: Heater warmup
2. **WET (10+s)**: Motor starts, samples rate
3. **WET (180+s)**: Peak detected (declining rate) → 60s buffer
4. **WET (240+s)**: Buffer expires, minimum check
5. **WET (300s)**: Minimum duration reached → **Exit to COOLING**
6. **COOLING (0-120s)**: Motor at 80% (standard for diff < 2.0)
7. **COOLING (120-240s)**: Stabilization phase
8. **COOLING (240s)**: Dry-check → likely DRY (success!)

---

## Configuration Constants (Updated)

### Timing (include/config.h)
```cpp
constexpr uint32_t WET_MIN_DURATION_MS = 300u * 1000u;      // 5 minutes minimum
constexpr uint32_t WET_PEAK_BUFFER_MS = 60u * 1000u;        // 60s buffer after peak
constexpr uint32_t DRY_COOL_MS_BASE = 120u * 1000u;         // Standard COOLING motor phase
constexpr uint32_t DRY_COOL_MS_WET = 180u * 1000u;          // Extended for very wet
constexpr uint32_t DRY_STABILIZE_MS = 120u * 1000u;         // Stabilization unchanged
```

### Peak Detection Sensitivity (src/tskFSM.cpp)
```cpp
constexpr int MIN_CONSECUTIVE_NEGATIVE = 3;                  // Was: 2
constexpr int MIN_AH_RATE_SAMPLES = 5;                       // Unchanged
constexpr float AH_RATE_DECLINE_THRESHOLD = -0.01f;         // Unchanged
```

---

## Expected Improvements

### Before v1.3.0
- WET exited after ~100 seconds (too early)
- COOLING @ 80% insufficient for very wet shoes
- Often required 2-3 cycles to achieve DRY (diff reduction only 7% per cycle)
- Noisy rates during motor startup caused false peaks

### After v1.3.0
- WET guaranteed minimum 5 minutes (300 seconds)
- COOLING adaptive: 100% duty for very wet, 80% for standard
- Expected 20-30% diff reduction per COOLING cycle
- Noise filtering with 3-sample consecutive requirement
- 60-second post-peak buffer ensures continued evaporation

---

## Safety Features Maintained

✅ Watchdog timeout guards (early returns on timing checks)
✅ NaN/Inf validation for all rate samples
✅ Rate clamping (±120 g/m³/min)
✅ EMI input validation (temp -40-85°C, RH 0-100%)
✅ Motor overlap prevention with coolingMotorPhaseActive()
✅ Sequential WET lock (only one shoe at a time)
✅ Dual-phase COOLING (motor + stabilization)
✅ Dry-check logic validates final state

---

## Testing Recommendations

1. **Test 1: Very Wet Shoe**
   - Initial diff > 4.0
   - Verify WET runs at least 300 seconds
   - Verify COOLING @ 100% for first shoe
   - Check final dry-check diff < 0.5

2. **Test 2: Moderately Wet Shoe**
   - Initial diff 1.0-2.0
   - Verify WET still protected by minimum duration
   - Verify COOLING @ 80% (standard)
   - Should succeed in first cycle

3. **Test 3: Nearly Dry Shoe**
   - Initial diff < 0.8
   - Should skip WET, go straight to DRY
   - Verify no unnecessary cycling

4. **Test 4: Motor Startup Noise**
   - Verify rate spikes don't cause false WET exit
   - Should require 3 consecutive negative rates
   - Post-peak buffer should protect late drying

---

## Files Modified

1. **include/config.h**
   - Added timing constants for WET/COOLING adaptive behavior

2. **src/tskFSM.cpp**
   - Updated MIN_CONSECUTIVE_NEGATIVE (2 → 3)
   - Added g_peakDetected[] and g_peakDetectedMs[] tracking
   - Added g_coolingMotorDurationMs[] for adaptive durations
   - Updated SUB1 & SUB2 WET entry callbacks (reset peak flags)
   - Updated SUB1 & SUB2 WET run callbacks (peak detection + buffer + minimum duration)
   - Updated SUB1 & SUB2 COOLING entry callbacks (adaptive duty/duration)
   - Updated SUB1 & SUB2 COOLING run callbacks (use adaptive duration)

---

## Backward Compatibility

✅ **Fully backward compatible** - no external API changes
✅ Configuration constants localized
✅ No new module dependencies
✅ Same serial logging format
✅ Same event flow structure

---

## Version History

- **v1.2.0**: Multi-layer EMI protection, rate clamping, watchdog guards
- **v1.3.0**: Robust WET exit (3-sample threshold), post-peak buffer, minimum duration, adaptive COOLING
- **v1.4.0** (future): Machine learning for individual shoe drying profiles

