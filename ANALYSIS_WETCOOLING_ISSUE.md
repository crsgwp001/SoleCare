# Analysis: WET Exit & COOLING Inefficiency Issues

## Current Behavior (from log monitor_20251213_230542.log)

### Timeline for First Cycle
- **t=0-10s**: Heater warmup, rate ramping (0.3 → 2.9 g/m³/min)
- **t=10-200s**: Active evaporation, rate high (2-7 g/m³/min), steadily declining
- **t~210s**: Rate drops to 0.61 g/m³/min (still positive, evaporating)
- **t~217s**: Rate becomes -0.51 g/m³/min (negative - reversed?) → **WET EXITS**
- **t~300s**: After COOLING, shoe diff is still 4.05 → not dry
- **Result**: Returned to WAITING for second WET cycle

### The Problem: Double-Exit Issue

**Current WET Exit Logic:**
```
Requires:
1. ≥5 samples of rate data collected
2. ≥2 consecutive negative rate-of-change samples
3. Exit immediately when both conditions met
```

**Why This Fails for Wet Shoes:**
1. Evaporation rate follows a curve: rises → peaks → declines
2. At end of active drying (t~200s), rate starts declining naturally from 2-7 down to 0.5-1.0
3. With spiky sensor data, you get: 0.61 → 0.57 (-0.04) → -0.51 (-1.09) = **2 negatives = EXIT**
4. But shoe is only moderately dry (diff=3.9), not peak moisture removal complete
5. COOLING phase then can't further dry because motor duty is only 80% (lower than WET's 100%)

---

## Root Cause Analysis

### Issue #1: Exiting at Rate Peak, Not Completion
The WET exit logic was designed to detect "peak evaporation" by rate-of-change. But:
- **Peak rate** (highest moisture removal speed) occurs at t~50-100s (rate=5-7 g/m³/min)
- **Rate decline detection** triggers at t~210s (rate=0.6→-0.5)
- **Gap**: 110 seconds of missing drying!

### Issue #2: Rate Fluctuations at Low Moisture Levels
When shoes approach dryness (diff<1.0), AH removal rate becomes slower and noisier:
- Natural equilibrium oscillations
- Sensor noise floor becomes significant
- **Result**: Can get false "rate reversals" even though shoe IS still drying

### Issue #3: COOLING Incompatible with Wet Shoes
COOLING was designed as a "stabilization" phase after significant drying:
- Motor at only 80% (vs 100% in WET)
- After 2-minute motor phase, 2-minute stabilization (total 4 min)
- **Problem**: If shoe not sufficiently dry before COOLING, the lower duty can't catch up

### Issue #4: Dry-Check Threshold Too Sensitive
- After COOLING, checking `diff > 0.5` to decide "dry" or "retry"
- But shoe at diff=4.05 is still very wet
- With only 4 minutes of COOLING at 80%, very wet shoes stay wet

---

## Recommendations for Enhancement

### Option A: Extend WET Phase Duration (Simpler)
**Change**: Instead of exiting on rate-of-change, use **absolute AH diff threshold** + minimum time

**Implementation**:
```
Exit WET when:
- Minimum 5 minutes elapsed, AND
- AH diff < 1.5 g/m³ (moderately dry), OR
- Timeout at 15 minutes (safety)
```

**Pros**:
- Ignores spiky rate data
- Simpler logic
- Guarantees sufficient drying before COOLING

**Cons**:
- Longer WET phase (5-15 min vs current ~3 min)
- Less adaptive to different loads

### Option B: Smarter Rate Detection (Better Long-term)
**Change**: Use rate-of-change, but with better peak detection

**Implementation**:
```
Exit WET when:
- Rate declines for 5+ consecutive samples, AND
- Average rate over last 5 samples < 0.5 g/m³/min
(i.e., evaporation slowing down AND nearly complete)
```

**Pros**:
- Adaptive to different loads
- Handles rate fluctuations better
- Still relatively quick

**Cons**:
- More complex logic
- Requires more samples to validate

### Option C: Hybrid Approach (Recommended)
**Combine duration + rate + threshold**:

```
Exit WET when ANY of:
1. Time >= 10 minutes (safety timeout), OR
2. (Time >= 5 minutes) AND (AH diff < 1.5) AND (rate declining for 2+ samples)

Exit early ONLY if:
- Time >= 2 minutes AND AH diff < 0.8 (almost dry already)
```

**Pros**:
- Handles wet shoes (5+ min duration)
- Handles borderline dry shoes (early exit)
- Rate declining prevents false trigger
- Safe fallback timeout

**Cons**:
- More states to track

---

## Enhanced COOLING Phase

### Current Design Problem:
Motor at 80% may be **too conservative** for wet shoes (diff>2).

### Recommendation:
**Adaptive COOLING with two phases based on dryness**:

```cpp
Phase 1A (if diff > 2.0 - very wet):
  - Motor at 100% for 3 minutes (continue drying)
  - Heater OFF
  
Phase 1B (if diff 0.5-2.0 - moderately wet):
  - Motor at 80% for 2 minutes (standard cooling)
  - Heater OFF

Phase 2: Stabilization
  - Motor OFF for 2 minutes
  - Heater OFF
  - Let moisture settle
```

**Implementation**:
```cpp
// At COOLING entry, check current diff
float coolingDiff = g_dhtAHDiff[idx];
if (coolingDiff > 2.0f) {
  g_coolingPhase1DurationMs = 180000;  // 3 min
  motorSetDutyPercent(idx, 100);       // 100% instead of 80%
} else {
  g_coolingPhase1DurationMs = 120000;  // 2 min  
  motorSetDutyPercent(idx, 80);        // standard 80%
}
```

---

## Sensor Spikes: Why They Happen

The rate fluctuations you're seeing (0.61 → -0.51) are **normal**, not just EMI:

1. **Motor running creates air circulation**
2. **Moisture removal slows as shoe dries**
3. **Approaching equilibrium** - readings oscillate around the moving average
4. **Sensor can't capture exact evaporation** - gets noise floor effects

The existing protections (rate clamping, spike rejection) are working, but the **WET exit trigger is too sensitive** to these natural fluctuations.

---

## Summary: What to Change

### Immediate (v1.2.1):
1. **Increase MIN_AH_RATE_SAMPLES** from 5 to 8-10 (more samples before rate-change check)
2. **Require declining rate for 3+ consecutive samples**, not 2
3. **Add minimum WET duration** of 5 minutes before allowing exit

### Short-term (v1.3):
1. **Implement adaptive COOLING** based on moisture level
2. **Add absolute diff threshold** as backup exit condition (e.g., diff < 0.8)
3. **Increase COOLING motor phase** for very wet shoes

### Long-term (Phase 2):
1. **Implement "equilibrium detection"** - when rate stabilizes around small value
2. **Add learning** - track how long shoes typically need based on initial wetness
3. **Dual-setpoint PID** - aggressive phase then gentle phase

---

## Expected Improvements

With these changes:
- **WET phase duration**: 5-15 minutes (from current ~3 min)
- **COOLING effectiveness**: +20-30% improvement for wet shoes
- **Total cycle time**: ~12-20 min (from current ~10 min)
- **Success rate**: Much higher on first cycle, fewer re-attempts needed

---

## Test Parameters to Monitor

1. **WET duration**: Should be 5+ minutes for very wet shoes
2. **Rate pattern**: Should see decline for 3+ consecutive samples before exit
3. **COOLING diff change**: Should see noticeable improvement from start to end
4. **Cycles to DRY**: Should achieve DRY in 1-2 attempts (not 3+)
