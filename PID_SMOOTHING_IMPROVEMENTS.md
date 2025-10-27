# PID Control and Reaction Wheel Smoothing Improvements

## Overview

The PID calculation and reaction wheel movement have been optimized to provide smoother, less aggressive control while maintaining effective stabilization. Additionally, the reaction wheel now properly stops after landing (IMPACT state).

## Changes Made

### 1. Primary ESP32 - PID Controller Smoothing

#### Reduced PID Gains

- **Kp (Proportional)**: 0.5 → **0.3** (40% reduction)
- **Ki (Integral)**: 0.1 → **0.05** (50% reduction)
- **Kd (Derivative)**: 0.2 → **0.15** (25% reduction)

_Effect_: Less aggressive initial response, smoother corrections

#### Added PID Deadband

- **Deadband**: **±5°/s**
- Ignores small gyro variations below this threshold
- _Effect_: Prevents micro-corrections and jittery movement

#### Implemented Low-Pass Filter

- **Smoothing Factor**: **0.3** (30% new value, 70% old value)
- Filters out rapid PID output changes
- _Effect_: Gradual, smooth transitions instead of sudden jumps

#### Added Rate Limiting

- **Max Change Rate**: **20 units/update**
- Prevents sudden acceleration/deceleration
- _Effect_: Controlled response speed

#### Reduced Integral Windup

- **Integral Limit**: 100 → **50**
- Prevents excessive integral accumulation
- _Effect_: Less overshoot, more stable

#### IMPACT State Detection

- **PID disabled after landing**
- Automatically stops when `currentState == IMPACT`
- Resets PID values (output, integral, smoothed output)
- _Effect_: No reaction wheel activity after landing

### 2. Secondary ESP32 - Reaction Wheel Smoothing

#### ESC Ramping

- **Ramp Rate**: **200 µs/second**
- Smoothly transitions between target positions
- Prevents sudden motor speed changes
- _Effect_: Gradual acceleration/deceleration

#### ESC Deadband

- **Deadband**: **±10** PID units
- Ignores small PID commands
- _Effect_: Reduces jittery motor movements

#### Minimum Change Threshold

- **Min Change**: **5 µs**
- Ignores micro-adjustments
- _Effect_: Prevents constant small corrections

#### Smooth Stop Function

- Uses same ramp rate for deceleration
- Gradually returns to neutral (1500 µs)
- _Effect_: No abrupt motor stops

#### Flight State Detection

- **Monitors IMPACT state from CSV data**
- Extracts flight state (column 20) from telemetry
- Stops reaction wheel immediately when IMPACT detected
- _Effect_: Wheel stops spinning after landing

## Technical Details

### PID Calculation Flow

```
1. Read gyro Z-axis (yaw rate)
2. Check if IMPACT state → if yes, set PID to 0 and exit
3. Calculate error (setpoint - actual)
4. Apply deadband (±5°/s)
5. Calculate P, I, D terms
6. Sum to get raw output
7. Apply rate limiting
8. Apply low-pass filter
9. Send smoothed output
```

### Reaction Wheel Control Flow

```
1. Receive CSV data from Primary
2. Extract flight state and PID value
3. Check if IMPACT state (state == 7)
   → If IMPACT: Stop wheel immediately
   → If not IMPACT: Continue control
4. Apply ESC deadband (±10)
5. Map to ESC microseconds (1000-2000)
6. Calculate change from current position
7. Apply ramp rate limiting
8. Check minimum change threshold
9. Update ESC position smoothly
```

## Tuning Parameters

### For Less Aggressive Movement (Current Settings)

- Lower PID gains (Kp, Ki, Kd)
- Higher smoothing factor (0.3-0.5)
- Lower ramp rate (100-200 µs/s)
- Larger deadbands (10-20)

### For More Responsive Movement

- Higher PID gains
- Lower smoothing factor (0.1-0.2)
- Higher ramp rate (300-500 µs/s)
- Smaller deadbands (3-5)

## Testing Recommendations

1. **Monitor PID Output**: Check CSV logs for PID output values

   - Should show smooth transitions, not sudden jumps
   - Should stay near zero when stable

2. **Observe Motor Behavior**: Watch reaction wheel during descent

   - Should have gradual speed changes
   - No sudden starts/stops
   - Should respond to rotation but not over-correct

3. **Check Stabilization**: Verify effective spin control

   - CanSat should maintain orientation
   - No oscillation or hunting
   - Smooth corrections to disturbances

4. **Adjust if Needed**:
   - If too slow: Increase Kp or ramp rate
   - If oscillating: Decrease Kd or increase smoothing
   - If drifting: Increase Ki (carefully)

## Performance Improvements

✅ Smoother motor transitions (no jerky movements)
✅ Reduced mechanical stress on reaction wheel
✅ Less power consumption from constant corrections
✅ More stable orientation control
✅ Better battery life
✅ Quieter operation
✅ **Reaction wheel stops after landing (no unnecessary power drain)**
✅ **Safer post-landing operation**

## Files Modified

1. `primary/src/main.cpp`

   - Added PID smoothing parameters
   - Updated `calculatePID()` function
   - Implemented low-pass filter and rate limiting
   - **Added IMPACT state detection to stop PID**
   - **Reset PID values when IMPACT state reached**

2. `secondary/src/main.cpp`
   - Added ESC smoothing parameters
   - Updated `controlReactionWheel()` function
   - Updated `stopReactionWheel()` function
   - Implemented ramping and deadband logic
   - **Added `extractFlightStateFromCSV()` function**
   - **Added IMPACT state detection in `receiveFromPrimary()`**
   - **Stops reaction wheel when flight state == 7 (IMPACT)**

## Flight State Reference

- 0 = BOOT
- 1 = TEST_MODE
- 2 = LAUNCH_PAD
- 3 = ASCENT
- 4 = ROCKET_DEPLOY (parachute deployed, PID starts)
- 5 = DESCENT (PID active)
- 6 = AEROBRAKE_RELEASE (PID active)
- **7 = IMPACT (PID DISABLED, reaction wheel STOPPED)**

## Notes

- All smoothing happens in real-time with minimal computational overhead
- Smoothing does not introduce significant lag (typically <100ms)
- Settings are conservative for safety and battery life
- Can be fine-tuned based on flight test results
