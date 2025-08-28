# ESP32_MinimaSlidrAutomat - Calibration Log

This document records the calibration process and measurements used to determine the accurate motion parameters for the MinimaSlidrAutomat system.

## Hardware Configuration

**Stepper Motor**: NEMA17
**Driver**: TMC2208 (basic step/dir mode)
**Rail System**: 639mm total physical travel
**Measurement Tool**: Laser measurement system

## Calibration History

### Initial Estimates (Incorrect)
```cpp
#define STEPS_PER_MM 53.33f  // Based on theoretical calculations
#define TOTAL_TRAVEL_MM 600.0f
```

**Result**: Significant overruns, inaccurate positioning

### First Measurement Test
**Test Parameters**:
- Command: 400mm movement
- Actual: 374mm measured movement
- Ratio: 374/400 = 0.935

**Calculation Attempt 1** (Incorrect logic):
```cpp
New steps/mm = 50.08 × 0.935 = 46.82 steps/mm
```

**Result**: Still overrunning - calculation was backwards

### Second Measurement Test  
**Test Parameters**:
- Command: 400mm movement  
- Actual: 374mm measured movement
- Steps sent: 50.08 × 400 = 20,032 steps

**Calculation Attempt 2** (Correct logic):
```cpp
Actual steps/mm = 20,032 steps ÷ 374mm = 53.56 steps/mm
```

**Result**: Better accuracy but still had issues

### Third Measurement - End Detection
**Test Scenario**: Ping-pong motion to physical limit
- Command: 550mm end position
- Actual: Hit physical limit at 441.7mm (serial monitor reading)
- Steps sent: 53.56 × 550 = 29,458 steps

**Calculation Attempt 3** (Major error):
```cpp
"Correct" steps/mm = 29,458 ÷ 441.7 = 66.70 steps/mm  
```

**Result**: Massive overruns - calculation logic was wrong again

### Fourth Correction - Understanding the Problem
**Realization**: When it hits the end at 441.7mm while commanded 550mm:
- The motor moved LESS than commanded (underrunning)
- Steps-per-mm was TOO HIGH
- Need to REDUCE steps-per-mm

**Calculation Attempt 4** (Correct):
```cpp
Correction ratio = 441.7 ÷ 550 = 0.803
New steps/mm = 53.56 × 0.803 = 43.01 steps/mm
```

**Result**: Much better accuracy, minimal overrun

### Final Laser Measurement Calibration
**Test Setup**: Laser measurement of complete ping-pong cycle
- **Commanded range**: 30mm to 550mm = 520mm
- **Laser measured**: 559mm actual movement
- **User feedback**: "I like the actual movement range"

**Final Calculation** (Position reporting accuracy):
```cpp
Calibration factor = 559 ÷ 520 = 1.075  
Final steps/mm = 43.01 × 1.075 = 46.24 steps/mm
```

## Final Calibrated Values

```cpp
// Motion constants - FINAL CALIBRATION
#define STEPS_PER_MM      46.24f    // Laser-calibrated for accurate position reporting
#define MAX_SPEED         800       // Steps per second (17.3mm/s actual)
#define ACCELERATION      200       // Steps per second squared (4-second smooth curves)  
#define HOMING_SPEED      1000      // Steps per second (21.6mm/s for fast homing)
#define STARTUP_SPEED     1000      // Steps per second (21.6mm/s for positioning)

#define TOTAL_TRAVEL_MM   580.0f    // Conservative travel length (within 639mm physical)
#define SAFETY_MARGIN     10.0f     // General safety margin from limits  
#define ACCEL_DECEL_MARGIN 35.0f    // Extra margin for acceleration/deceleration distance
#define PING_PONG_START   30.0f     // Start position (hall sensor clearance)
#define PING_PONG_END     515.0f    // End position (accounts for accel/decel distance)
```

## Verification Results

**Position Accuracy**: Serial monitor positions now match actual measured positions
**Travel Range**: 485mm usable ping-pong range (30mm to 515mm)
**Physical Movement**: 559mm actual travel distance (user-preferred range)
**Safety**: No overruns with 35mm acceleration distance compensation
**Speed**: 17.3mm/s actual maximum speed during ping-pong motion

## Key Lessons Learned

### 1. Measurement Interpretation
- **Underrunning** (actual < commanded) means steps-per-mm is TOO HIGH
- **Overrunning** (actual > commanded) means steps-per-mm is TOO LOW  
- Always consider which direction the error goes

### 2. Acceleration Distance Impact
- Slower acceleration (200 vs 400 steps/sec²) requires more distance
- Must account for ~35mm extra travel for acceleration/deceleration
- Failing to account for this causes end-of-travel crashes

### 3. User Preference vs Accuracy
- User preferred the 559mm actual movement range
- Calibrated software to accurately report positions during this movement
- Better to match software to hardware than force hardware to match software

### 4. Calibration Process
1. **Measure actual movement** with reliable method (laser)
2. **Compare to commanded movement** 
3. **Calculate correction ratio**
4. **Apply ratio to current steps-per-mm**
5. **Verify with test movements**
6. **Account for acceleration distance**

## Measurement Tools Used

**Laser Measurement System**: Primary measurement tool for final calibration
- Provides precise distance measurements
- Used for complete ping-pong cycle measurement
- Confirmed 559mm actual travel range

**Serial Monitor**: Secondary feedback tool  
- Reports calculated positions during movement
- Used for real-time position tracking
- Helpful for detecting overruns and crashes

**Physical End Stops**: Reference points
- 639mm total rail travel (physical limit)
- Used to detect overrun conditions
- Confirmed when motion exceeded safe bounds

## Future Recalibration Notes

If hardware changes require recalibration:

1. **Run controlled test movement** (e.g., 400mm command)
2. **Measure actual distance** with laser
3. **Calculate ratio**: Actual ÷ Commanded
4. **Update steps-per-mm**: Current × Ratio
5. **Test full ping-pong range**
6. **Verify no overruns occur**
7. **Check acceleration distance compensation**

## Calibration Validation

The final calibrated values have been validated through:
- Multiple complete ping-pong cycles without overrun
- Laser measurement confirmation of travel distances  
- Serial position reporting accuracy verification
- Long-term operation stability testing
- Crash detection system functionality testing

System is now precisely calibrated for reliable, accurate operation.