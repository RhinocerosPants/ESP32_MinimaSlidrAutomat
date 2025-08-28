# ESP32 MinimaSlidrAutomat

A self-calibrating ESP32-based camera slider with automatic ping-pong motion and manual control capabilities.

## Project Credits

**Project by:** RhinocerosPants Team - Aaron Young  
**Development:** With assistance from Anthropic's Claude Code  
**Repository:** https://github.com/RhinocerosPants/ESP32_MinimaSlidrAutomat  
**Version:** 2.0 - Production Release

A fully autonomous ESP32 camera slider with dual hall effect sensors that automatically measures travel distance and runs continuous ping-pong motion without any user configuration.

## Overview

MinimaSlidrAutomat is designed for simplicity and self-calibration. Upon power-up, it:
1. Automatically finds Home A sensor (left end)
2. Moves to find Home B sensor (right end) 
3. Measures actual travel distance between sensors
4. Calculates optimal ping-pong range automatically
5. Moves to start position with a 3-second pause
6. Begins continuous ping-pong motion within measured range
7. Runs indefinitely until power is removed

## Features

- **Zero-configuration operation** - No web interface, WiFi, or manual measurement required
- **Dual sensor auto-calibration** - Automatically measures travel distance between sensors
- **Self-configuring range** - Calculates safe ping-pong limits with sensor clearance
- **Automatic homing** - Uses dual hall effect sensors for precise positioning
- **Smooth cinematic motion** - Long acceleration/deceleration curves for professional footage
- **Manual control** - Three buttons for manual positioning and mode switching
- **Dual sensor crash detection** - Monitors both sensors during operation for safety
- **Precise calibration** - Accurate position reporting matching actual movement
- **Fast startup** - Very quick dual sensor homing before smooth operation
- **Visual status indicators** - LED strip shows homing, ping-pong, manual, and error states
- **Universal compatibility** - Adapts to any rail length automatically

## Hardware Requirements

### Components
- ESP32 development board
- TMC2208 stepper motor driver (basic step/dir mode)
- NEMA17 stepper motor
- **2x Hall effect sensors** (A3144, OH3144 or similar - active low)
- WS2812B LED strip (5 LEDs)
- Camera slider rail system (any length)
- 12V power supply
- **2x Neodymium magnets** (for sensor triggering)

### Connections
```
ESP32 Pin -> Component
Pin 2     -> TMC2208 STEP
Pin 4     -> TMC2208 DIR  
Pin 5     -> TMC2208 EN (Enable)
Pin 21    -> Home A Hall Sensor Signal (with pullup) - LEFT END
Pin 22    -> Home B Hall Sensor Signal (with pullup) - RIGHT END
Pin 23    -> WS2812B LED Data In
Pin 25    -> Move to A Button (with internal pullup)
Pin 26    -> Move to B Button (with internal pullup)
Pin 27    -> Return to Ping-pong Button (with internal pullup)
GND       -> Both Hall Sensors GND
3.3V      -> Both Hall Sensors VCC
5V        -> WS2812B Power (VDD)
GND       -> WS2812B Ground
```

### TMC2208 Wiring (Basic Mode)
```
TMC2208 -> Connection
VDD     -> 3.3V
GND     -> GND
VM      -> 12V Motor Supply
GND     -> Motor Supply GND
1A, 1B  -> Motor Coil 1
2A, 2B  -> Motor Coil 2
STEP    -> ESP32 Pin 2
DIR     -> ESP32 Pin 4
EN      -> ESP32 Pin 5
```

## Software Configuration

### Key Parameters (Calibrated Values)
```cpp
#define STEPS_PER_MM      46.24f    // Precisely calibrated for accurate positioning
#define MAX_SPEED         800       // Steps/sec (17.3mm/s actual speed)
#define ACCELERATION      200       // Steps/sec² (smooth 4-second accel/decel)
#define HOMING_SPEED      4624      // Steps/sec (100mm/s for all pre-ping-pong movements)
#define SENSOR_CLEARANCE_MM  20.0f  // Clearance from sensors for ping-pong range
// Travel distance and ping-pong range measured automatically during dual sensor homing
float measuredTravelMM;             // Actual distance between sensors
float pingPongStartMM;              // Calculated start position
float pingPongEndMM;                // Calculated end position
```

### Timing Settings
```cpp
#define HOMING_SPEED      4624      // All pre-ping-pong movements (100mm/s)
#define HOMING_TIMEOUT_MS 40000     // 40-second timeout per sensor (1000mm travel limit)
#define START_PAUSE_MS    3000      // 3-second pause before ping-pong
```

## Operation Sequence

### 1. Power-On Boot
- Initializes hardware (stepper, dual hall sensors, serial)
- Enables motor driver
- Begins automatic dual sensor homing sequence

### 2. Home A Homing Phase
- **Speed**: 100mm/s (constant speed, no acceleration) 
- **Direction**: Moves left (negative) to find Home A sensor
- **Detection**: Stops immediately when Home A sensor activates
- **Timeout**: 40 seconds maximum (1000mm safe travel limit)
- **Completion**: Sets Home A sensor position as 0mm reference
- **LED Status**: Breathing amber animation during homing

### 3. Home B Homing Phase  
- **Speed**: 100mm/s (constant speed, no acceleration)
- **Direction**: Moves right (positive) to find Home B sensor
- **Detection**: Stops immediately when Home B sensor activates
- **Timeout**: 40 seconds maximum (1000mm safe travel limit)
- **Measurement**: Records actual travel distance between sensors
- **LED Status**: Continues breathing amber animation

### 4. Travel Range Calculation
- **Measurement**: Calculates actual distance between Home A and Home B
- **Range Calculation**: Determines safe ping-pong limits with sensor clearance
- **Safety Check**: Validates minimum 100mm travel range available
- **Configuration**: Sets pingPongStartMM and pingPongEndMM automatically

### 5. Start Pause
- **Duration**: 3 seconds
- **Purpose**: Time to prepare camera/recording equipment  
- **Position**: Pauses at Home B end (travel end position)
- **Status**: Reports "PAUSING_AT_START" state

### 6. Ping-Pong Motion
- **Range**: Calculated automatically based on measured travel distance
- **Example**: If 500mm travel measured, ping-pong from 20mm to 445mm (425mm range)
- **Speed**: 17.3mm/s maximum speed
- **Acceleration**: 4-second smooth acceleration/deceleration
- **Pattern**: Continuous back-and-forth motion within calculated safe limits
- **Duration**: Infinite (until power removed)
- **LED Status**: Green ping-pong animation following motion direction

## State Machine

```
STATE_BOOT
    ↓
STATE_HOMING_A → (find Home A sensor - left end, wait for motor stop)
    ↓
STATE_HOMING_B → (find Home B sensor - right end, wait for motor stop)
    ↓
STATE_MEASURING_TRAVEL → (calculate ping-pong range from measured travel)
    ↓
STATE_PAUSING_AT_START → (3-second pause at Home B end)
    ↓
STATE_PING_PONG → (continuous motion from Home B to Home A and back)
    ↑_____________↓
    
STATE_ERROR → (permanent error, motor holds position, power cycle required)
```

## Safety Features

### Crash Detection
- **Dual Sensor Monitoring**: Monitors both Home A and Home B sensors during operation
- **Home A Trigger**: Sensor activation when position > sensor clearance (20mm)
- **Home B Trigger**: Sensor activation when position < (measured travel - clearance)
- **Response**: Immediate emergency stop, **motor holds position**
- **Recovery**: Power cycle required
- **Purpose**: Detects mechanical collisions, belt slippage, or sensor interference
- **Safety**: Motor maintains holding torque to prevent sliding on inclines

### Position Bounds Checking  
- **Dynamic Range**: -50mm to (measured travel + 50mm) safety bounds
- **Example**: If 500mm travel measured, bounds are -50mm to 550mm
- **Response**: Emergency stop, **motor holds position**
- **Purpose**: Prevents runaway motion beyond physically possible limits
- **Safety**: Motor maintains holding torque to prevent sliding on inclines

### Homing Timeout
- **Duration**: 40 seconds maximum
- **Purpose**: Prevents infinite homing if sensor fails
- **Response**: Error state, **motor holds position**

### Acceleration Distance Compensation
- **Margin**: 35mm buffer for acceleration/deceleration
- **Purpose**: Prevents overrun due to motion physics
- **Implementation**: Reduced end position from 550mm to 515mm

### Incline Safety System
- **Problem**: On inclined setups, motor disable would cause uncontrolled sliding
- **Solution**: Motor maintains holding torque during error states
- **Benefit**: Protects expensive camera equipment from crashes
- **Operation**: stepper.stop() called but motor stays enabled
- **Recovery**: Power cycle required to restart from safe state

## LED Status Indicators

MinimaSlidrAutomat features a 5-LED WS2812B strip providing visual status indication:

### LED States
- **🟠 Breathing Amber**: Homing phase - searching for hall sensor
- **🟢 Green Ping-Pong**: Normal operation - LED moves back and forth following slider motion  
- **🔴 Breathing Red**: Error state - crash detected, timeout, or bounds exceeded

### LED Behaviors
- **Breathing Effect**: Smooth fade in/out animation (30ms updates)
- **Ping-Pong Effect**: Single LED moves left-right with trailing dim LED (150ms updates)
- **Test Pattern**: Blue sequence flash on startup to verify LEDs working

### Hardware
- **LEDs**: 5x WS2812B addressable RGB LEDs
- **Connection**: Data pin 23, 5V power, GND
- **Brightness**: 25% default (64/255) for comfortable viewing
- **Protocol**: FastLED library with GRB color order

## Serial Monitor Output

### Startup Messages
```
MinimaSlidrAutomat - Minimal Camera Slider
Starting initialization...
Hardware initialized
Travel: 580.0mm | Steps/mm: 46.24 | Ping-pong: 30.0mm to 515.0mm
Starting automatic homing...
```

### Status Updates (Every 2 seconds)
```
State: HOMING_A | Position: -15.2mm | Target: -1000.0mm | Hall A: inactive | Motor: RUNNING
State: HOMING_B | Position: 0.0mm | Target: 1000.0mm | Hall B: inactive | Motor: RUNNING
State: MEASURING_TRAVEL | Travel: 580.0mm | Ping-pong: 20.0mm to 560.0mm | Motor: stopped
State: PAUSING_AT_START | Position: 580.0mm | Target: 580.0mm | Hall: inactive | Motor: stopped
State: PING_PONG | Position: 245.3mm | Target: 20.0mm | Hall: inactive | Motor: RUNNING
```

### Error Messages
```
ERROR: Homing timeout - no hall sensor found
CRASH DETECTED: Hall sensor active at 245.3mm (should only be at 0mm)
Emergency stop - HOLDING POSITION for safety on inclines
ERROR: Position out of bounds (723.1mm) - emergency stop
HOLDING POSITION for safety on inclines
SYSTEM ERROR - Motor holding position - Power cycle required to restart
```

## Calibration Process

The current values were calibrated using laser measurement:

### Steps-per-MM Calibration
1. **Commanded distance**: 520mm ping-pong range
2. **Laser measured**: 559mm actual movement
3. **Calculation**: 43.01 × (559 ÷ 520) = 46.24 steps/mm
4. **Result**: Position reporting now matches actual movement

### Travel Distance Verification
- **Physical rail**: ~639mm maximum travel
- **Software limit**: 580mm (conservative safety margin)
- **Ping-pong range**: 485mm usable travel
- **Acceleration margin**: 35mm buffer for smooth motion

## Troubleshooting

### Common Issues

**System won't home**
- Check dual hall sensor connections (Pin 21 for Home A, Pin 22 for Home B, GND, 3.3V)
- Verify sensor polarity (active low with pullup)
- Ensure Home A sensor is positioned at left end and Home B sensor at right end of travel
- Check 40-second timeout hasn't expired for each sensor

**Motor doesn't move**
- Verify TMC2208 connections (STEP=Pin2, DIR=Pin4, EN=Pin5)
- Check 12V motor power supply
- Ensure motor coils properly connected
- Test Enable pin (should be LOW when running)

**Overruns end of travel**
- Check steps-per-mm calibration (currently 46.24)
- Verify acceleration distance compensation (35mm margin)
- Reduce PING_PONG_END value if necessary
- Check for mechanical binding or belt stretch

**Crashes detected falsely**
- Verify hall sensor isn't triggered by magnetic interference
- Check sensor mounting - should only activate at home position
- Adjust crash detection threshold (currently 30mm)

**Jerky or harsh motion**
- Increase ACCELERATION value for gentler motion (currently 200)
- Check power supply stability under load
- Verify stepper motor current settings
- Test with lower MAX_SPEED

### Error Recovery
Most errors require **power cycle** to reset:
1. **Motor will hold position** during error state (safe for inclines)
2. Power off ESP32 completely
3. Check and fix underlying issue
4. Power on - system will restart from beginning
5. **Note**: Motor maintains holding torque until power cycle for safety

## Customization

### Adjusting Motion Parameters

**Speed Changes:**
```cpp
#define MAX_SPEED 600    // Slower: 13mm/s instead of 17.3mm/s  
#define MAX_SPEED 1000   // Faster: 21.6mm/s instead of 17.3mm/s
```

**Acceleration Changes:**
```cpp
#define ACCELERATION 100  // Gentler: 8-second accel/decel
#define ACCELERATION 400  // Sharper: 2-second accel/decel  
```

**Travel Range Changes:**
```cpp
#define PING_PONG_START 20.0f   // Closer to sensor
#define PING_PONG_END   520.0f  // Longer travel (check overrun!)
```

**Pause Duration:**
```cpp
#define START_PAUSE_MS 5000  // 5-second pause instead of 3
#define START_PAUSE_MS 1000  // 1-second pause instead of 3
```

### Re-calibration Process

If mechanical changes are made:

1. **Measure actual travel** with laser/ruler during ping-pong
2. **Calculate new steps-per-mm**:
   ```
   New = Current × (Commanded ÷ Actual)
   ```
3. **Update STEPS_PER_MM** in code
4. **Verify no overruns** occur
5. **Test crash detection** still works

## Dependencies

### Arduino Libraries
```cpp
#include <AccelStepper.h>  // Stepper motor control with acceleration
#include <FastLED.h>       // WS2812B LED strip control
```

### Installation
1. Install ESP32 board support in Arduino IDE
2. Install AccelStepper library via Library Manager  
3. Install FastLED library via Library Manager
4. Upload ESP32_MinimaSlidrAutomat.ino to ESP32
5. Open Serial Monitor (115200 baud) to view status

## Version History

**v1.0** - Initial release with basic functionality
- Automatic homing and ping-pong motion
- Basic safety features and serial output

**v1.1** - Calibration and safety improvements  
- Precise steps-per-mm calibration (46.24)
- Crash detection via hall sensor monitoring
- Acceleration distance compensation
- Extended homing timeout and improved error handling

**v1.2** - Motion quality enhancements
- 3-second startup pause for camera preparation
- Doubled acceleration time for smoother motion (4-second curves)
- Fast homing and positioning (21.6mm/s)
- Improved serial status reporting

**v1.3** - Visual status indicators
- Added WS2812B LED strip support (5 LEDs)
- Breathing amber animation during homing phase
- Green ping-pong animation during normal operation
- Breathing red animation for error states
- Startup LED test pattern for verification

**v2.0** - Production Release
- Professional code cleanup with named constants
- Refined homing speed to 100mm/s for optimal sensor detection
- Eliminated debug version strings and test code
- All pre-ping-pong movements use consistent constant speed
- Immediate emergency stops with no deceleration
- Production-ready firmware with comprehensive error handling
- Maintains all dual sensor auto-calibration features from v1.4

## License

Open source - modify and use freely for personal and commercial projects.

## Support

For issues or modifications, check:
1. Serial monitor output for error messages
2. Hardware connections per wiring diagrams  
3. Calibration values match your mechanical setup
4. Power supply stability under motor load

System designed for reliable, autonomous operation with minimal maintenance required.