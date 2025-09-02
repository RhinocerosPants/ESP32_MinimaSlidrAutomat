# ESP32 MinimaSlidrAutomat

**Smart Automation Retrofit for Existing Camera Sliders**

Transform your manual camera slider into a self-calibrating, intelligent motion control system with automatic ping-pong motion and manual positioning capabilities.

## Project Credits

**Project by:** Aaron Young  
**Development:** With assistance from Anthropic's Claude Code  
**Repository:** https://github.com/RhinocerosPants/ESP32_MinimaSlidrAutomat  
**Version:** 2.0 - Production Release

## Project Overview

This project is designed for **creative tinkerers and budget-conscious creators** who want to add intelligent automation to their existing camera slider systems. It combines:

- **Electronics**: ESP32 microcontroller, stepper motor control, sensor integration
- **Programming**: Arduino C++ firmware with sophisticated state machine architecture  
- **3D Design & Printing**: Custom mounting brackets and sensor housings
- **Mechanical Integration**: Retrofit installation onto existing slider hardware

The system uses dual hall effect sensors to automatically measure your slider's travel distance and creates a self-calibrating ping-pong motion system with no manual configuration required.

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

### Electronic Components
- **ESP32 development board** (DevKit V1 or similar)
- **TMC2208 stepper motor driver** (basic step/dir mode)
- **NEMA 17 stepper motor** (1.8° step angle recommended)
- **2x Digital Hall effect sensors** (A3144, OH3144, or US1881 - digital switching types only)
- **WS2812B LED strip** (5 LEDs)
- **4x Push buttons** (momentary, normally open - 3 for manual control, 1 for reset)
- **USB-C PD breakout board** (configurable to 12V output)
- **LM2596 buck converter module** (12V to 5V, 3A capacity)
- **Wire and connectors** (for hall sensor connections)
- **2x Neodymium magnets** (for sensor triggering)

### Mechanical Components (Retrofit Requirements)
- **Existing camera slider** (this project retrofits your current slider)
- **Motor mounting bracket** (3D printed or custom fabricated)
- **Sensor mounting brackets** (3D printed housings for hall sensors)
- **Belt/pulley system** (if not already present on your slider)
- **Electronics enclosure** (3D printed or purchased, weather protection recommended)

## Power System Architecture

### Power Distribution
```
USB-C PD (12V) ──┬── TMC2208 VM (Motor Power)
                 │
                 └── LM2596 Buck Module (5V/3A)
                     │
                     ├── ESP32 VIN (→ Internal 3.3V regulator)
                     ├── WS2812B LED Strip (5V)
                     ├── TMC2208 VDD (Logic Power)
                     └── Digital Hall Sensors (5V)
```

### Power Budget
| Component | Current Draw | Supply Rail | Power |
|-----------|--------------|-------------|--------|
| ESP32 (active) | ~250mA | 5V→3.3V | 1.25W |
| WS2812B LEDs (5x) | ~300mA | 5V | 1.5W |
| Digital Hall Sensors (2x) | ~20mA | 5V | 0.1W |
| TMC2208 Logic | ~100mA | 5V | 0.5W |
| TMC2208 + NEMA17 | ~1.5A | 12V | 18W |
| **Total System** | **~2.17A** | **Mixed** | **21.35W** |

**5V Rail Load:** 670mA (well within LM2596 3A capacity)  
**12V Rail Load:** 1.5A (motor operation)

## Detailed Wiring Diagram

### Power Connections
```
USB-C PD BREAKOUT:
12V+ ──┬── TMC2208 VM
       └── LM2596 VIN

GND ───┴── Common Ground Bus (all components)

LM2596 5V OUT ──┬── ESP32 VIN
                ├── WS2812B 5V
                ├── TMC2208 VDD  
                └── Digital Hall Sensor VCC (both sensors)
```

### Signal Connections
```
ESP32 Pin -> Component
Pin 2     -> TMC2208 STEP
Pin 4     -> TMC2208 DIR  
Pin 5     -> TMC2208 EN (Enable)
Pin 21    -> Digital Hall Sensor #1 OUT (Home A - LEFT END)
Pin 22    -> Digital Hall Sensor #2 OUT (Home B - RIGHT END)
Pin 23    -> WS2812B LED Data In
Pin 25    -> Move to A Button (with internal pullup to 3.3V)
Pin 26    -> Move to B Button (with internal pullup to 3.3V)
Pin 27    -> Return to Ping-pong Button (with internal pullup to 3.3V)
EN        -> Reset Button (connects EN pin to GND for system reset)
Digital Hall Sensors have active push-pull outputs - no pull-up resistors needed
GND       -> All component ground connections
```

### Hall Sensor Wiring Detail
```
Digital Hall Sensor #1 (Home A):    Digital Hall Sensor #2 (Home B):
Pin 1 (VCC) -> 5V                    Pin 1 (VCC) -> 5V
Pin 2 (GND) -> GND                   Pin 2 (GND) -> GND  
Pin 3 (OUT) -> ESP32 GPIO21 (direct) Pin 3 (OUT) -> ESP32 GPIO22 (direct)

Note: Digital hall sensors (A3144/OH3144/US1881) have active push-pull outputs
No pull-up resistors required - sensors actively drive HIGH/LOW states
```

### Button Wiring
```
Manual Control Buttons: One side to ESP32 GPIO, other side to GND
ESP32 internal pull-ups enabled in software (INPUT_PULLUP)

Reset Button: One side to ESP32 EN pin, other side to GND
Pressing resets the ESP32 immediately (hardware reset)
```

## Complete Bill of Materials (BOM)

### Electronic Components - Core System
| Component | Quantity | Specification | Est. Price | Purpose |
|-----------|----------|--------------|------------|---------|
| ESP32 DevKit V1 | 1 | 30-pin development board | $8-12 | Main controller |
| TMC2208 Stepper Driver | 1 | V3.0 with heatsink | $8-15 | Motor driver |
| NEMA 17 Stepper Motor | 1 | 1.8°, 1.5A, 4-wire | $15-25 | Motion control |
| Digital Hall Sensors | 2 | A3144/OH3144/US1881, TO-92 | $1-3 each | Position sensing |
| WS2812B LED Strip | 1 | 5 LEDs, 5V addressable | $5-8 | Status indication |
| Push Buttons | 4 | 6mm tactile, momentary NO | $1-2 each | Manual control + reset |

### Power System
| Component | Quantity | Specification | Est. Price | Purpose |
|-----------|----------|--------------|------------|---------|
| USB-C PD Breakout | 1 | 12V configurable output | $10-20 | Primary power |
| LM2596 Buck Module | 1 | 12V→5V, 3A adjustable | $3-8 | 5V regulation |
| Wire & Connectors | 1 set | 22-24 AWG, JST/Dupont | $5-10 | Hall sensor connections |
| Neodymium Magnets | 2 | N35, 10-15mm diameter | $2-5 each | Sensor triggering |

### Connection Hardware
| Component | Quantity | Specification | Est. Price | Purpose |
|-----------|----------|--------------|------------|---------|
| Breadboard/Perfboard | 1 | Half-size or larger | $3-8 | Prototyping |
| Jumper Wires | 1 set | Male-male, various lengths | $5-10 | Connections |
| Header Pins | As needed | 2.54mm pitch | $2-5 | Board connections |
| Terminal Blocks | 2-3 | Screw terminals | $3-8 | Power connections |

### Retrofit Hardware (As Needed)
| Component | Quantity | Specification | Est. Price | Purpose |
|-----------|----------|--------------|------------|---------|
| Timing Belt | 1 | GT2, length to match slider | $5-15 | Motion transmission (if needed) |
| Belt Pulleys | 2 | GT2, 20-tooth | $3-8 each | Belt drive (if needed) |
| 3D Printing Filament | ~200g | PLA or PETG | $5-10 | Custom brackets and housings |
| M3/M4 Bolts & Nuts | Assorted | Stainless steel | $5-15 | Mounting hardware |
| Aluminum Extrusion | As needed | 20x20 or 30x30mm | Variable | Custom brackets (alternative to 3D printing) |

### Tools Required
| Item | Purpose |
|------|---------|
| Soldering iron & solder | Component assembly |
| Wire strippers | Cable preparation |
| Small screwdriver set | Mechanical assembly |
| Multimeter | Testing & debugging |
| Heat shrink tubing | Connection protection |
| 3D Printer | Custom brackets and sensor housings |
| CAD Software | Designing custom mounting solutions |
| Drill & bits | Mounting holes in existing slider |

### Sourcing Recommendations
- **Electronics**: AliExpress, Amazon, DigiKey, Mouser
- **Mechanical**: OpenBuilds, RobotShop, local machine shops
- **3D Printed Parts**: Custom brackets and mounts as needed

### TMC2208 Wiring (Basic Mode)
```
TMC2208 -> Connection
VDD     -> 5V (from LM2596)    // Logic power
GND     -> Common ground       // Logic ground
VM      -> 12V (from USB-C PD) // Motor power supply
GND     -> Common ground       // Motor power ground
1A, 1B  -> NEMA 17 Coil A     // Motor winding 1
2A, 2B  -> NEMA 17 Coil B     // Motor winding 2
STEP    -> ESP32 Pin 2         // Step pulse signal
DIR     -> ESP32 Pin 4         // Direction control
EN      -> ESP32 Pin 5         // Enable (active low)
```

### Power Supply Requirements Summary
- **USB-C PD**: 12V, minimum 2A capacity (24W minimum)
- **LM2596 Module**: Pre-configured for 5V output, 3A capacity  
- **Efficiency**: ~92% power conversion efficiency
- **Heat Generation**: Minimal (both supplies run cool)
- **Safety**: Fused input recommended (3A fast-blow fuse)

## Software Configuration

### Key Parameters (Calibrated Values)
```cpp
#define STEPS_PER_MM      46.24f    // Precisely calibrated for accurate positioning
#define MAX_SPEED         400       // Steps/sec (8.7mm/s smooth filming ping-pong)
#define MANUAL_SPEED      1600      // Steps/sec (34.6mm/s - 4x faster than ping-pong)
#define ACCELERATION      200       // Steps/sec² (smooth 4-second accel/decel)
#define HOMING_SPEED      2400      // Steps/sec (51.9mm/s for all pre-ping-pong movements)
#define SENSOR_CLEARANCE_MM  20.0f  // Clearance from sensors for ping-pong range
// Travel distance and ping-pong range measured automatically during dual sensor homing
float measuredTravelMM;             // Actual distance between sensors
float pingPongStartMM;              // Calculated start position
float pingPongEndMM;                // Calculated end position
```

### Timing Settings
```cpp
#define HOMING_SPEED      2400      // All pre-ping-pong movements (51.9mm/s)
#define HOMING_TIMEOUT_MS 40000     // 40-second timeout per sensor (1000mm travel limit)
#define START_PAUSE_MS    3000      // 3-second pause before ping-pong
```

## Operation Sequence

### 1. Power-On Boot
- Initializes hardware (stepper, dual hall sensors, serial)
- Enables motor driver
- Begins automatic dual sensor homing sequence

### 2. Home A Homing Phase
- **Speed**: 51.9mm/s (constant speed, no acceleration) 
- **Direction**: Moves left (negative) to find Home A sensor
- **Detection**: Stops immediately when Home A sensor activates
- **Timeout**: 40 seconds maximum (1000mm safe travel limit)
- **Completion**: Sets Home A sensor position as 0mm reference
- **LED Status**: Breathing amber animation during homing

### 3. Home B Homing Phase  
- **Speed**: 51.9mm/s (constant speed, no acceleration)
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
- **Speed**: 8.7mm/s maximum speed (smooth cinematic motion)
- **Acceleration**: 4-second smooth acceleration/deceleration
- **Pattern**: Continuous back-and-forth motion within calculated safe limits
- **Duration**: Infinite (until manual control or power removed)
- **LED Status**: Green ping-pong animation following motion direction

## Manual Control Operation

The system includes three manual control buttons that can interrupt ping-pong mode for precise positioning:

### Button Functions
- **Move to A Button (GPIO 25)**: Move to Home A position + clearance (20mm from sensor)
- **Move to B Button (GPIO 26)**: Move to Home B position - clearance (20mm from sensor)  
- **Return to Ping-pong (GPIO 27)**: Return to automatic ping-pong mode from manual mode

### Manual Operation Sequence

#### From Ping-pong Mode:
1. **Button Press**: Press Move to A or Move to B button during ping-pong operation
2. **Transition**: System stops ping-pong and switches to manual mode
3. **Movement**: Accelerated motion to target position at 34.6mm/s (4x faster than ping-pong)
4. **Arrival**: System holds position and waits for next button press
5. **LED Status**: Purple sweep animation during movement, purple breathing when stopped

#### In Manual Mode:
- **A ↔ B Switching**: Press opposite button to move between A and B positions
- **Return to Ping-pong**: Press Return button to restart full homing sequence
- **LED Status**: Purple sweep animation shows direction during movement, purple breathing when stopped
- **Speed**: 4x faster than ping-pong for quick repositioning

#### Manual Mode Characteristics:
- **Speed**: 1600 steps/sec (34.6mm/s) - optimized for quick positioning
- **Motion Type**: Accelerated movement with 2x acceleration for responsive feel
- **Target Positions**: Safe clearance positions (not actual sensor locations)
- **Recovery**: Power cycle or Return button restarts full automatic sequence

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
STATE_MOVING_TO_START → (move to calculated start position)
    ↓
STATE_PAUSING_AT_START → (3-second pause before ping-pong)
    ↓
STATE_PING_PONG ←→ (continuous motion between calculated endpoints)
    ↓         ↑
    ↓ (button press)
    ↓         ↑ (Return button)
    ↓         ↑
STATE_MANUAL_MOVE_TO_A → STATE_MANUAL_WAIT_A ←→ STATE_MANUAL_MOVE_TO_B → STATE_MANUAL_WAIT_B
                                ↑                                              ↓
                                ↑______________________________________________|
                                                (A/B switching)

STATE_ERROR → (permanent error, motor holds position, power cycle required)
```

### State Descriptions
- **Automatic States**: BOOT → HOMING_A → HOMING_B → MEASURING → MOVING → PAUSING → PING_PONG
- **Manual States**: MANUAL_MOVE_TO_A/B (moving), MANUAL_WAIT_A/B (positioned and waiting)
- **Transitions**: Button presses interrupt ping-pong; Return button restarts homing sequence
- **Error State**: Accessible from any state, requires power cycle to recover

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
- **🟣 Purple Manual Mode**: Manual control mode with directional animations
- **🔴 Breathing Red**: Error state - crash detected, timeout, or bounds exceeded

### LED Behaviors
- **Breathing Effect**: Smooth fade in/out animation (30ms updates)
- **Ping-Pong Effect**: Single LED moves left-right with trailing dim LED (225ms updates - 50% slower than previous versions)
- **Manual Movement**: Purple sweep animation showing direction of travel during movement
- **Manual Stopped**: Purple breathing animation when stopped in manual mode
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
Most errors require **reset** to recover:
1. **Motor will hold position** during error state (safe for inclines)
2. **Hardware Reset**: Press the reset button (connects EN to GND) for immediate restart
3. **Power Cycle**: Alternatively, power off ESP32 completely then power on
4. Check and fix underlying issue before restarting
5. System will restart from beginning with full homing sequence
6. **Note**: Motor maintains holding torque until reset for safety

## Customization

### Adjusting Motion Parameters

**Speed Changes:**
```cpp
#define MAX_SPEED 200    // Slower: 4.3mm/s instead of 8.7mm/s  
#define MAX_SPEED 800    // Faster: 17.3mm/s instead of 8.7mm/s
#define MANUAL_SPEED 800 // Slower manual: 17.3mm/s instead of 34.6mm/s
#define MANUAL_SPEED 3200 // Faster manual: 69.2mm/s instead of 34.6mm/s
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