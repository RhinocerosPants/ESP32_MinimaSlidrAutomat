# ESP32_MinimaSlidrAutomat - Hardware Setup Guide

Complete hardware setup and wiring guide for the MinimaSlidrAutomat self-calibrating dual sensor camera slider system.

## Bill of Materials

### Core Components
- **ESP32 Development Board** (DevKit V1 or similar)
- **TMC2208 Stepper Motor Driver** 
- **NEMA17 Stepper Motor** (1.8° step angle)
- **2x Hall Effect Sensors** (A3144, OH3144, or similar - active low)
- **2x Neodymium Magnets** (5-10mm diameter, 2-3mm thick)
- **WS2812B LED Strip** (5 LEDs)
- **Camera Slider Rail** (any length - automatically measured)
- **GT2 Timing Belt and Pulleys**
- **Linear Bearings/Rails**
- **Motor Mount**
- **Camera Platform/Carriage**

### Power Supply
- **12V DC Power Supply** (2-3A minimum for motor)
- **DC Barrel Jack** or equivalent connector
- **Power Distribution Board** (optional)

### Wiring and Connectors
- **Jumper Wires** (male-to-male, male-to-female)
- **Breadboard** or **PCB** for connections
- **4-pin JST Connector** (for stepper motor - optional)
- **2x 3-pin JST Connectors** (for hall sensors - optional)
- **Heat Shrink Tubing**
- **Wire Management** (cable ties, cable chain)

## Wiring Diagrams

### ESP32 to TMC2208 Connections
```
ESP32 Pin    ->    TMC2208 Pin    ->    Function
Pin 2        ->    STEP           ->    Step pulse signal
Pin 4        ->    DIR            ->    Direction control  
Pin 5        ->    EN             ->    Enable (active low)
3.3V         ->    VDD            ->    Logic power
GND          ->    GND            ->    Logic ground
```

### TMC2208 to Motor Power
```
Power Supply ->    TMC2208 Pin    ->    Function
+12V         ->    VM             ->    Motor power supply
GND          ->    GND            ->    Motor power ground
```

### TMC2208 to Stepper Motor
```
TMC2208 Pin  ->    Motor Wire     ->    Function
1A           ->    Coil A+        ->    Motor coil 1 positive
1B           ->    Coil A-        ->    Motor coil 1 negative  
2A           ->    Coil B+        ->    Motor coil 2 positive
2B           ->    Coil B-        ->    Motor coil 2 negative
```

### Dual Hall Effect Sensor Connections
```
ESP32 Pin    ->    Sensor Pin     ->    Function
Pin 21       ->    Home A Signal/OUT  ->    Left end sensor (with internal pullup)
Pin 22       ->    Home B Signal/OUT  ->    Right end sensor (with internal pullup)  
3.3V         ->    Both VCC       ->    Sensor power for both sensors
GND          ->    Both GND       ->    Sensor ground for both sensors
```

### WS2812B LED Strip Connections  
```
ESP32 Pin    ->    LED Strip Pin  ->    Function
Pin 23       ->    Data In        ->    LED control signal
5V           ->    VDD/+5V        ->    LED strip power
GND          ->    GND            ->    LED strip ground
```

## Detailed Setup Instructions

### 1. TMC2208 Driver Configuration

**Mode Selection**: Use basic step/dir mode (no UART communication)
- **MS1**: Leave floating or connect to GND
- **MS2**: Leave floating or connect to GND  
- **Microstep Setting**: 16x microstepping (default)

**Current Setting**: Adjust VREF for motor current
- **Formula**: VREF = I_motor × 2.5 × R_sense × 0.7
- **Typical**: VREF ≈ 0.4-0.8V for NEMA17 motors
- **Adjustment**: Use small screwdriver on potentiometer

**Driver Orientation**: Ensure TMC2208 is oriented correctly
- **Pin 1 marker** (dot or notch) should align with board markings
- **Double-check** all pin connections before powering up

### 2. Dual Hall Effect Sensor Mounting

**Sensor Type**: Use active-low hall effect sensors (A3144, OH3144)  
- **Trigger**: Activates (LOW) when magnet is near
- **Default**: High (3.3V) when no magnet present
- **Internal Pullup**: ESP32 Pins 21 and 22 configured with INPUT_PULLUP

**Home A Sensor (Left End)**:
- Mount **Home A sensor** at **left end** of rail travel
- **ESP32 Pin 21** connection
- Position **perpendicular** to carriage movement
- **Distance**: 2-5mm gap when carriage is at left limit
- **Alignment**: Magnet on carriage should pass directly over sensor

**Home B Sensor (Right End)**:  
- Mount **Home B sensor** at **right end** of rail travel
- **ESP32 Pin 22** connection
- Position **perpendicular** to carriage movement
- **Distance**: 2-5mm gap when carriage is at right limit
- **Alignment**: Same magnet triggers both sensors as carriage moves

**Dual Magnet Configuration**:
- **Primary Option**: Single magnet on carriage triggers both sensors
- **Alternative**: Two magnets on carriage (one for each sensor)
- **Neodymium magnet** (strong, small)
- **Size**: 5-10mm diameter, 2-3mm thick
- **Mounting**: Secure to carriage, aligned with both sensor positions
- **Polarity**: Either pole can trigger sensors

**Critical Alignment**:
- Both sensors must detect the **same physical carriage position**
- Test sensor activation by manually moving carriage to each end
- Verify sensors activate only at intended positions

### 3. Stepper Motor Setup

**Motor Specifications**:
- **Step Angle**: 1.8° (200 steps per revolution)
- **Current**: 1.0-2.0A per phase typical
- **Voltage**: 12V recommended
- **Torque**: Sufficient for camera+carriage load

**Motor Mounting**:
- **Rigid mounting** to prevent vibration
- **Alignment** with belt/pulley system
- **Tensioning** mechanism for belt adjustment
- **Protection** from dust and debris

**Belt and Pulley System**:
- **Belt Type**: GT2 timing belt (2mm pitch)
- **Pulley**: 20-tooth GT2 pulley on motor shaft
- **Tension**: Firm but not over-tight
- **Length**: Match rail length + motor position

### 4. Power Supply Considerations

**Motor Power (12V)**:
- **Current**: 2-3A minimum for smooth operation
- **Regulation**: Stable voltage under load
- **Protection**: Fuse or circuit breaker recommended
- **Connector**: Secure, polarized connection

**Logic Power (3.3V)**:
- **Source**: ESP32 internal regulator or external 3.3V supply
- **Current**: <500mA for ESP32 + sensor
- **Isolation**: Separate ground paths if using external supply
- **Filtering**: Capacitors for noise reduction if needed

### 5. Safety Considerations

**Electrical Safety**:
- **Double-check** all connections before powering up
- **Fuse protection** on 12V supply line
- **Secure connections** - no loose wires
- **Insulation** of exposed conductors

**Mechanical Safety**:
- **End stops** or physical barriers at rail limits
- **Emergency stop** capability (power disconnect)
- **Secure mounting** of all components
- **Cable management** to prevent snagging

**Operational Safety**:
- **Clear travel path** for carriage movement
- **Camera security** to prevent drops
- **Supervision** during initial setup and testing
- **Power isolation** for maintenance

## Testing and Verification

### 1. Basic Connectivity Test
```cpp
// Simple test code to verify connections
void setup() {
  pinMode(2, OUTPUT);  // STEP
  pinMode(4, OUTPUT);  // DIR
  pinMode(5, OUTPUT);  // EN
  pinMode(21, INPUT_PULLUP);  // HALL
  
  digitalWrite(5, LOW);  // Enable motor
  Serial.begin(115200);
}

void loop() {
  Serial.print("Hall Sensor: ");
  Serial.println(digitalRead(21) ? "HIGH" : "LOW");
  
  // Test step pulse
  digitalWrite(2, HIGH);
  delayMicroseconds(10);
  digitalWrite(2, LOW);
  delayMicroseconds(1000);
}
```

### 2. Hall Sensor Test
- **Power on** system
- **Observe** serial monitor - should show "HIGH"
- **Move magnet near** sensor
- **Verify** output changes to "LOW"
- **Remove magnet** - should return to "HIGH"

### 3. Motor Movement Test
- **Enable** motor (EN pin LOW)
- **Set direction** (DIR pin HIGH or LOW)
- **Send step pulses** to STEP pin
- **Verify** motor rotates in expected direction
- **Test both directions**
- **Check** for smooth, quiet operation

### 4. Complete System Test
- **Upload** MinimaSlidrAutomat firmware
- **Power on** and observe serial monitor
- **Verify** dual sensor homing sequence finds both Home A and Home B sensors
- **Check** automatic travel distance measurement and ping-pong range calculation
- **Observe** ping-pong motion starting from Home B end within calculated range
- **Confirm** smooth acceleration/deceleration during ping-pong motion

## Troubleshooting Common Issues

### Motor Not Moving
- **Check** 12V power supply connection
- **Verify** TMC2208 EN pin is LOW (enabled)
- **Test** STEP/DIR pin connections
- **Adjust** TMC2208 current setting (VREF)
- **Check** motor coil connections

### Hall Sensor Not Working
- **Verify** 3.3V power and GND connections
- **Test** with multimeter - should show 3.3V normally, 0V when triggered
- **Check** magnet position and strength
- **Confirm** ESP32 pin 21 connection
- **Test** sensor with separate magnet

### Erratic Movement
- **Check** power supply stability under load
- **Verify** all connections are secure
- **Adjust** TMC2208 current setting
- **Check** for electromagnetic interference
- **Verify** belt tension and alignment

### Overrunning Travel
- **Verify** steps-per-mm calibration (46.24)
- **Check** acceleration distance compensation
- **Confirm** physical end stop positions
- **Test** with reduced travel distance first
- **Check** belt slippage or mechanical binding

## Mechanical Assembly Notes

### Rail System
- **Linear rails** with precision bearings
- **Smooth operation** throughout travel range
- **Minimal backlash** in belt drive system
- **Rigid mounting** to prevent flex

### Camera Platform
- **Secure mounting** for camera equipment
- **Weight distribution** balanced on carriage
- **Vibration isolation** if needed
- **Cable management** for camera cables

### Belt Drive System
- **Proper tension** - not too loose or tight
- **Alignment** of pulleys and belt path
- **Quality components** for smooth operation
- **Protection** from contamination

This hardware setup provides a solid foundation for the MinimaSlidrAutomat autonomous camera slider system. Follow all safety precautions and verify each connection before powering up the system.