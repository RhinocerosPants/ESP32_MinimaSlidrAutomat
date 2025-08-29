/*
 * ESP32 MinimaSlidrAutomat - Configuration Header
 * v2.0 - Production Release
 * 
 * Project by Aaron Young
 * Developed with assistance from Anthropic's Claude Code
 * Repository: https://github.com/RhinocerosPants/ESP32_MinimaSlidrAutomat
 * 
 * Centralized configuration for all system constants and parameters
 */

#pragma once

// ========== HARDWARE PIN DEFINITIONS ==========
#define MOTOR_STEP_PIN               2        // TMC2208 STEP pin
#define MOTOR_DIR_PIN                4        // TMC2208 DIR pin  
#define MOTOR_ENABLE_PIN             5        // TMC2208 EN pin (active low)
#define HALL_SENSOR_A_PIN            21       // Home A sensor (left end)
#define HALL_SENSOR_B_PIN            22       // Home B sensor (right end)
#define LED_DATA_PIN                 23       // WS2812B LED strip data pin
#define BUTTON_MOVE_TO_A_PIN         25       // Move to Home A button
#define BUTTON_MOVE_TO_B_PIN         26       // Move to Home B button
#define BUTTON_RETURN_PING_PONG_PIN  27       // Return to ping-pong button

// ========== MOTION PARAMETERS - CALIBRATED VALUES ==========
#define STEPS_PER_MM                 46.24f   // Steps per millimeter (laser calibrated)
#define MAX_SPEED                    400      // Steps/sec for ping-pong (8.7mm/s smooth filming)
#define ACCELERATION                 200      // Steps/sec² for ping-pong (4-second accel/decel curves)
#define HOMING_SPEED                 2400     // Steps/sec for all pre-ping-pong movements (100mm/s)
#define MANUAL_SPEED                 1600     // Steps/sec for manual moves (4x faster than ping-pong: 34.6mm/s)

// ========== SAFETY AND TOLERANCE CONSTANTS ==========
#define SENSOR_CLEARANCE_MM          20.0f    // Distance from sensors for ping-pong range
#define POSITION_TOLERANCE_MM        1.0f     // Tolerance for reaching target positions
#define CRASH_DETECTION_TOLERANCE_MM 10.0f    // Extra tolerance beyond ping-pong range for crash detection
#define MINIMUM_TRAVEL_RANGE_MM      100.0f   // Minimum required travel range for operation
#define BOUNDS_SAFETY_MARGIN_MM      50.0f    // Safety margin beyond measured travel for bounds checking

// ========== TIMING CONSTANTS ==========
#define HOMING_TIMEOUT_MS            40000    // 40-second timeout per sensor (1000mm travel limit)
#define START_PAUSE_MS               3000     // 3-second pause before ping-pong begins
#define STATUS_UPDATE_INTERVAL_MS    2000     // Status reporting interval
#define DEBUG_PRINT_INTERVAL_MS      500      // Debug message interval during movement
#define BUTTON_DEBOUNCE_MS           50       // Button debounce time

// ========== SENSOR CLEARING PARAMETERS ==========
#define SENSOR_CLEAR_STEPS           2000     // Steps to move away from active sensor (~43mm)
#define STEP_PULSE_WIDTH_US          200      // Microseconds for step pulse duration
#define STEP_INTERVAL_US             200      // Microseconds between steps (2500 steps/sec clearing speed)

// ========== LED CONFIGURATION ==========
#define NUM_LEDS                     5        // Number of LEDs in WS2812B strip
#define LED_BRIGHTNESS               64       // Default brightness (25% of 255)

// ========== SYSTEM STATES ==========
enum SliderState {
  STATE_BOOT,                                // System initialization
  STATE_HOMING_A,                           // Finding Home A sensor (left end)
  STATE_HOMING_B,                           // Finding Home B sensor (right end) 
  STATE_MEASURING_TRAVEL,                   // Calculating travel distance and ping-pong range
  STATE_MOVING_TO_START,                    // Moving to calculated start position
  STATE_PAUSING_AT_START,                   // 3-second pause before ping-pong
  STATE_PING_PONG,                          // Normal ping-pong operation
  STATE_MANUAL_MOVE_TO_A,                   // Manual move to Home A position
  STATE_MANUAL_MOVE_TO_B,                   // Manual move to Home B position
  STATE_MANUAL_WAIT_A,                      // Wait at Home A position
  STATE_MANUAL_WAIT_B,                      // Wait at Home B position
  STATE_ERROR                               // Error state - hold position
};

// ========== MOTION MODES ==========
enum MotionMode {
  MOTION_CONSTANT_SPEED,                    // Constant speed using setSpeed()/runSpeed()
  MOTION_ACCELERATED                        // Accelerated motion using moveTo()/run()
};

// ========== LED STATES ==========
enum LEDState {
  LED_HOMING,                               // Breathing amber during homing
  LED_PING_PONG,                            // Green ping-pong animation during normal operation
  LED_MANUAL,                               // Purple breathing during manual moves
  LED_ERROR                                 // Breathing red during error states
};

// ========== CALCULATED VALUES (Set at runtime) ==========
// These are calculated automatically during dual sensor homing
extern float measuredTravelMM;              // Actual distance between sensors
extern float pingPongStartMM;               // Calculated start position (SENSOR_CLEARANCE_MM)
extern float pingPongEndMM;                 // Calculated end position (measuredTravelMM - SENSOR_CLEARANCE_MM)

// ========== DEBUG CONFIGURATION ==========
// Uncomment the following line to enable debug output
// #define DEBUG_MODE

#ifdef DEBUG_MODE
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x) 
  #define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(fmt, ...)
#endif

// ========== VERSION INFORMATION ==========
#define FIRMWARE_VERSION "v2.0"
#define FIRMWARE_DESCRIPTION "Production Release"