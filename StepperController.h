/**
 * @file StepperController.h
 * @brief Stepper Motor Controller for ESP32 MinimaSlidrAutomat
 * @version 2.0
 * @date 2024
 * @author Aaron Young
 * @note Developed with assistance from Anthropic's Claude Code
 * @repository https://github.com/RhinocerosPants/ESP32_MinimaSlidrAutomat
 * 
 * Handles all stepper motor operations with clean separation between
 * constant speed (homing) and accelerated (ping-pong) motion modes.
 * 
 * This class provides a clean interface for controlling a TMC2208 stepper
 * driver with support for both constant speed movement (used during homing)
 * and accelerated movement (used for smooth ping-pong motion).
 * 
 * @warning Motor will be enabled after successful initialization
 * @see SensorManager, SafetyMonitor
 */

#pragma once

#include <AccelStepper.h>
#include "config.h"

/**
 * @class StepperController
 * @brief Controls stepper motor with dual motion modes
 * 
 * This class handles all stepper motor operations including constant speed
 * movement for homing and accelerated movement for smooth ping-pong motion.
 * Provides clean separation between motion modes and comprehensive position
 * tracking in millimeters.
 */
class StepperController {
private:
  AccelStepper stepper;
  MotionMode currentMode;
  unsigned long lastDebugPrint;
  
public:
  StepperController();
  
  // ========== INITIALIZATION ==========
  
  /**
   * @brief Initialize the stepper motor system
   * 
   * Configures pins, sets initial parameters, and enables the motor driver.
   * Must be called before any other stepper operations.
   * 
   * @return true if initialization successful
   * @return false if initialization failed
   * 
   * @warning Motor will be enabled after successful initialization
   * @note Sets initial mode to constant speed for homing operations
   */
  bool initialize();
  
  /**
   * @brief Enable the stepper motor driver
   * 
   * Activates the TMC2208 driver by setting the EN pin LOW.
   * Motor will have holding torque after this call.
   * 
   * @note TMC2208 enable pin is active LOW
   */
  void enableMotor();
  
  /**
   * @brief Disable the stepper motor driver
   * 
   * Deactivates the TMC2208 driver by setting the EN pin HIGH.
   * Motor will lose holding torque and be in high impedance state.
   * 
   * @warning Use carefully - motor may move on inclined setups
   */
  void disableMotor();
  
  // ========== POSITION MANAGEMENT ==========
  
  /**
   * @brief Get current position in millimeters
   * 
   * @return Current stepper position converted to millimeters using STEPS_PER_MM
   * @retval 0.0 Position at Home A sensor (reference point)
   * @retval >0.0 Position towards Home B sensor
   * @retval <0.0 Position beyond Home A sensor (error condition)
   */
  float getCurrentPositionMM();
  
  /**
   * @brief Set current position reference point
   * 
   * Updates the stepper library's internal position counter.
   * Typically called when Home A sensor is detected to establish
   * the 0mm reference point.
   * 
   * @param positionMM New position value in millimeters
   * @note This does not move the motor, only updates position tracking
   */
  void setCurrentPositionMM(float positionMM);
  
  /**
   * @brief Get target position for accelerated movement
   * 
   * @return Target position in millimeters for accelerated mode
   * @note Only meaningful when in MOTION_ACCELERATED mode
   */
  float getTargetPositionMM();
  
  /**
   * @brief Get remaining distance to target
   * 
   * @return Distance to target in millimeters (can be negative)
   * @retval 0.0 At target position
   * @retval >0.0 Target is to the right
   * @retval <0.0 Target is to the left
   */
  float getDistanceToGoMM();
  
  // ========== CONSTANT SPEED MOTION (Homing) ==========
  
  /**
   * @brief Start constant speed movement towards the left
   * 
   * Configures the stepper for constant speed movement in the negative
   * direction at HOMING_SPEED. Used during Home A sensor homing.
   * 
   * @note This switches the motor to constant speed mode automatically
   * @see startConstantSpeedRight(), runConstantSpeed()
   */
  void startConstantSpeedLeft();
  
  /**
   * @brief Start constant speed movement towards the right
   * 
   * Configures the stepper for constant speed movement in the positive
   * direction at HOMING_SPEED. Used during Home B sensor homing.
   * 
   * @note This switches the motor to constant speed mode automatically
   * @see startConstantSpeedLeft(), runConstantSpeed()
   */
  void startConstantSpeedRight();
  
  /**
   * @brief Execute one step of constant speed movement
   * 
   * Must be called repeatedly in the main loop to maintain constant
   * speed movement. Only effective when in MOTION_CONSTANT_SPEED mode.
   * 
   * @note Call this in every loop iteration during homing
   * @see startConstantSpeedLeft(), startConstantSpeedRight()
   */
  void runConstantSpeed();
  
  /**
   * @brief Stop constant speed movement immediately
   * 
   * Sets speed to zero and waits briefly for motor to physically stop.
   * Used when sensors are triggered during homing.
   * 
   * @note Includes brief delay to ensure physical motor stop
   */
  void stopConstantSpeed();
  
  // ========== MANUAL SPEED MOTION (3x faster than ping-pong) ==========
  
  /**
   * @brief Start manual speed movement towards the left
   * 
   * Configures the stepper for constant speed movement in the negative
   * direction at MANUAL_SPEED (3x faster than ping-pong).
   * 
   * @note This switches the motor to constant speed mode automatically
   */
  void startManualSpeedLeft();
  
  /**
   * @brief Start manual speed movement towards the right
   * 
   * Configures the stepper for constant speed movement in the positive
   * direction at MANUAL_SPEED (3x faster than ping-pong).
   * 
   * @note This switches the motor to constant speed mode automatically
   */
  void startManualSpeedRight();
  
  // ========== ACCELERATED MOTION (Ping-pong) ==========
  void configureForPingPong();     // Set up acceleration parameters for smooth filming
  void configureForManual();       // Set up acceleration parameters for manual moves
  void moveToPositionMM(float targetMM);  // Set target for accelerated movement
  void runAccelerated();           // Execute accelerated movement
  bool isMoving();           // Check if motor is running
  
  // ========== EMERGENCY OPERATIONS ==========
  void emergencyStop();            // Immediate stop with no deceleration
  void holdPosition();             // Maintain holding torque (safety for inclines)
  void stopAndHold();              // Normal stop and hold position (not emergency)
  
  // ========== MODE MANAGEMENT ==========
  MotionMode getCurrentMode() const;
  void switchToConstantSpeed();    // Prepare for constant speed operations
  void switchToAccelerated();      // Prepare for accelerated operations
  
  // ========== RAW STEP OPERATIONS ==========
  // For power-on sensor clearing only
  void executeRawSteps(int steps, bool direction, int pulseWidthUs, int intervalUs);
  
  // ========== DEBUG AND MONITORING ==========
  void printStatus();
  float getCurrentSpeedStepsPerSec();
  float getCurrentSpeedMmPerSec();
};