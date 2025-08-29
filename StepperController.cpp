/*
 * ESP32 MinimaSlidrAutomat - Stepper Motor Controller Implementation
 * v2.0 - Production Release
 * 
 * Project by Aaron Young
 * Developed with assistance from Anthropic's Claude Code
 * Repository: https://github.com/RhinocerosPants/ESP32_MinimaSlidrAutomat
 */

#include "StepperController.h"
#include <Arduino.h>

StepperController::StepperController() 
  : stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN),
    currentMode(MOTION_CONSTANT_SPEED),
    lastDebugPrint(0) {
}

// ========== INITIALIZATION ==========

bool StepperController::initialize() {
  // Configure stepper with initial parameters
  stepper.setMaxSpeed(HOMING_SPEED);  // Start in constant speed mode
  stepper.setAcceleration(ACCELERATION);
  stepper.setCurrentPosition(0);
  
  // Configure motor control pins
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  enableMotor();
  
  Serial.println("StepperController: Initialized");
  return true;
}

void StepperController::enableMotor() {
  digitalWrite(MOTOR_ENABLE_PIN, LOW);  // TMC2208 enable is active low
}

void StepperController::disableMotor() {
  digitalWrite(MOTOR_ENABLE_PIN, HIGH); // Disable motor (high impedance)
}

// ========== POSITION MANAGEMENT ==========

float StepperController::getCurrentPositionMM() {
  return stepper.currentPosition() / STEPS_PER_MM;
}

void StepperController::setCurrentPositionMM(float positionMM) {
  stepper.setCurrentPosition(positionMM * STEPS_PER_MM);
}

float StepperController::getTargetPositionMM() {
  return stepper.targetPosition() / STEPS_PER_MM;
}

float StepperController::getDistanceToGoMM() {
  return stepper.distanceToGo() / STEPS_PER_MM;
}

// ========== CONSTANT SPEED MOTION (Homing) ==========

void StepperController::startConstantSpeedLeft() {
  switchToConstantSpeed();
  stepper.setSpeed(-HOMING_SPEED);  // Negative = left/CCW
  DEBUG_PRINTF("StepperController: Starting constant speed LEFT at %.1fmm/s\n", 
                HOMING_SPEED / STEPS_PER_MM);
}

void StepperController::startConstantSpeedRight() {
  switchToConstantSpeed();
  stepper.setSpeed(HOMING_SPEED);   // Positive = right/CW
  DEBUG_PRINTF("StepperController: Starting constant speed RIGHT at %.1fmm/s\n", 
                HOMING_SPEED / STEPS_PER_MM);
}

void StepperController::runConstantSpeed() {
  if (currentMode == MOTION_CONSTANT_SPEED) {
    stepper.runSpeed();
  }
}

void StepperController::stopConstantSpeed() {
  stepper.setSpeed(0);
  // Brief delay to ensure motor physically stops
  delay(100);
  DEBUG_PRINTLN("StepperController: Constant speed stopped");
}

// ========== MANUAL SPEED MOTION (3x faster than ping-pong) ==========

void StepperController::startManualSpeedLeft() {
  // Smooth transition to manual speed without delays
  if (currentMode != MOTION_CONSTANT_SPEED) {
    stepper.setMaxSpeed(MANUAL_SPEED);  // Set max speed for constant speed mode
    currentMode = MOTION_CONSTANT_SPEED;
    DEBUG_PRINTLN("StepperController: Switched to MANUAL SPEED mode");
  }
  stepper.setSpeed(-MANUAL_SPEED);  // Negative = left/CCW
  DEBUG_PRINTF("StepperController: Starting manual speed LEFT at %.1fmm/s\n", 
                MANUAL_SPEED / STEPS_PER_MM);
}

void StepperController::startManualSpeedRight() {
  // Smooth transition to manual speed without delays
  if (currentMode != MOTION_CONSTANT_SPEED) {
    stepper.setMaxSpeed(MANUAL_SPEED);  // Set max speed for constant speed mode
    currentMode = MOTION_CONSTANT_SPEED;
    DEBUG_PRINTLN("StepperController: Switched to MANUAL SPEED mode");
  }
  stepper.setSpeed(MANUAL_SPEED);   // Positive = right/CW
  DEBUG_PRINTF("StepperController: Starting manual speed RIGHT at %.1fmm/s\n", 
                MANUAL_SPEED / STEPS_PER_MM);
}

// ========== ACCELERATED MOTION (Ping-pong) ==========

void StepperController::configureForPingPong() {
  switchToAccelerated();
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);
  DEBUG_PRINTF("StepperController: Configured for ping-pong (%.1fmm/s max, %.1fmm/s²)\n",
                MAX_SPEED / STEPS_PER_MM, ACCELERATION / STEPS_PER_MM);
}

void StepperController::configureForManual() {
  // Smooth transition without delays or stops
  currentMode = MOTION_ACCELERATED;
  stepper.setMaxSpeed(MANUAL_SPEED);
  stepper.setAcceleration(ACCELERATION * 2);  // 2x acceleration for snappier manual moves
  DEBUG_PRINTF("StepperController: Configured for manual moves (%.1fmm/s max, %.1fmm/s²)\n",
                MANUAL_SPEED / STEPS_PER_MM, (ACCELERATION * 2) / STEPS_PER_MM);
}

void StepperController::moveToPositionMM(float targetMM) {
  if (currentMode == MOTION_ACCELERATED) {
    long targetSteps = targetMM * STEPS_PER_MM;
    stepper.moveTo(targetSteps);
    DEBUG_PRINTF("StepperController: Moving to %.1fmm (accelerated)\n", targetMM);
  }
}

void StepperController::runAccelerated() {
  if (currentMode == MOTION_ACCELERATED) {
    stepper.run();
  }
}

bool StepperController::isMoving() {
  return stepper.isRunning();
}

// ========== EMERGENCY OPERATIONS ==========

void StepperController::emergencyStop() {
  stepper.setSpeed(0);      // Stop constant speed immediately
  stepper.stop();           // Stop accelerated movement immediately
  Serial.println("StepperController: EMERGENCY STOP");
}

void StepperController::holdPosition() {
  emergencyStop();
  // Motor stays enabled to maintain holding torque (critical for inclines)
  Serial.println("StepperController: Holding position for safety");
}

void StepperController::stopAndHold() {
  stepper.setSpeed(0);      // Stop constant speed movement
  stepper.stop();           // Stop accelerated movement with deceleration
  // Motor stays enabled to maintain holding torque
  DEBUG_PRINTLN("StepperController: Stopped and holding position");
}

// ========== MODE MANAGEMENT ==========

MotionMode StepperController::getCurrentMode() const {
  return currentMode;
}

void StepperController::switchToConstantSpeed() {
  if (currentMode != MOTION_CONSTANT_SPEED) {
    stepper.stop();           // Stop any accelerated movement
    stepper.setSpeed(0);      // Clear any constant speed
    delay(50);                // Brief settling time
    
    stepper.setMaxSpeed(HOMING_SPEED);  // Configure for constant speed
    currentMode = MOTION_CONSTANT_SPEED;
    DEBUG_PRINTLN("StepperController: Switched to CONSTANT SPEED mode");
  }
}

void StepperController::switchToAccelerated() {
  if (currentMode != MOTION_ACCELERATED) {
    stepper.setSpeed(0);      // Clear any constant speed
    stepper.stop();           // Ensure stopped
    delay(50);                // Brief settling time
    
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(ACCELERATION);
    currentMode = MOTION_ACCELERATED;
    DEBUG_PRINTLN("StepperController: Switched to ACCELERATED mode");
  }
}

// ========== RAW STEP OPERATIONS ==========

void StepperController::executeRawSteps(int steps, bool direction, int pulseWidthUs, int intervalUs) {
  // Set direction pin directly
  digitalWrite(MOTOR_DIR_PIN, direction ? HIGH : LOW);
  
  DEBUG_PRINTF("StepperController: Executing %d raw steps %s\n", 
                steps, direction ? "RIGHT" : "LEFT");
  
  // Execute steps with precise timing
  for (int i = 0; i < steps; i++) {
    digitalWrite(MOTOR_STEP_PIN, HIGH);
    delayMicroseconds(pulseWidthUs);
    digitalWrite(MOTOR_STEP_PIN, LOW);
    delayMicroseconds(intervalUs);
  }
  
  // Update stepper library position tracking
  long positionChange = direction ? steps : -steps;
  stepper.setCurrentPosition(stepper.currentPosition() + positionChange);
}

// ========== DEBUG AND MONITORING ==========

void StepperController::printStatus() {
  // Disable stepper status printing to avoid timing interruptions
  return;
  
  // Throttle debug output
  if (millis() - lastDebugPrint < DEBUG_PRINT_INTERVAL_MS) {
    return;
  }
  
  Serial.printf("Stepper: Pos=%.1fmm, Target=%.1fmm, Speed=%.1fmm/s, Mode=%s, Running=%s\n",
                getCurrentPositionMM(),
                getTargetPositionMM(),
                getCurrentSpeedMmPerSec(),
                (currentMode == MOTION_CONSTANT_SPEED) ? "CONST" : "ACCEL",
                isMoving() ? "YES" : "NO");
                
  lastDebugPrint = millis();
}

float StepperController::getCurrentSpeedStepsPerSec() {
  return stepper.speed();
}

float StepperController::getCurrentSpeedMmPerSec() {
  return stepper.speed() / STEPS_PER_MM;
}