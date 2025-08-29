/*
 * ESP32 MinimaSlidrAutomat - Self-Calibrating Camera Slider
 * v2.0 - Production Release
 * 
 * Project by Aaron Young
 * Developed with assistance from Anthropic's Claude Code
 * Repository: https://github.com/RhinocerosPants/ESP32_MinimaSlidrAutomat
 * 
 * Description:
 * Fully automatic dual sensor ping-pong camera slider with auto-calibration.
 * Features clean separation of concerns architecture for maintainability,
 * manual control buttons, and comprehensive safety systems.
 * 
 * Hardware Requirements:
 * - ESP32 DevKit
 * - TMC2208 stepper driver (basic step/dir mode)
 * - Hall effect sensor A on pin 21 (Home A - left end)
 * - Hall effect sensor B on pin 22 (Home B - right end)  
 * - WS2812B LED strip (5 LEDs) on pin 23
 * - Manual control buttons on pins 25, 26, 27
 * - Stepper motor (NEMA 17 recommended)
 * 
 * Operation:
 * 1. Power on -> Auto home to sensor A, then find sensor B (breathing amber LEDs)
 * 2. Measure actual travel distance automatically
 * 3. Move to start position with clearance from sensors
 * 4. Begin continuous ping-pong between measured endpoints (green ping-pong LEDs)
 * 5. Manual control available via buttons for positioning
 * 
 * LED Indicators:
 * - Homing: Breathing amber animation
 * - Ping-pong: Green ping-pong animation following motion
 * - Manual: Purple breathing animation
 * - Error: Breathing red animation
 * 
 * Safety Features:
 * - Motor holds position on crash/error to prevent sliding on inclines
 * - Comprehensive bounds checking and crash detection
 * - Timeout protection during homing operations
 */

#include "config.h"
#include "StepperController.h" 
#include "SensorManager.h"
#include "SliderStateMachine.h"
#include "SafetyMonitor.h"
#include "led_indicators.h"
#include "ButtonManager.h"

// ========== SYSTEM COMPONENTS ==========
StepperController stepper;
SensorManager sensors;
SliderStateMachine stateMachine;
SafetyMonitor safety;
LEDIndicators leds;
ButtonManager buttons;

void setup() {
  Serial.begin(115200);
  Serial.println("=== ESP32 MinimaSlidrAutomat v2.0 - Production Release ===");
  Serial.println("By Aaron Young");
  Serial.println("Self-calibrating dual sensor camera slider");
  Serial.printf("Homing Speed: %d steps/sec (%.1fmm/s)\n", HOMING_SPEED, HOMING_SPEED / STEPS_PER_MM);
  Serial.println("Starting initialization...");
  
  // Initialize all system components
  if (!initializeSystem()) {
    Serial.println("FATAL ERROR: System initialization failed");
    while (true) {
      delay(1000);  // Halt system
    }
  }
  
  Serial.println("System initialized successfully");
  Serial.println("Starting dual sensor homing and travel measurement...");
  
  // Handle power-on sensor clearing if needed
  handlePowerOnSensorClearing();
  
  // Begin autonomous operation
  stateMachine.startHomingSequence();
  leds.setState(LED_HOMING);
}

void loop() {
  // Check for button presses (only when system is operational)
  checkButtonPresses();
  
  // Update all system components
  updateSystemComponents();
  
  // Execute current state
  executeCurrentState();
  
  // Perform safety monitoring
  performSafetyChecks();
  
  // Update status displays
  updateStatusDisplays();
}

// ========== SYSTEM INITIALIZATION ==========

bool initializeSystem() {
  bool success = true;
  
  Serial.println("Initializing system components...");
  
  success &= sensors.initialize();
  success &= stepper.initialize(); 
  success &= stateMachine.initialize();
  success &= safety.initialize();
  success &= leds.initialize();
  success &= buttons.initialize();
  
  if (!success) {
    Serial.println("ERROR: One or more components failed to initialize");
    return false;
  }
  
  Serial.println("All components initialized successfully");
  return true;
}

void handlePowerOnSensorClearing() {
  SensorManager::SensorClearingResult clearResult = sensors.checkAndClearSensorsAtPowerOn();
  
  if (clearResult.clearingPerformed) {
    Serial.println("Power-on sensor clearing completed");
    
    // Update stepper position tracking after manual clearing
    int totalSteps = clearResult.stepsExecutedA - clearResult.stepsExecutedB;
    stepper.setCurrentPositionMM(totalSteps / STEPS_PER_MM);
    
    Serial.printf("Position adjusted for sensor clearing: %.1fmm\n", 
                  stepper.getCurrentPositionMM());
  }
}

// ========== SYSTEM UPDATES ==========

void updateSystemComponents() {
  leds.update();
  
  // Skip safety monitoring updates during manual moves to avoid interruptions
  SliderState currentState = stateMachine.getCurrentState();
  if (currentState != STATE_MANUAL_MOVE_TO_A && currentState != STATE_MANUAL_MOVE_TO_B) {
    safety.updateSafetyMonitoring();
  }
}

void executeCurrentState() {
  switch (stateMachine.getCurrentState()) {
    case STATE_BOOT:
      // Boot state handled in setup()
      break;
      
    case STATE_HOMING_A:
      executeHomingA();
      break;
      
    case STATE_HOMING_B:
      executeHomingB();
      break;
      
    case STATE_MEASURING_TRAVEL:
      executeTravelMeasurement();
      break;
      
    case STATE_MOVING_TO_START:
      executeMoveToStart();
      break;
      
    case STATE_PAUSING_AT_START:
      executePauseAtStart();
      break;
      
    case STATE_PING_PONG:
      executePingPong();
      break;
      
    case STATE_MANUAL_MOVE_TO_A:
      executeManualMoveToA();
      break;
      
    case STATE_MANUAL_MOVE_TO_B:
      executeManualMoveToB();
      break;
      
    case STATE_MANUAL_WAIT_A:
      executeManualWaitA();
      break;
      
    case STATE_MANUAL_WAIT_B:
      executeManualWaitB();
      break;
      
    case STATE_ERROR:
      executeErrorState();
      break;
  }
}

// ========== STATE EXECUTION FUNCTIONS ==========

void executeHomingA() {
  // Check for timeout
  if (stateMachine.hasHomingTimedOut()) {
    safety.performComprehensiveSafetyCheck(stepper.getCurrentPositionMM(), 0, 0, false, false);
    return;  // Error state entered by safety monitor
  }
  
  // Check if sensor triggered
  if (sensors.hasSensorATriggered()) {
    Serial.println("Home A sensor detected - completing homing");
    stepper.stopConstantSpeed();
    stepper.setCurrentPositionMM(0.0f);  // Establish reference point
    stateMachine.completeHomingA();
    stepper.startConstantSpeedRight();   // Begin moving to Home B
    return;
  }
  
  // Continue constant speed movement
  if (!stepper.isMoving()) {
    stepper.startConstantSpeedLeft();
  }
  stepper.runConstantSpeed();
}

void executeHomingB() {
  // Check for timeout
  if (stateMachine.hasHomingTimedOut()) {
    safety.performComprehensiveSafetyCheck(stepper.getCurrentPositionMM(), 0, 0, false, false);
    return;  // Error state entered by safety monitor
  }
  
  // Check if sensor triggered
  if (sensors.hasSensorBTriggered()) {
    Serial.println("Home B sensor detected - completing homing");
    stepper.stopConstantSpeed();
    
    // Record measured travel distance
    measuredTravelMM = stepper.getCurrentPositionMM();
    Serial.printf("Measured travel distance: %.1fmm\n", measuredTravelMM);
    
    stateMachine.completeHomingB();
    return;
  }
  
  // Continue constant speed movement
  stepper.runConstantSpeed();
}

void executeTravelMeasurement() {
  // Validate travel range
  SafetyMonitor::TravelValidationResult validation = safety.validateTravelRange(measuredTravelMM);
  
  if (!validation.isValid) {
    Serial.printf("ERROR: %s (%.1fmm measured, %.1fmm minimum required)\n",
                  validation.errorReason, validation.requestedTravelMM, validation.minimumRequiredMM);
    stateMachine.enterErrorState("Insufficient travel range");
    leds.setState(LED_ERROR);
    return;
  }
  
  // Calculate ping-pong range
  pingPongStartMM = SENSOR_CLEARANCE_MM;
  pingPongEndMM = measuredTravelMM - SENSOR_CLEARANCE_MM;
  
  Serial.printf("Ping-pong range: %.1fmm to %.1fmm (%.1fmm available)\n",
                pingPongStartMM, pingPongEndMM, pingPongEndMM - pingPongStartMM);
  
  // Mark system as fully homed for safety monitoring
  safety.setHomingComplete(measuredTravelMM);
  
  // Transition to positioning
  stateMachine.markTravelMeasured();
}

void executeMoveToStart() {
  // Calculate movement needed
  float currentPos = stepper.getCurrentPositionMM();
  float targetPos = pingPongEndMM;  // Start ping-pong from Home B end
  float distance = abs(currentPos - targetPos);
  
  // Check if already at target
  if (distance < POSITION_TOLERANCE_MM) {
    Serial.println("Already at start position");
    stateMachine.startPause();
    return;
  }
  
  // Start constant speed movement if not already moving or speed is zero
  if (!stepper.isMoving() || abs(stepper.getCurrentSpeedMmPerSec()) < 0.1) {
    if (currentPos > targetPos) {
      stepper.startConstantSpeedLeft();   // Move towards Home A
    } else {
      stepper.startConstantSpeedRight();  // Move towards Home B
    }
  }
  
  // Execute movement
  stepper.runConstantSpeed();
  
  // Check if reached target
  if (distance < POSITION_TOLERANCE_MM) {
    stepper.stopConstantSpeed();
    stateMachine.startPause();
  }
}

void executePauseAtStart() {
  if (stateMachine.isPauseComplete()) {
    Serial.println("Pause complete - beginning ping-pong motion");
    stepper.configureForPingPong();  // Switch to acceleration mode
    stepper.moveToPositionMM(pingPongStartMM);  // First target: Home A side
    stateMachine.startPingPong();
    leds.setState(LED_PING_PONG);
  }
}

void executePingPong() {
  stepper.runAccelerated();
  
  if (!stepper.isMoving()) {
    // Reached end of travel - reverse direction
    stateMachine.togglePingPongDirection();
    
    float nextTarget = stateMachine.isMovingTowardsEnd() ? pingPongEndMM : pingPongStartMM;
    stepper.moveToPositionMM(nextTarget);
    
    Serial.printf("Ping-pong: Moving to %.1fmm\n", nextTarget);
  }
}

void executeErrorState() {
  // Hold position and maintain error status
  stepper.holdPosition();
  leds.setState(LED_ERROR);
  
  // Periodic error message
  static unsigned long lastErrorMsg = 0;
  if (millis() - lastErrorMsg > 5000) {
    Serial.println("SYSTEM ERROR - Motor holding position - Power cycle required to restart");
    lastErrorMsg = millis();
  }
}

// ========== SAFETY MONITORING ==========

void performSafetyChecks() {
  if (!safety.isSystemFullyHomed()) {
    return;  // Limited safety monitoring until homing complete
  }
  
  // Skip intensive safety monitoring during manual moves to avoid interruptions
  SliderState currentState = stateMachine.getCurrentState();
  if (currentState == STATE_MANUAL_MOVE_TO_A || currentState == STATE_MANUAL_MOVE_TO_B) {
    return;  // Manual moves go to safe positions, no need for safety monitoring
  }
  
  // Get current system state
  float currentPos = stepper.getCurrentPositionMM();
  bool sensorA = sensors.isSensorAActive();
  bool sensorB = sensors.isSensorBActive();
  
  // Perform comprehensive safety check
  SafetyMonitor::EmergencyCheckResult emergencyCheck = 
    safety.performComprehensiveSafetyCheck(currentPos, pingPongStartMM, pingPongEndMM, sensorA, sensorB);
  
  if (emergencyCheck.emergencyDetected) {
    Serial.printf("EMERGENCY STOP: %s\n", emergencyCheck.emergencyReason);
    stepper.emergencyStop();
    stateMachine.enterErrorState(emergencyCheck.emergencyReason);
    leds.setState(LED_ERROR);
  }
  
  // Check for state-specific timeouts
  stateMachine.checkTimeouts();
}

// ========== STATUS UPDATES ==========

void updateStatusDisplays() {
  // Skip status updates during manual moves to avoid interrupting stepper timing
  SliderState currentState = stateMachine.getCurrentState();
  if (currentState == STATE_MANUAL_MOVE_TO_A || currentState == STATE_MANUAL_MOVE_TO_B) {
    return;  // No status updates during manual moves
  }
  
  // Throttled status updates
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > STATUS_UPDATE_INTERVAL_MS) {
    printSystemStatus();
    lastUpdate = millis();
  }
}

void printSystemStatus() {
  float currentPos = stepper.getCurrentPositionMM();
  
  // Print state and position
  Serial.printf("State: %s | Pos: %.1fmm | Target: %.1fmm | %s | Motor: %s\n",
                stateMachine.getStateString().c_str(),
                currentPos,
                stepper.getTargetPositionMM(),
                sensors.getSensorStatusString().c_str(),
                stepper.isMoving() ? "RUNNING" : "stopped");
  
  // Print travel info if available
  if (safety.isSystemFullyHomed()) {
    Serial.printf("Travel: %.1fmm | Range: %.1f-%.1fmm\n",
                  measuredTravelMM, pingPongStartMM, pingPongEndMM);
  }
}

// ========== BUTTON HANDLING ==========

void checkButtonPresses() {
  // Only allow button presses when system is fully operational
  if (!safety.isSystemFullyHomed() || stateMachine.isInErrorState()) {
    return;
  }
  
  // Skip button checking during manual moves to avoid interruptions
  SliderState currentState = stateMachine.getCurrentState();
  if (currentState == STATE_MANUAL_MOVE_TO_A || currentState == STATE_MANUAL_MOVE_TO_B) {
    return;  // Don't process buttons while moving
  }
  
  // Check Move to A button
  if (buttons.isButtonAPressed()) {
    Serial.println("Move to A button pressed - transitioning to manual control");
    // No need to stop here - let manual move function handle the transition
    stateMachine.transitionTo(STATE_MANUAL_MOVE_TO_A);
    leds.setState(LED_MANUAL);    // Use purple LED pattern for manual moves
    leds.setManualMovement(true, false);  // Moving towards A (left)
  }
  
  // Check Move to B button  
  if (buttons.isButtonBPressed()) {
    Serial.println("Move to B button pressed - transitioning to manual control");
    // No need to stop here - let manual move function handle the transition
    stateMachine.transitionTo(STATE_MANUAL_MOVE_TO_B);
    leds.setState(LED_MANUAL);    // Use purple LED pattern for manual moves
    leds.setManualMovement(true, true);   // Moving towards B (right)
  }
  
  // Check Return to Ping-pong button (only when in manual wait states)
  if (buttons.isButtonPingPongPressed()) {
    if (currentState == STATE_MANUAL_WAIT_A || currentState == STATE_MANUAL_WAIT_B) {
      Serial.println("Return to ping-pong button pressed - resuming ping-pong motion");
      stepper.configureForPingPong();  // Configure for ping-pong speeds
      leds.setState(LED_PING_PONG);    // Switch to ping-pong LEDs
      stateMachine.transitionTo(STATE_PING_PONG);  // Go directly to ping-pong mode
    }
  }
}

// ========== MANUAL MOVEMENT FUNCTIONS ==========

void executeManualMoveToA() {
  static bool hasSetTarget = false;
  float currentPos = stepper.getCurrentPositionMM();
  float targetPos = pingPongStartMM;  // Safe clearance position near Home A
  float distance = abs(currentPos - targetPos);
  
  // Check if already at target
  if (distance < POSITION_TOLERANCE_MM) {
    Serial.println("Reached Home A safe position");
    hasSetTarget = false;  // Reset for next move
    leds.setManualMovement(false);  // Stop movement, switch to breathing
    stateMachine.transitionTo(STATE_MANUAL_WAIT_A);
    return;
  }
  
  // Configure for manual speed and set target only once when entering this state
  if (!hasSetTarget) {
    stepper.configureForManual();  // 3x faster speed with 2x acceleration
    stepper.moveToPositionMM(targetPos);
    hasSetTarget = true;
  }
  
  stepper.runAccelerated();
}

void executeManualMoveToB() {
  static bool hasSetTarget = false;
  float currentPos = stepper.getCurrentPositionMM();
  float targetPos = pingPongEndMM;  // Safe clearance position near Home B
  float distance = abs(currentPos - targetPos);
  
  // Check if already at target
  if (distance < POSITION_TOLERANCE_MM) {
    Serial.println("Reached Home B safe position");
    hasSetTarget = false;  // Reset for next move
    leds.setManualMovement(false);  // Stop movement, switch to breathing
    stateMachine.transitionTo(STATE_MANUAL_WAIT_B);
    return;
  }
  
  // Configure for manual speed and set target only once when entering this state
  if (!hasSetTarget) {
    stepper.configureForManual();  // 3x faster speed with 2x acceleration
    stepper.moveToPositionMM(targetPos);
    hasSetTarget = true;
  }
  
  stepper.runAccelerated();
}

void executeManualWaitA() {
  // Hold position at Home A safe position until button pressed again
  static bool hasReportedHolding = false;
  
  if (stepper.isMoving()) {
    stepper.stopAndHold();  // Normal stop and hold
    hasReportedHolding = false;  // Reset flag so we report once when stopped
  } else if (!hasReportedHolding) {
    Serial.println("Manual control: Holding at Home A safe position - press B button to move to Home B (power cycle to return to ping-pong)");
    hasReportedHolding = true;
  }
  
  // Check for button presses to move to other position
  if (buttons.isButtonBPressed()) {
    Serial.println("Manual control: Moving to Home B safe position");
    hasReportedHolding = false;  // Reset flag for movement
    leds.setManualMovement(true, true);   // Moving towards B (right)
    stateMachine.transitionTo(STATE_MANUAL_MOVE_TO_B);
  }
}

void executeManualWaitB() {
  // Hold position at Home B safe position until button pressed again
  static bool hasReportedHolding = false;
  
  if (stepper.isMoving()) {
    stepper.stopAndHold();  // Normal stop and hold
    hasReportedHolding = false;  // Reset flag so we report once when stopped
  } else if (!hasReportedHolding) {
    Serial.println("Manual control: Holding at Home B safe position - press A button to move to Home A (power cycle to return to ping-pong)");
    hasReportedHolding = true;
  }
  
  // Check for button presses to move to other position
  if (buttons.isButtonAPressed()) {
    Serial.println("Manual control: Moving to Home A safe position");
    hasReportedHolding = false;  // Reset flag for movement
    leds.setManualMovement(true, false);  // Moving towards A (left)
    stateMachine.transitionTo(STATE_MANUAL_MOVE_TO_A);
  }
}