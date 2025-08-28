/*
 * MinimaSlidrAutomat - Dual Hall Effect Sensor Manager Implementation
 * v2.0 - Production Release
 */

#include "SensorManager.h"
#include <Arduino.h>

SensorManager::SensorManager() 
  : lastSensorAState(false),
    lastSensorBState(false),
    lastDebounceTime(0) {
}

// ========== INITIALIZATION ==========

bool SensorManager::initialize() {
  // Configure sensor pins with internal pullup resistors
  // Sensors are active-low (LOW when magnet is near)
  pinMode(HALL_SENSOR_A_PIN, INPUT_PULLUP);
  pinMode(HALL_SENSOR_B_PIN, INPUT_PULLUP);
  
  // Read initial states
  lastSensorAState = readSensorA();
  lastSensorBState = readSensorB();
  
  Serial.println("SensorManager: Initialized dual hall effect sensors");
  Serial.printf("Initial states - Home A: %s, Home B: %s\n",
                lastSensorAState ? "ACTIVE" : "inactive",
                lastSensorBState ? "ACTIVE" : "inactive");
  
  return true;
}

// ========== SENSOR READING ==========

bool SensorManager::readSensorA() const {
  // Hall effect sensors are active-low (LOW = magnet detected)
  return digitalRead(HALL_SENSOR_A_PIN) == LOW;
}

bool SensorManager::readSensorB() const {
  // Hall effect sensors are active-low (LOW = magnet detected)
  return digitalRead(HALL_SENSOR_B_PIN) == LOW;
}

bool SensorManager::readSensorADebounced() {
  bool currentState = readSensorA();
  
  if (currentState != lastSensorAState) {
    lastDebounceTime = millis();
    lastSensorAState = currentState;
    return false;  // State is changing, not stable yet
  }
  
  // State has been stable for debounce period
  if (millis() - lastDebounceTime > DEBOUNCE_DELAY_MS) {
    return currentState;
  }
  
  return false;  // Still within debounce period
}

bool SensorManager::readSensorBDebounced() {
  bool currentState = readSensorB();
  
  if (currentState != lastSensorBState) {
    lastDebounceTime = millis();
    lastSensorBState = currentState;
    return false;  // State is changing, not stable yet
  }
  
  // State has been stable for debounce period
  if (millis() - lastDebounceTime > DEBOUNCE_DELAY_MS) {
    return currentState;
  }
  
  return false;  // Still within debounce period
}

// ========== SENSOR STATE CHECKING ==========

bool SensorManager::isSensorAActive() const {
  return readSensorA();
}

bool SensorManager::isSensorBActive() const {
  return readSensorB();
}

bool SensorManager::areAnySensorsActive() const {
  return readSensorA() || readSensorB();
}

bool SensorManager::areBothSensorsActive() const {
  return readSensorA() && readSensorB();
}

// ========== POWER-ON SENSOR CLEARING ==========

SensorManager::SensorClearingResult SensorManager::checkAndClearSensorsAtPowerOn() {
  SensorClearingResult result = {false, false, false, 0, 0};
  
  result.sensorAWasActive = readSensorA();
  result.sensorBWasActive = readSensorB();
  
  if (!result.sensorAWasActive && !result.sensorBWasActive) {
    Serial.println("SensorManager: No sensors active at power-on - no clearing needed");
    return result;
  }
  
  result.clearingPerformed = true;
  Serial.println("SensorManager: POWER-ON SENSOR CLEARING - moving away from active sensors");
  
  // Configure pins for direct step control
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  
  if (result.sensorAWasActive) {
    Serial.println("SensorManager: Home A sensor active - moving RIGHT to clear");
    digitalWrite(MOTOR_DIR_PIN, HIGH);  // Set direction RIGHT
    
    for (int i = 0; i < SENSOR_CLEAR_STEPS; i++) {
      digitalWrite(MOTOR_STEP_PIN, HIGH);
      delayMicroseconds(STEP_PULSE_WIDTH_US);
      digitalWrite(MOTOR_STEP_PIN, LOW);
      delayMicroseconds(STEP_INTERVAL_US);
      result.stepsExecutedA++;
    }
  }
  
  if (result.sensorBWasActive) {
    Serial.println("SensorManager: Home B sensor active - moving LEFT to clear");
    digitalWrite(MOTOR_DIR_PIN, LOW);   // Set direction LEFT
    
    for (int i = 0; i < SENSOR_CLEAR_STEPS; i++) {
      digitalWrite(MOTOR_STEP_PIN, HIGH);
      delayMicroseconds(STEP_PULSE_WIDTH_US);
      digitalWrite(MOTOR_STEP_PIN, LOW);
      delayMicroseconds(STEP_INTERVAL_US);
      result.stepsExecutedB++;
    }
  }
  
  Serial.printf("SensorManager: Sensor clearing complete - A: %d steps, B: %d steps\n",
                result.stepsExecutedA, result.stepsExecutedB);
  
  // Verify sensors are now clear
  delay(100);  // Allow settling time
  if (areAnySensorsActive()) {
    Serial.println("WARNING: Sensors still active after clearing - may need manual intervention");
  } else {
    Serial.println("SensorManager: All sensors cleared successfully");
  }
  
  return result;
}

// ========== CRASH DETECTION ==========

SensorManager::CrashDetectionResult SensorManager::checkForCrash(float currentPositionMM, 
                                                                float pingPongStartMM, 
                                                                float pingPongEndMM) const {
  CrashDetectionResult result = {false, false, false, currentPositionMM, 0, 0};
  
  // Calculate allowed range with tolerance
  result.allowedRangeStartMM = pingPongStartMM - CRASH_DETECTION_TOLERANCE_MM;
  result.allowedRangeEndMM = pingPongEndMM + CRASH_DETECTION_TOLERANCE_MM;
  
  // Check Home A sensor (should only be active near start position)
  if (readSensorA() && currentPositionMM > result.allowedRangeStartMM) {
    result.crashDetected = true;
    result.sensorATriggered = true;
  }
  
  // Check Home B sensor (should only be active near end position)
  if (readSensorB() && currentPositionMM < result.allowedRangeEndMM) {
    result.crashDetected = true;
    result.sensorBTriggered = true;
  }
  
  return result;
}

// ========== HOMING DETECTION ==========

bool SensorManager::hasSensorATriggered() const {
  return readSensorA();
}

bool SensorManager::hasSensorBTriggered() const {
  return readSensorB();
}

// ========== DEBUG AND MONITORING ==========

void SensorManager::printSensorStatus() const {
  Serial.printf("Sensors - Home A: %s, Home B: %s\n",
                readSensorA() ? "ACTIVE" : "inactive",
                readSensorB() ? "ACTIVE" : "inactive");
}

String SensorManager::getSensorStatusString() const {
  String status = "A:";
  status += readSensorA() ? "ACTIVE" : "inactive";
  status += " B:";
  status += readSensorB() ? "ACTIVE" : "inactive";
  return status;
}