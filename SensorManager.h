/*
 * MinimaSlidrAutomat - Dual Hall Effect Sensor Manager
 * v2.0 - Production Release
 * 
 * Handles all dual sensor operations including reading, debouncing,
 * and power-on sensor clearing
 */

#pragma once

#include <Arduino.h>
#include "config.h"

class SensorManager {
private:
  bool lastSensorAState;
  bool lastSensorBState;
  unsigned long lastDebounceTime;
  static const unsigned long DEBOUNCE_DELAY_MS = 50;  // 50ms debounce
  
public:
  SensorManager();
  
  // ========== INITIALIZATION ==========
  bool initialize();
  
  // ========== SENSOR READING ==========
  bool readSensorA() const;                // Read Home A sensor (left end)
  bool readSensorB() const;                // Read Home B sensor (right end)
  bool readSensorADebounced();             // Debounced Home A reading
  bool readSensorBDebounced();             // Debounced Home B reading
  
  // ========== SENSOR STATE CHECKING ==========
  bool isSensorAActive() const;            // Check if Home A is currently active
  bool isSensorBActive() const;            // Check if Home B is currently active
  bool areAnySensorsActive() const;        // Check if any sensor is active
  bool areBothSensorsActive() const;       // Check if both sensors are active (error condition)
  
  // ========== POWER-ON SENSOR CLEARING ==========
  struct SensorClearingResult {
    bool sensorAWasActive;
    bool sensorBWasActive;
    bool clearingPerformed;
    int stepsExecutedA;
    int stepsExecutedB;
  };
  
  SensorClearingResult checkAndClearSensorsAtPowerOn();
  
  // ========== CRASH DETECTION ==========
  struct CrashDetectionResult {
    bool crashDetected;
    bool sensorATriggered;
    bool sensorBTriggered;
    float currentPositionMM;
    float allowedRangeStartMM;
    float allowedRangeEndMM;
  };
  
  CrashDetectionResult checkForCrash(float currentPositionMM, 
                                   float pingPongStartMM, 
                                   float pingPongEndMM) const;
  
  // ========== HOMING DETECTION ==========
  bool hasSensorATriggered() const;        // Check if Home A triggered during homing
  bool hasSensorBTriggered() const;        // Check if Home B triggered during homing
  
  // ========== DEBUG AND MONITORING ==========
  void printSensorStatus() const;          // Print current sensor states
  String getSensorStatusString() const;    // Get sensor status as formatted string
};