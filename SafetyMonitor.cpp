/*
 * MinimaSlidrAutomat - Safety Monitor System Implementation
 * v2.0 - Production Release
 */

#include "SafetyMonitor.h"
#include <Arduino.h>

SafetyMonitor::SafetyMonitor()
  : isFullyHomed(false),
    measuredTravelDistance(0.0f),
    lastSafetyCheck(0) {
}

// ========== INITIALIZATION ==========

bool SafetyMonitor::initialize() {
  isFullyHomed = false;
  measuredTravelDistance = 0.0f;
  lastSafetyCheck = millis();
  
  Serial.println("SafetyMonitor: Initialized safety monitoring system");
  return true;
}

void SafetyMonitor::setHomingComplete(float travelDistanceMM) {
  measuredTravelDistance = travelDistanceMM;
  isFullyHomed = true;
  
  Serial.printf("SafetyMonitor: Homing complete - Travel: %.1fmm, Bounds: %.1f to %.1fmm\n",
                measuredTravelDistance, 
                getBoundsMinimumMM(),
                getBoundsMaximumMM());
}

// ========== BOUNDS CHECKING ==========

SafetyMonitor::BoundsCheckResult SafetyMonitor::checkPositionBounds(float currentPositionMM) const {
  BoundsCheckResult result = {};
  result.currentPositionMM = currentPositionMM;
  result.minimumBoundMM = getBoundsMinimumMM();
  result.maximumBoundMM = getBoundsMaximumMM();
  
  result.belowMinimum = currentPositionMM < result.minimumBoundMM;
  result.aboveMaximum = currentPositionMM > result.maximumBoundMM;
  result.withinBounds = !result.belowMinimum && !result.aboveMaximum;
  
  return result;
}

bool SafetyMonitor::isPositionSafe(float currentPositionMM) const {
  if (!isFullyHomed) {
    return true;  // No bounds checking until homing is complete
  }
  
  return checkPositionBounds(currentPositionMM).withinBounds;
}

// ========== CRASH DETECTION ==========

SafetyMonitor::CrashCheckResult SafetyMonitor::checkForCrash(float currentPositionMM,
                                                            float pingPongStartMM, 
                                                            float pingPongEndMM,
                                                            bool sensorAActive,
                                                            bool sensorBActive) const {
  CrashCheckResult result = {};
  result.currentPositionMM = currentPositionMM;
  result.allowedRangeStartMM = pingPongStartMM - getCrashToleranceMM();
  result.allowedRangeEndMM = pingPongEndMM + getCrashToleranceMM();
  
  // Check Home A sensor (should only be active near start position)
  if (sensorAActive && currentPositionMM > result.allowedRangeStartMM) {
    result.crashDetected = true;
    result.sensorAUnexpectedlyActive = true;
    result.crashReason = "Home A sensor active outside expected range";
  }
  
  // Check Home B sensor (should only be active near end position)
  if (sensorBActive && currentPositionMM < result.allowedRangeEndMM) {
    result.crashDetected = true;
    result.sensorBUnexpectedlyActive = true;
    result.crashReason = "Home B sensor active outside expected range";
  }
  
  return result;
}

// ========== TRAVEL RANGE VALIDATION ==========

SafetyMonitor::TravelValidationResult SafetyMonitor::validateTravelRange(float measuredTravelMM) const {
  TravelValidationResult result = {};
  result.requestedTravelMM = measuredTravelMM;
  result.minimumRequiredMM = MINIMUM_TRAVEL_RANGE_MM;
  
  // Calculate usable range after sensor clearance
  result.calculatedRangeMM = measuredTravelMM - (2 * SENSOR_CLEARANCE_MM);
  
  if (result.calculatedRangeMM < result.minimumRequiredMM) {
    result.isValid = false;
    result.errorReason = "Insufficient travel range after sensor clearance";
  } else {
    result.isValid = true;
  }
  
  return result;
}

// ========== EMERGENCY CONDITIONS ==========

SafetyMonitor::EmergencyCheckResult SafetyMonitor::performComprehensiveSafetyCheck(
    float currentPositionMM,
    float pingPongStartMM,
    float pingPongEndMM,
    bool sensorAActive,
    bool sensorBActive) const {
    
  EmergencyCheckResult result = {};
  
  // Check position bounds
  BoundsCheckResult boundsCheck = checkPositionBounds(currentPositionMM);
  if (!boundsCheck.withinBounds) {
    result.emergencyDetected = true;
    result.boundsViolation = true;
    result.emergencyReason = "Position outside safe bounds";
    return result;
  }
  
  // Check for crashes during ping-pong operation
  if (isFullyHomed) {
    CrashCheckResult crashCheck = checkForCrash(currentPositionMM, pingPongStartMM, 
                                               pingPongEndMM, sensorAActive, sensorBActive);
    if (crashCheck.crashDetected) {
      result.emergencyDetected = true;
      result.crashDetected = true;
      result.emergencyReason = crashCheck.crashReason;
      return result;
    }
  }
  
  // Check sensor health
  if (!areSensorsHealthy(sensorAActive, sensorBActive)) {
    result.emergencyDetected = true;
    result.sensorMalfunction = true;
    result.emergencyReason = "Sensor malfunction detected";
    return result;
  }
  
  return result;  // No emergency detected
}

// ========== POSITION VALIDATION ==========

bool SafetyMonitor::isPositionReasonable(float positionMM) const {
  // Sanity check for encoder/stepper issues
  const float MAX_REASONABLE_POSITION = 2000.0f;  // 2 meters maximum
  const float MIN_REASONABLE_POSITION = -200.0f;  // 20cm negative maximum
  
  return positionMM >= MIN_REASONABLE_POSITION && positionMM <= MAX_REASONABLE_POSITION;
}

bool SafetyMonitor::isWithinPhysicalLimits(float positionMM) const {
  if (!isFullyHomed) {
    return isPositionReasonable(positionMM);
  }
  
  return checkPositionBounds(positionMM).withinBounds;
}

// ========== SENSOR VALIDATION ==========

bool SafetyMonitor::areSensorsHealthy(bool sensorAActive, bool sensorBActive) const {
  // Check for impossible sensor states
  if (sensorAActive && sensorBActive) {
    // Both sensors active simultaneously is usually impossible
    return false;
  }
  
  // Additional sensor health checks could be added here
  return true;
}

bool SafetyMonitor::isSensorStateValid(bool sensorAActive, bool sensorBActive, float currentPositionMM) const {
  if (!isFullyHomed) {
    return true;  // Can't validate until we know the range
  }
  
  // Use crash detection logic for sensor state validation
  CrashCheckResult crashCheck = checkForCrash(currentPositionMM, 
                                             SENSOR_CLEARANCE_MM,
                                             measuredTravelDistance - SENSOR_CLEARANCE_MM,
                                             sensorAActive, sensorBActive);
  
  return !crashCheck.crashDetected;
}

// ========== CONFIGURATION ==========

void SafetyMonitor::setTravelDistance(float travelMM) {
  measuredTravelDistance = travelMM;
}

bool SafetyMonitor::isSystemFullyHomed() const {
  return isFullyHomed;
}

// ========== MONITORING ==========

void SafetyMonitor::updateSafetyMonitoring() {
  lastSafetyCheck = millis();
  // Periodic safety monitoring could be implemented here
}

void SafetyMonitor::printSafetyStatus(float currentPositionMM) const {
  if (!isFullyHomed) {
    Serial.println("Safety: System not fully homed - limited safety monitoring");
    return;
  }
  
  BoundsCheckResult bounds = checkPositionBounds(currentPositionMM);
  
  Serial.printf("Safety: Pos=%.1fmm, Bounds=[%.1f,%.1f], Status=%s\n",
                currentPositionMM,
                bounds.minimumBoundMM,
                bounds.maximumBoundMM,
                bounds.withinBounds ? "SAFE" : "OUT OF BOUNDS");
}

// ========== CONSTANTS GETTERS ==========

float SafetyMonitor::getBoundsMinimumMM() const {
  return -BOUNDS_SAFETY_MARGIN_MM;
}

float SafetyMonitor::getBoundsMaximumMM() const {
  return measuredTravelDistance + BOUNDS_SAFETY_MARGIN_MM;
}

float SafetyMonitor::getCrashToleranceMM() const {
  return CRASH_DETECTION_TOLERANCE_MM;
}