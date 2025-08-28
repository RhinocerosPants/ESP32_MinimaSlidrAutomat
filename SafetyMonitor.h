/**
 * @file SafetyMonitor.h
 * @brief Safety Monitor System for MinimaSlidrAutomat
 * @version 2.0
 * @date 2024
 * @author MinimaSlidrAutomat Project
 * 
 * Comprehensive safety monitoring system that protects expensive camera
 * equipment through multi-layered safety checks including position bounds
 * validation, crash detection, and emergency stop conditions.
 * 
 * The safety system is designed with a "safety-first" approach where any
 * detected anomaly results in immediate motor stop with position holding
 * to prevent equipment damage on inclined setups.
 * 
 * @warning All safety violations result in motor stop with holding torque
 * @see StepperController, SensorManager
 */

#pragma once

#include "config.h"

/**
 * @class SafetyMonitor
 * @brief Comprehensive safety monitoring and validation system
 * 
 * Provides multi-layered safety monitoring including:
 * - Position bounds checking with configurable margins
 * - Crash detection through sensor monitoring
 * - Travel range validation
 * - Emergency condition detection
 * 
 * All safety checks use structured return types for detailed diagnostics.
 */
class SafetyMonitor {
private:
  bool isFullyHomed;
  float measuredTravelDistance;
  unsigned long lastSafetyCheck;
  
  static const unsigned long SAFETY_CHECK_INTERVAL_MS = 100;  // Check every 100ms
  
public:
  SafetyMonitor();
  
  // ========== INITIALIZATION ==========
  bool initialize();
  void setHomingComplete(float travelDistanceMM);
  
  // ========== BOUNDS CHECKING ==========
  
  /**
   * @struct BoundsCheckResult
   * @brief Result structure for position bounds checking
   * 
   * Contains detailed information about position validation including
   * which bounds were violated and the current safety margins.
   */
  struct BoundsCheckResult {
    bool withinBounds;        ///< True if position is within safe bounds
    bool belowMinimum;        ///< True if position is below minimum bound
    bool aboveMaximum;        ///< True if position is above maximum bound
    float currentPositionMM;  ///< Current position being checked
    float minimumBoundMM;     ///< Calculated minimum safe position
    float maximumBoundMM;     ///< Calculated maximum safe position
  };
  
  /**
   * @brief Check if current position is within safe bounds
   * 
   * Validates the current position against calculated safety bounds
   * based on measured travel distance plus safety margins.
   * 
   * @param currentPositionMM Current position to validate
   * @return BoundsCheckResult with detailed validation information
   * 
   * @note Bounds are only enforced after system is fully homed
   * @see isPositionSafe(), BOUNDS_SAFETY_MARGIN_MM
   */
  BoundsCheckResult checkPositionBounds(float currentPositionMM) const;
  
  /**
   * @brief Quick check if position is safe
   * 
   * Simplified bounds checking that returns true if position is safe.
   * Used for quick validation in main control loops.
   * 
   * @param currentPositionMM Position to validate
   * @return true if position is within safe bounds
   * @return false if position violates safety bounds
   * 
   * @note Returns true if system not fully homed (limited checking)
   */
  bool isPositionSafe(float currentPositionMM) const;
  
  // ========== CRASH DETECTION ==========
  struct CrashCheckResult {
    bool crashDetected;
    bool sensorAUnexpectedlyActive;
    bool sensorBUnexpectedlyActive;
    float currentPositionMM;
    float allowedRangeStartMM;
    float allowedRangeEndMM;
    const char* crashReason;
  };
  
  CrashCheckResult checkForCrash(float currentPositionMM,
                                float pingPongStartMM, 
                                float pingPongEndMM,
                                bool sensorAActive,
                                bool sensorBActive) const;
  
  // ========== TRAVEL RANGE VALIDATION ==========
  struct TravelValidationResult {
    bool isValid;
    float requestedTravelMM;
    float minimumRequiredMM;
    float calculatedRangeMM;
    const char* errorReason;
  };
  
  TravelValidationResult validateTravelRange(float measuredTravelMM) const;
  
  // ========== EMERGENCY CONDITIONS ==========
  struct EmergencyCheckResult {
    bool emergencyDetected;
    bool boundsViolation;
    bool crashDetected;
    bool sensorMalfunction;
    const char* emergencyReason;
  };
  
  EmergencyCheckResult performComprehensiveSafetyCheck(float currentPositionMM,
                                                      float pingPongStartMM,
                                                      float pingPongEndMM,
                                                      bool sensorAActive,
                                                      bool sensorBActive) const;
  
  // ========== POSITION VALIDATION ==========
  bool isPositionReasonable(float positionMM) const;       // Sanity check for encoder issues
  bool isWithinPhysicalLimits(float positionMM) const;     // Check against absolute physical limits
  
  // ========== SENSOR VALIDATION ==========
  bool areSensorsHealthy(bool sensorAActive, bool sensorBActive) const;  // Check for sensor malfunctions
  bool isSensorStateValid(bool sensorAActive, bool sensorBActive, float currentPositionMM) const;
  
  // ========== CONFIGURATION ==========
  void setTravelDistance(float travelMM);
  bool isSystemFullyHomed() const;
  
  // ========== MONITORING ==========
  void updateSafetyMonitoring();                          // Periodic safety check
  void printSafetyStatus(float currentPositionMM) const;
  
  // ========== CONSTANTS GETTERS ==========
  float getBoundsMinimumMM() const;
  float getBoundsMaximumMM() const;
  float getCrashToleranceMM() const;
};