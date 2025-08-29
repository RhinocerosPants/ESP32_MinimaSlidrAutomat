/*
 * ESP32 MinimaSlidrAutomat - State Machine Controller Implementation
 * v2.0 - Production Release
 * 
 * Project by Aaron Young
 * Developed with assistance from Anthropic's Claude Code
 * Repository: https://github.com/RhinocerosPants/ESP32_MinimaSlidrAutomat
 */

#include "SliderStateMachine.h"
#include <Arduino.h>

SliderStateMachine::SliderStateMachine()
  : currentState(STATE_BOOT),
    previousState(STATE_BOOT),
    stateStartTime(0),
    lastStatusUpdate(0),
    homingACompleted(false),
    homingBCompleted(false),
    travelMeasured(false),
    movingTowardsEnd(false),
    homingStartTime(0),
    pauseStartTime(0) {
}

// ========== INITIALIZATION ==========

bool SliderStateMachine::initialize() {
  currentState = STATE_BOOT;
  stateStartTime = millis();
  lastStatusUpdate = millis();
  
  Serial.println("SliderStateMachine: Initialized in BOOT state");
  return true;
}

// ========== STATE MANAGEMENT ==========

SliderState SliderStateMachine::getCurrentState() const {
  return currentState;
}

SliderState SliderStateMachine::getPreviousState() const {
  return previousState;
}

void SliderStateMachine::transitionTo(SliderState newState) {
  if (newState == currentState) {
    return;  // No change needed
  }
  
  if (!canTransitionTo(newState)) {
    Serial.printf("SliderStateMachine: ERROR - Invalid transition from %s to %s\n",
                  getStateString().c_str(), getStateString(newState).c_str());
    return;
  }
  
  previousState = currentState;
  currentState = newState;
  stateStartTime = millis();
  
  Serial.printf("SliderStateMachine: %s -> %s\n", 
                getStateString(previousState).c_str(),
                getStateString(currentState).c_str());
  
  // State-specific initialization
  switch (newState) {
    case STATE_HOMING_A:
      homingStartTime = millis();
      break;
    case STATE_HOMING_B:
      homingStartTime = millis();
      break;
    case STATE_PAUSING_AT_START:
      pauseStartTime = millis();
      break;
    case STATE_PING_PONG:
      movingTowardsEnd = false;  // Start by moving towards start (Home A side)
      break;
    default:
      break;
  }
}

unsigned long SliderStateMachine::getTimeInCurrentState() const {
  return millis() - stateStartTime;
}

// ========== STATE QUERIES ==========

bool SliderStateMachine::isInHomingState() const {
  return currentState == STATE_HOMING_A || currentState == STATE_HOMING_B;
}

bool SliderStateMachine::isInOperationalState() const {
  return currentState == STATE_PING_PONG;
}

bool SliderStateMachine::isInErrorState() const {
  return currentState == STATE_ERROR;
}

bool SliderStateMachine::canTransitionTo(SliderState newState) const {
  // Error state can be entered from any state
  if (newState == STATE_ERROR) {
    return true;
  }
  
  // Cannot exit error state (requires power cycle)
  if (currentState == STATE_ERROR) {
    return false;
  }
  
  // Valid state transitions
  switch (currentState) {
    case STATE_BOOT:
      return newState == STATE_HOMING_A;
      
    case STATE_HOMING_A:
      return newState == STATE_HOMING_B;
      
    case STATE_HOMING_B:
      return newState == STATE_MEASURING_TRAVEL;
      
    case STATE_MEASURING_TRAVEL:
      return newState == STATE_MOVING_TO_START;
      
    case STATE_MOVING_TO_START:
      return newState == STATE_PAUSING_AT_START;
      
    case STATE_PAUSING_AT_START:
      return newState == STATE_PING_PONG;
      
    case STATE_PING_PONG:
      return newState == STATE_MANUAL_MOVE_TO_A || newState == STATE_MANUAL_MOVE_TO_B;
      
    case STATE_MANUAL_MOVE_TO_A:
      return newState == STATE_MANUAL_WAIT_A;
      
    case STATE_MANUAL_MOVE_TO_B:
      return newState == STATE_MANUAL_WAIT_B;
      
    case STATE_MANUAL_WAIT_A:
      return newState == STATE_MANUAL_MOVE_TO_B || newState == STATE_PING_PONG;
      
    case STATE_MANUAL_WAIT_B:
      return newState == STATE_MANUAL_MOVE_TO_A || newState == STATE_PING_PONG;
      
    default:
      return false;
  }
}

// ========== HOMING STATE MANAGEMENT ==========

void SliderStateMachine::startHomingSequence() {
  transitionTo(STATE_HOMING_A);
}

void SliderStateMachine::completeHomingA() {
  homingACompleted = true;
  Serial.println("SliderStateMachine: Home A completed");
  transitionTo(STATE_HOMING_B);
}

void SliderStateMachine::completeHomingB() {
  homingBCompleted = true;
  Serial.println("SliderStateMachine: Home B completed");
  transitionTo(STATE_MEASURING_TRAVEL);
}

bool SliderStateMachine::isHomingACompleted() const {
  return homingACompleted;
}

bool SliderStateMachine::isHomingBCompleted() const {
  return homingBCompleted;
}

bool SliderStateMachine::hasHomingTimedOut() const {
  if (!isInHomingState()) {
    return false;
  }
  
  return (millis() - homingStartTime) > HOMING_TIMEOUT_MS;
}

// ========== TRAVEL MEASUREMENT ==========

void SliderStateMachine::markTravelMeasured() {
  travelMeasured = true;
  transitionTo(STATE_MOVING_TO_START);
}

bool SliderStateMachine::isTravelMeasured() const {
  return travelMeasured;
}

// ========== PING-PONG STATE MANAGEMENT ==========

void SliderStateMachine::startPause() {
  transitionTo(STATE_PAUSING_AT_START);
}

bool SliderStateMachine::isPauseComplete() const {
  if (currentState != STATE_PAUSING_AT_START) {
    return false;
  }
  
  return (millis() - pauseStartTime) >= START_PAUSE_MS;
}

void SliderStateMachine::startPingPong() {
  transitionTo(STATE_PING_PONG);
}

void SliderStateMachine::togglePingPongDirection() {
  if (currentState == STATE_PING_PONG) {
    movingTowardsEnd = !movingTowardsEnd;
    Serial.printf("SliderStateMachine: Ping-pong direction -> %s\n",
                  movingTowardsEnd ? "towards END" : "towards START");
  }
}

bool SliderStateMachine::isMovingTowardsEnd() const {
  return movingTowardsEnd;
}

// ========== ERROR HANDLING ==========

void SliderStateMachine::enterErrorState(const char* reason) {
  Serial.printf("SliderStateMachine: ENTERING ERROR STATE - %s\n", reason);
  transitionTo(STATE_ERROR);
}

bool SliderStateMachine::canRecoverFromError() const {
  return false;  // Always requires power cycle for safety
}

// ========== TIMEOUT MANAGEMENT ==========

bool SliderStateMachine::checkTimeouts() {
  if (hasHomingTimedOut()) {
    enterErrorState("Homing timeout - sensor not found");
    return true;
  }
  
  return false;
}

// ========== STATUS AND DEBUGGING ==========

void SliderStateMachine::printStateStatus() const {
  unsigned long timeInState = getTimeInCurrentState();
  
  Serial.printf("State: %s (%.1fs) - %s\n",
                getStateString().c_str(),
                timeInState / 1000.0f,
                getStateDescription().c_str());
}

String SliderStateMachine::getStateString() const {
  return getStateString(currentState);
}

String SliderStateMachine::getStateString(SliderState state) const {
  switch (state) {
    case STATE_BOOT: return "BOOT";
    case STATE_HOMING_A: return "HOMING_A";
    case STATE_HOMING_B: return "HOMING_B";
    case STATE_MEASURING_TRAVEL: return "MEASURING_TRAVEL";
    case STATE_MOVING_TO_START: return "MOVING_TO_START";
    case STATE_PAUSING_AT_START: return "PAUSING_AT_START";
    case STATE_PING_PONG: return "PING_PONG";
    case STATE_MANUAL_MOVE_TO_A: return "MANUAL_MOVE_TO_A";
    case STATE_MANUAL_MOVE_TO_B: return "MANUAL_MOVE_TO_B";
    case STATE_MANUAL_WAIT_A: return "MANUAL_WAIT_A";
    case STATE_MANUAL_WAIT_B: return "MANUAL_WAIT_B";
    case STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

String SliderStateMachine::getStateDescription() const {
  switch (currentState) {
    case STATE_BOOT: 
      return "System initialization";
    case STATE_HOMING_A: 
      return "Finding Home A sensor (left end)";
    case STATE_HOMING_B: 
      return "Finding Home B sensor (right end)";
    case STATE_MEASURING_TRAVEL: 
      return "Calculating travel distance and ping-pong range";
    case STATE_MOVING_TO_START: 
      return "Moving to ping-pong start position";
    case STATE_PAUSING_AT_START: 
      return "3-second pause before ping-pong begins";
    case STATE_PING_PONG: 
      return String("Continuous ping-pong motion (") + (movingTowardsEnd ? "-> END)" : "-> START)");
    case STATE_MANUAL_MOVE_TO_A:
      return "Manual move to Home A position";
    case STATE_MANUAL_MOVE_TO_B:
      return "Manual move to Home B position";
    case STATE_MANUAL_WAIT_A:
      return "Manual wait at Home A position";
    case STATE_MANUAL_WAIT_B:
      return "Manual wait at Home B position";
    case STATE_ERROR: 
      return "ERROR - Motor holding position, power cycle required";
    default: 
      return "Unknown state";
  }
}

void SliderStateMachine::updateStatusIfNeeded() {
  if (millis() - lastStatusUpdate >= STATUS_UPDATE_INTERVAL_MS) {
    printStateStatus();
    lastStatusUpdate = millis();
  }
}