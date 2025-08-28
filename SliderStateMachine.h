/*
 * MinimaSlidrAutomat - State Machine Controller
 * v2.0 - Production Release
 * 
 * Manages the slider state machine with clean transitions and timeout handling
 */

#pragma once

#include <Arduino.h>
#include "config.h"

class SliderStateMachine {
private:
  SliderState currentState;
  SliderState previousState;
  unsigned long stateStartTime;
  unsigned long lastStatusUpdate;
  
  // State-specific data
  bool homingACompleted;
  bool homingBCompleted;
  bool travelMeasured;
  bool movingTowardsEnd;          // Direction for ping-pong (true = towards B end)
  unsigned long homingStartTime;
  unsigned long pauseStartTime;
  
public:
  SliderStateMachine();
  
  // ========== INITIALIZATION ==========
  bool initialize();
  
  // ========== STATE MANAGEMENT ==========
  SliderState getCurrentState() const;
  SliderState getPreviousState() const;
  void transitionTo(SliderState newState);
  unsigned long getTimeInCurrentState() const;
  
  // ========== STATE QUERIES ==========
  bool isInHomingState() const;
  bool isInOperationalState() const;  
  bool isInErrorState() const;
  bool canTransitionTo(SliderState newState) const;
  
  // ========== HOMING STATE MANAGEMENT ==========
  void startHomingSequence();
  void completeHomingA();
  void completeHomingB(); 
  bool isHomingACompleted() const;
  bool isHomingBCompleted() const;
  bool hasHomingTimedOut() const;
  
  // ========== TRAVEL MEASUREMENT ==========
  void markTravelMeasured();
  bool isTravelMeasured() const;
  
  // ========== PING-PONG STATE MANAGEMENT ==========
  void startPause();
  bool isPauseComplete() const;
  void startPingPong();
  void togglePingPongDirection();
  bool isMovingTowardsEnd() const;
  
  // ========== ERROR HANDLING ==========
  void enterErrorState(const char* reason);
  bool canRecoverFromError() const;      // Always false - requires power cycle
  
  // ========== TIMEOUT MANAGEMENT ==========
  bool checkTimeouts();                  // Returns true if timeout occurred
  
  // ========== STATUS AND DEBUGGING ==========
  void printStateStatus() const;
  String getStateString() const;
  String getStateString(SliderState state) const;
  String getStateDescription() const;
  void updateStatusIfNeeded();           // Throttled status updates
};