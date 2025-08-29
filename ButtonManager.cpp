/*
 * ESP32 MinimaSlidrAutomat - Button Manager Implementation
 * v2.0 - Production Release
 * 
 * Project by Aaron Young
 * Developed with assistance from Anthropic's Claude Code
 * Repository: https://github.com/RhinocerosPants/ESP32_MinimaSlidrAutomat
 */

#include "ButtonManager.h"

ButtonManager::ButtonManager() 
  : lastButtonAState(true),
    lastButtonBState(true),
    lastButtonPingPongState(true),
    lastButtonAPress(0),
    lastButtonBPress(0),
    lastButtonPingPongPress(0) {
}

// ========== INITIALIZATION ==========

bool ButtonManager::initialize() {
  // Configure button pins with internal pull-up resistors
  pinMode(BUTTON_MOVE_TO_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MOVE_TO_B_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RETURN_PING_PONG_PIN, INPUT_PULLUP);
  
  // Initialize button states
  lastButtonAState = digitalRead(BUTTON_MOVE_TO_A_PIN);
  lastButtonBState = digitalRead(BUTTON_MOVE_TO_B_PIN);
  lastButtonPingPongState = digitalRead(BUTTON_RETURN_PING_PONG_PIN);
  
  Serial.println("ButtonManager: Initialized with pull-up resistors");
  Serial.printf("Initial states - Move to A: %s, Move to B: %s, Return to ping-pong: %s\n",
                lastButtonAState ? "released" : "pressed",
                lastButtonBState ? "released" : "pressed",
                lastButtonPingPongState ? "released" : "pressed");
  
  return true;
}

// ========== BUTTON READING ==========

bool ButtonManager::readButtonA() const {
  return !digitalRead(BUTTON_MOVE_TO_A_PIN);  // Invert because of pull-up (LOW = pressed)
}

bool ButtonManager::readButtonB() const {
  return !digitalRead(BUTTON_MOVE_TO_B_PIN);  // Invert because of pull-up (LOW = pressed)
}

bool ButtonManager::readButtonPingPong() const {
  return !digitalRead(BUTTON_RETURN_PING_PONG_PIN);  // Invert because of pull-up (LOW = pressed)
}

bool ButtonManager::isButtonAPressed() {
  bool currentState = readButtonA();
  unsigned long currentTime = millis();
  
  // Check for press (transition from released to pressed)
  if (currentState && !lastButtonAState) {
    // Debounce check
    if (currentTime - lastButtonAPress >= BUTTON_DEBOUNCE_MS) {
      lastButtonAPress = currentTime;
      lastButtonAState = currentState;
      DEBUG_PRINTLN("ButtonManager: Move to A button pressed");
      return true;
    }
  }
  
  lastButtonAState = currentState;
  return false;
}

bool ButtonManager::isButtonBPressed() {
  bool currentState = readButtonB();
  unsigned long currentTime = millis();
  
  // Check for press (transition from released to pressed)
  if (currentState && !lastButtonBState) {
    // Debounce check
    if (currentTime - lastButtonBPress >= BUTTON_DEBOUNCE_MS) {
      lastButtonBPress = currentTime;
      lastButtonBState = currentState;
      DEBUG_PRINTLN("ButtonManager: Move to B button pressed");
      return true;
    }
  }
  
  lastButtonBState = currentState;
  return false;
}

bool ButtonManager::isButtonPingPongPressed() {
  bool currentState = readButtonPingPong();
  unsigned long currentTime = millis();
  
  // Check for press (transition from released to pressed)
  if (currentState && !lastButtonPingPongState) {
    // Debounce check
    if (currentTime - lastButtonPingPongPress >= BUTTON_DEBOUNCE_MS) {
      lastButtonPingPongPress = currentTime;
      lastButtonPingPongState = currentState;
      DEBUG_PRINTLN("ButtonManager: Return to ping-pong button pressed");
      return true;
    }
  }
  
  lastButtonPingPongState = currentState;
  return false;
}

// ========== STATUS ==========

void ButtonManager::printButtonStatus() const {
  Serial.printf("Buttons - Move to A: %s, Move to B: %s, Return to ping-pong: %s\n",
                readButtonA() ? "PRESSED" : "released",
                readButtonB() ? "PRESSED" : "released",
                readButtonPingPong() ? "PRESSED" : "released");
}