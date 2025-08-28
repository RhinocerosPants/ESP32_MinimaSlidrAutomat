/*
 * ESP32 MinimaSlidrAutomat - Button Manager
 * v2.0 - Production Release
 * 
 * Project by RhinocerosPants Team - Aaron Young
 * Developed with assistance from Anthropic's Claude Code
 * Repository: https://github.com/RhinocerosPants/ESP32_MinimaSlidrAutomat
 * 
 * Handles button inputs with debouncing for manual slider control
 */

#pragma once

#include <Arduino.h>
#include "config.h"

class ButtonManager {
private:
  bool lastButtonAState;
  bool lastButtonBState;
  bool lastButtonPingPongState;
  unsigned long lastButtonAPress;
  unsigned long lastButtonBPress;
  unsigned long lastButtonPingPongPress;
  
public:
  ButtonManager();
  
  // ========== INITIALIZATION ==========
  bool initialize();
  
  // ========== BUTTON READING ==========
  bool readButtonA() const;                    // Raw button A reading
  bool readButtonB() const;                    // Raw button B reading
  bool readButtonPingPong() const;             // Raw ping-pong return button reading
  bool isButtonAPressed();                     // Debounced button A press detection
  bool isButtonBPressed();                     // Debounced button B press detection
  bool isButtonPingPongPressed();              // Debounced ping-pong return button press detection
  
  // ========== STATUS ==========
  void printButtonStatus() const;              // Debug button states
};