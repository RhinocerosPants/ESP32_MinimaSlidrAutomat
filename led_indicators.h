/*
 * MinimaSlidrAutomat - Simple LED Indicators
 * Three main states: Homing (breathing amber), Ping-pong (green animation), Error (breathing red)
 */

#pragma once

#include <FastLED.h>
#include "config.h"

// LED Configuration
#define LED_DATA_PIN      23        // WS2812B Data pin
#define NUM_LEDS          5         // Number of LEDs in strip
#define LED_BRIGHTNESS    64        // Default brightness (25% of 255)

class LEDIndicators {
private:
  CRGB leds[NUM_LEDS];
  LEDState currentState;
  uint8_t brightness;
  
  // Animation parameters
  uint32_t lastUpdate;
  uint16_t animationStep;
  bool breathingDirection;
  uint8_t pingPongPos;
  bool pingPongDirection;
  
  // Manual movement tracking
  bool manualMoving;
  bool manualDirection;  // true = right/towards B, false = left/towards A
  
  // Update intervals (milliseconds)
  static const uint16_t BREATHING_INTERVAL = 30;    // Smooth breathing
  static const uint16_t PING_PONG_INTERVAL = 225;   // Ping-pong speed (50% slower)
  static const uint16_t MANUAL_MOVE_INTERVAL = 100; // Manual move direction animation
  
  // Colors
  static const uint32_t COLOR_AMBER = 0xFF8000;     // Amber for homing
  static const uint32_t COLOR_GREEN = 0x00FF00;     // Green for ping-pong
  static const uint32_t COLOR_PURPLE = 0xFF00FF;    // Purple for manual moves
  static const uint32_t COLOR_RED = 0xFF0000;       // Red for error
  
public:
  LEDIndicators();
  ~LEDIndicators();
  
  // Initialization
  bool initialize();
  void setBrightness(uint8_t level);
  
  // State control
  void setState(LEDState state);
  void update();
  
  // Manual movement control
  void setManualMovement(bool moving, bool directionRight = true);
  
  // Manual control
  void clearAll();
  void setAll(uint32_t color);
  void testPattern();
  
private:
  // Animation functions
  void updateHomingAnimation();
  void updatePingPongAnimation();
  void updateManualAnimation();
  void updateErrorAnimation();
  
  // Utility functions
  void breathingEffect(uint32_t color);
  void pingPongEffect(uint32_t color);
  void manualDirectionEffect(uint32_t color);
  uint32_t dimColor(uint32_t color, uint8_t dimAmount);
  CRGB uint32ToCRGB(uint32_t color);
  bool isTimeForUpdate(uint16_t interval);
};