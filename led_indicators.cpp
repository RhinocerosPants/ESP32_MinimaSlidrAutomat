/*
 * MinimaSlidrAutomat - Simple LED Indicators Implementation
 */

#include "led_indicators.h"

LEDIndicators::LEDIndicators() {
  currentState = LED_HOMING;
  brightness = LED_BRIGHTNESS;
  lastUpdate = 0;
  animationStep = 0;
  breathingDirection = true;
  pingPongPos = 0;
  pingPongDirection = true;
  manualMoving = false;
  manualDirection = true;
}

LEDIndicators::~LEDIndicators() {
  clearAll();
}

bool LEDIndicators::initialize() {
  // Initialize FastLED
  FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(brightness);
  
  // Clear all LEDs
  clearAll();
  
  // Test pattern on startup
  testPattern();
  
  Serial.println("LED Indicators initialized");
  return true;
}

void LEDIndicators::setBrightness(uint8_t level) {
  brightness = level;
  FastLED.setBrightness(brightness);
  FastLED.show();
}

void LEDIndicators::setState(LEDState state) {
  if (currentState != state) {
    currentState = state;
    animationStep = 0;
    lastUpdate = millis();
    
    // Reset animation parameters for new state
    switch (state) {
      case LED_HOMING:
        breathingDirection = true;
        Serial.println("LED: Homing mode (breathing amber)");
        break;
      case LED_PING_PONG:
        pingPongPos = 0;
        pingPongDirection = true;
        Serial.println("LED: Ping-pong mode (green animation)");
        break;
      case LED_MANUAL:
        breathingDirection = true;
        Serial.println("LED: Manual mode (breathing purple)");
        break;
      case LED_ERROR:
        breathingDirection = true;
        Serial.println("LED: Error mode (breathing red)");
        break;
    }
  }
}

void LEDIndicators::update() {
  switch (currentState) {
    case LED_HOMING:
      updateHomingAnimation();
      break;
    case LED_PING_PONG:
      updatePingPongAnimation();
      break;
    case LED_MANUAL:
      updateManualAnimation();
      break;
    case LED_ERROR:
      updateErrorAnimation();
      break;
  }
}

void LEDIndicators::updateHomingAnimation() {
  if (isTimeForUpdate(BREATHING_INTERVAL)) {
    breathingEffect(COLOR_AMBER);
    lastUpdate = millis();
  }
}

void LEDIndicators::updatePingPongAnimation() {
  if (isTimeForUpdate(PING_PONG_INTERVAL)) {
    pingPongEffect(COLOR_GREEN);
    lastUpdate = millis();
  }
}

void LEDIndicators::setManualMovement(bool moving, bool directionRight) {
  manualMoving = moving;
  manualDirection = directionRight;
  
  // Reset animation when movement changes
  if (moving) {
    animationStep = 0;
    if (directionRight) {
      pingPongPos = 0;  // Start from left for right movement
    } else {
      pingPongPos = NUM_LEDS - 1;  // Start from right for left movement
    }
  } else {
    // Reset breathing animation when stopping
    breathingDirection = true;
    animationStep = 30;  // Start at minimum brightness
  }
}

void LEDIndicators::updateManualAnimation() {
  if (manualMoving) {
    // Show directional movement animation
    if (isTimeForUpdate(MANUAL_MOVE_INTERVAL)) {
      manualDirectionEffect(COLOR_PURPLE);
      lastUpdate = millis();
    }
  } else {
    // Show breathing animation when stopped
    if (isTimeForUpdate(BREATHING_INTERVAL)) {
      breathingEffect(COLOR_PURPLE);
      lastUpdate = millis();
    }
  }
}

void LEDIndicators::updateErrorAnimation() {
  if (isTimeForUpdate(BREATHING_INTERVAL)) {
    breathingEffect(COLOR_RED);
    lastUpdate = millis();
  }
}

void LEDIndicators::breathingEffect(uint32_t color) {
  // Breathing animation using sine wave approximation
  if (breathingDirection) {
    animationStep++;
    if (animationStep >= 255) {
      breathingDirection = false;
    }
  } else {
    animationStep--;
    if (animationStep <= 30) {  // Don't go completely dark
      breathingDirection = true;
    }
  }
  
  // Apply breathing effect to all LEDs
  uint32_t dimmedColor = dimColor(color, animationStep);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = uint32ToCRGB(dimmedColor);
  }
  
  FastLED.show();
}

void LEDIndicators::pingPongEffect(uint32_t color) {
  // Clear all LEDs first
  clearAll();
  
  // Move the active LED back and forth
  if (pingPongDirection) {
    pingPongPos++;
    if (pingPongPos >= NUM_LEDS - 1) {
      pingPongDirection = false;
    }
  } else {
    pingPongPos--;
    if (pingPongPos <= 0) {
      pingPongDirection = true;
    }
  }
  
  // Set the current position LED
  leds[pingPongPos] = uint32ToCRGB(color);
  
  // Add trailing effect for smoother animation
  if (pingPongDirection && pingPongPos > 0) {
    leds[pingPongPos - 1] = uint32ToCRGB(dimColor(color, 100));
  } else if (!pingPongDirection && pingPongPos < NUM_LEDS - 1) {
    leds[pingPongPos + 1] = uint32ToCRGB(dimColor(color, 100));
  }
  
  FastLED.show();
}

void LEDIndicators::manualDirectionEffect(uint32_t color) {
  // Clear all LEDs first
  clearAll();
  
  if (manualDirection) {
    // Moving right/towards B - sweep from left to right
    animationStep++;
    if (animationStep >= NUM_LEDS * 3) {
      animationStep = 0;  // Reset animation cycle
    }
    
    // Create a moving sweep effect
    int activePos = (animationStep / 3) % NUM_LEDS;
    for (int i = 0; i <= activePos; i++) {
      uint8_t brightness = 255 - (activePos - i) * 60;  // Trailing fade effect
      if (brightness > 50) {  // Keep minimum visibility
        leds[i] = uint32ToCRGB(dimColor(color, brightness));
      }
    }
  } else {
    // Moving left/towards A - sweep from right to left
    animationStep++;
    if (animationStep >= NUM_LEDS * 3) {
      animationStep = 0;  // Reset animation cycle
    }
    
    // Create a moving sweep effect from right
    int activePos = NUM_LEDS - 1 - ((animationStep / 3) % NUM_LEDS);
    for (int i = NUM_LEDS - 1; i >= activePos; i--) {
      uint8_t brightness = 255 - (i - activePos) * 60;  // Trailing fade effect
      if (brightness > 50) {  // Keep minimum visibility
        leds[i] = uint32ToCRGB(dimColor(color, brightness));
      }
    }
  }
  
  FastLED.show();
}

void LEDIndicators::clearAll() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

void LEDIndicators::setAll(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = uint32ToCRGB(color);
  }
  FastLED.show();
}

void LEDIndicators::testPattern() {
  // Quick test pattern on startup
  Serial.println("LED: Running test pattern...");
  
  // Flash each LED in sequence
  for (int i = 0; i < NUM_LEDS; i++) {
    clearAll();
    leds[i] = CRGB::Blue;
    FastLED.show();
    delay(100);
  }
  
  // Flash all white
  setAll(0xFFFFFF);
  delay(200);
  clearAll();
  delay(200);
  
  Serial.println("LED: Test pattern complete");
}

uint32_t LEDIndicators::dimColor(uint32_t color, uint8_t dimAmount) {
  uint8_t r = (color >> 16) & 0xFF;
  uint8_t g = (color >> 8) & 0xFF;
  uint8_t b = color & 0xFF;
  
  r = (r * dimAmount) / 255;
  g = (g * dimAmount) / 255;  
  b = (b * dimAmount) / 255;
  
  return (r << 16) | (g << 8) | b;
}

CRGB LEDIndicators::uint32ToCRGB(uint32_t color) {
  uint8_t r = (color >> 16) & 0xFF;
  uint8_t g = (color >> 8) & 0xFF;
  uint8_t b = color & 0xFF;
  return CRGB(r, g, b);
}

bool LEDIndicators::isTimeForUpdate(uint16_t interval) {
  return (millis() - lastUpdate) >= interval;
}