# MinimaSlidrAutomat - Testing Guide
## v2.0 - Production Release

Comprehensive testing guide for the MinimaSlidrAutomat separated architecture. This guide covers unit testing strategies, integration testing, and hardware-in-the-loop testing approaches.

## Overview

The separated architecture enables comprehensive testing of individual components. Each class can be tested independently using mock objects and test fixtures.

## Testing Strategy

### Unit Testing
- **Individual class testing** with mocked dependencies
- **Algorithm validation** for safety and motion calculations
- **State machine logic** verification
- **Boundary condition testing** for safety systems

### Integration Testing  
- **Component interaction** testing
- **End-to-end sequence** validation
- **Error handling** across component boundaries

### Hardware-in-the-Loop (HIL) Testing
- **Complete system testing** with real hardware
- **Sensor failure simulation** 
- **Emergency response** validation

---

## Unit Testing Examples

### 1. SafetyMonitor Bounds Checking Tests

```cpp
// test_SafetyMonitor.cpp
#include "SafetyMonitor.h"
#include <assert.h>

class SafetyMonitorTest {
private:
    SafetyMonitor safety;
    
public:
    void setUp() {
        safety.initialize();
        // Simulate completed homing with 500mm travel
        safety.setHomingComplete(500.0f);
    }
    
    void testPositionWithinBounds() {
        // Test position well within bounds
        SafetyMonitor::BoundsCheckResult result = safety.checkPositionBounds(250.0f);
        
        assert(result.withinBounds == true);
        assert(result.belowMinimum == false);
        assert(result.aboveMaximum == false);
        assert(result.currentPositionMM == 250.0f);
        
        Serial.println("✅ testPositionWithinBounds passed");
    }
    
    void testPositionBelowMinimum() {
        // Test position below minimum bound (-50mm)
        SafetyMonitor::BoundsCheckResult result = safety.checkPositionBounds(-60.0f);
        
        assert(result.withinBounds == false);
        assert(result.belowMinimum == true);
        assert(result.aboveMaximum == false);
        assert(result.minimumBoundMM == -50.0f);  // BOUNDS_SAFETY_MARGIN_MM
        
        Serial.println("✅ testPositionBelowMinimum passed");
    }
    
    void testPositionAboveMaximum() {
        // Test position above maximum bound (500 + 50 = 550mm)
        SafetyMonitor::BoundsCheckResult result = safety.checkPositionBounds(560.0f);
        
        assert(result.withinBounds == false);
        assert(result.belowMinimum == false);
        assert(result.aboveMaximum == true);
        assert(result.maximumBoundMM == 550.0f);  // measuredTravel + BOUNDS_SAFETY_MARGIN_MM
        
        Serial.println("✅ testPositionAboveMaximum passed");
    }
    
    void testTravelRangeValidation() {
        // Test insufficient travel range
        SafetyMonitor::TravelValidationResult result = safety.validateTravelRange(80.0f);
        
        assert(result.isValid == false);
        assert(result.calculatedRangeMM == 40.0f);  // 80 - (2 * 20 clearance)
        assert(result.minimumRequiredMM == 100.0f); // MINIMUM_TRAVEL_RANGE_MM
        
        Serial.println("✅ testTravelRangeValidation passed");
    }
    
    void runAllTests() {
        Serial.println("Running SafetyMonitor Unit Tests...");
        setUp();
        testPositionWithinBounds();
        testPositionBelowMinimum();
        testPositionAboveMaximum();
        testTravelRangeValidation();
        Serial.println("✅ All SafetyMonitor tests passed!\n");
    }
};
```

### 2. SliderStateMachine Transition Tests

```cpp
// test_SliderStateMachine.cpp
#include "SliderStateMachine.h"
#include <assert.h>

class SliderStateMachineTest {
private:
    SliderStateMachine stateMachine;
    
public:
    void setUp() {
        stateMachine.initialize();
    }
    
    void testValidStateTransitions() {
        // Test normal boot sequence
        assert(stateMachine.getCurrentState() == STATE_BOOT);
        
        stateMachine.transitionTo(STATE_HOMING_A);
        assert(stateMachine.getCurrentState() == STATE_HOMING_A);
        
        stateMachine.completeHomingA();
        assert(stateMachine.getCurrentState() == STATE_HOMING_B);
        assert(stateMachine.isHomingACompleted() == true);
        
        stateMachine.completeHomingB();
        assert(stateMachine.getCurrentState() == STATE_MEASURING_TRAVEL);
        assert(stateMachine.isHomingBCompleted() == true);
        
        Serial.println("✅ testValidStateTransitions passed");
    }
    
    void testInvalidStateTransitions() {
        // Test invalid transition rejection
        stateMachine.transitionTo(STATE_HOMING_A);
        
        // Try to skip directly to ping-pong (should fail)
        SliderState stateBefore = stateMachine.getCurrentState();
        stateMachine.transitionTo(STATE_PING_PONG);
        
        assert(stateMachine.getCurrentState() == stateBefore);  // State unchanged
        
        Serial.println("✅ testInvalidStateTransitions passed");
    }
    
    void testErrorStateTransition() {
        // Error state can be entered from any state
        stateMachine.transitionTo(STATE_HOMING_A);
        stateMachine.enterErrorState("Test error");
        
        assert(stateMachine.getCurrentState() == STATE_ERROR);
        assert(stateMachine.isInErrorState() == true);
        
        // Cannot exit error state
        stateMachine.transitionTo(STATE_PING_PONG);
        assert(stateMachine.getCurrentState() == STATE_ERROR);
        
        Serial.println("✅ testErrorStateTransition passed");
    }
    
    void testPingPongDirectionToggle() {
        // Advance to ping-pong state
        stateMachine.transitionTo(STATE_HOMING_A);
        stateMachine.completeHomingA();
        stateMachine.completeHomingB();
        stateMachine.markTravelMeasured();
        stateMachine.startPause();
        stateMachine.startPingPong();
        
        // Test direction toggling
        assert(stateMachine.isMovingTowardsEnd() == false);  // Initially towards start
        
        stateMachine.togglePingPongDirection();
        assert(stateMachine.isMovingTowardsEnd() == true);   // Now towards end
        
        stateMachine.togglePingPongDirection();
        assert(stateMachine.isMovingTowardsEnd() == false);  // Back to start
        
        Serial.println("✅ testPingPongDirectionToggle passed");
    }
    
    void runAllTests() {
        Serial.println("Running SliderStateMachine Unit Tests...");
        setUp();
        testValidStateTransitions();
        setUp(); // Reset for next test
        testInvalidStateTransitions();
        setUp(); // Reset for next test  
        testErrorStateTransition();
        setUp(); // Reset for next test
        testPingPongDirectionToggle();
        Serial.println("✅ All SliderStateMachine tests passed!\n");
    }
};
```

### 3. SensorManager Debouncing Tests

```cpp
// test_SensorManager.cpp (Requires mock hardware)
#include "SensorManager.h"
#include <assert.h>

class MockSensorManager : public SensorManager {
public:
    bool mockSensorAState = false;
    bool mockSensorBState = false;
    
    // Override sensor reading for testing
    bool readSensorA() const override {
        return mockSensorAState;
    }
    
    bool readSensorB() const override {
        return mockSensorBState;
    }
};

class SensorManagerTest {
private:
    MockSensorManager sensors;
    
public:
    void setUp() {
        sensors.initialize();
        sensors.mockSensorAState = false;
        sensors.mockSensorBState = false;
    }
    
    void testSensorReading() {
        // Test basic sensor reading
        assert(sensors.isSensorAActive() == false);
        assert(sensors.isSensorBActive() == false);
        assert(sensors.areAnySensorsActive() == false);
        
        sensors.mockSensorAState = true;
        assert(sensors.isSensorAActive() == true);
        assert(sensors.areAnySensorsActive() == true);
        
        sensors.mockSensorBState = true;
        assert(sensors.areBothSensorsActive() == true);
        
        Serial.println("✅ testSensorReading passed");
    }
    
    void testCrashDetection() {
        // Test crash detection outside normal range
        sensors.mockSensorAState = true;  // Sensor A active
        
        SensorManager::CrashDetectionResult result = 
            sensors.checkForCrash(50.0f,    // Current position
                                 20.0f,    // Ping-pong start
                                 480.0f,   // Ping-pong end
                                 true,     // Sensor A active
                                 false);   // Sensor B inactive
        
        assert(result.crashDetected == true);
        assert(result.sensorAUnexpectedlyActive == true);
        assert(result.sensorBUnexpectedlyActive == false);
        
        Serial.println("✅ testCrashDetection passed");
    }
    
    void runAllTests() {
        Serial.println("Running SensorManager Unit Tests...");
        setUp();
        testSensorReading();
        setUp();
        testCrashDetection();
        Serial.println("✅ All SensorManager tests passed!\n");
    }
};
```

---

## Integration Testing Examples

### Complete Homing Sequence Test

```cpp
// test_HomingSequence.cpp
#include "StepperController.h"
#include "SensorManager.h"
#include "SliderStateMachine.h"
#include "SafetyMonitor.h"

class HomingSequenceTest {
private:
    StepperController stepper;
    SensorManager sensors;
    SliderStateMachine stateMachine;
    SafetyMonitor safety;
    
public:
    void setUp() {
        stepper.initialize();
        sensors.initialize();
        stateMachine.initialize();
        safety.initialize();
    }
    
    void testCompleteHomingSequence() {
        Serial.println("Testing complete homing sequence...");
        
        // Start homing sequence
        stateMachine.startHomingSequence();
        assert(stateMachine.getCurrentState() == STATE_HOMING_A);
        
        // Simulate Home A sensor detection
        stepper.setCurrentPositionMM(0.0f);  // Establish reference
        stateMachine.completeHomingA();
        assert(stateMachine.getCurrentState() == STATE_HOMING_B);
        
        // Simulate movement to Home B and detection
        stepper.setCurrentPositionMM(500.0f);  // Simulate 500mm travel
        stateMachine.completeHomingB();
        assert(stateMachine.getCurrentState() == STATE_MEASURING_TRAVEL);
        
        // Complete travel measurement
        safety.setHomingComplete(500.0f);
        stateMachine.markTravelMeasured();
        assert(stateMachine.getCurrentState() == STATE_MOVING_TO_START);
        
        Serial.println("✅ Complete homing sequence test passed");
    }
    
    void runAllTests() {
        Serial.println("Running Integration Tests...");
        setUp();
        testCompleteHomingSequence();
        Serial.println("✅ All Integration tests passed!\n");
    }
};
```

---

## Test Runner

```cpp
// TestRunner.ino - Complete test suite runner
#include "config.h"

// Enable debug mode for testing
#define DEBUG_MODE

// Include test classes (would be separate files in practice)
// #include "test_SafetyMonitor.cpp"
// #include "test_SliderStateMachine.cpp"
// #include "test_SensorManager.cpp"
// #include "test_HomingSequence.cpp"

void setup() {
    Serial.begin(115200);
    delay(2000);  // Wait for serial connection
    
    Serial.println("=== MinimaSlidrAutomat Test Suite ===");
    Serial.println("v2.0 - Production Release\n");
    
    // Run unit tests
    SafetyMonitorTest safetyTests;
    safetyTests.runAllTests();
    
    SliderStateMachineTest stateTests;
    stateTests.runAllTests();
    
    SensorManagerTest sensorTests;
    sensorTests.runAllTests();
    
    // Run integration tests
    HomingSequenceTest homingTests;
    homingTests.runAllTests();
    
    Serial.println("=== ALL TESTS COMPLETED ===");
    Serial.println("✅ Test Suite PASSED");
}

void loop() {
    // Test runner runs once
    delay(1000);
}
```

---

## Hardware-in-the-Loop Testing

### 1. Complete System Test

Upload the main firmware and verify:

- **Power-on sequence** completes without errors
- **Dual sensor homing** finds both sensors correctly  
- **Travel measurement** reports reasonable distance
- **Ping-pong motion** operates smoothly within calculated range
- **Emergency stops** work when sensors triggered unexpectedly

### 2. Sensor Failure Simulation

- **Disconnect Home A sensor** → Verify timeout and error state
- **Disconnect Home B sensor** → Verify timeout and error state
- **Short sensor to ground** → Verify immediate error detection
- **Mechanical obstruction** → Verify crash detection works

### 3. Power Cycle Recovery

- **Interrupt during homing** → Verify clean restart
- **Interrupt during ping-pong** → Verify position recovery
- **Error state recovery** → Verify requires power cycle

### 4. Performance Validation

- **Measure actual speeds** with oscilloscope or laser measurement
- **Verify position accuracy** over multiple cycles
- **Test on inclined setup** → Verify holding torque during errors
- **Long-term reliability** → Run for extended periods

---

## Test Development Guidelines

### Creating New Tests

1. **Use structured return types** for complex validation
2. **Test boundary conditions** thoroughly
3. **Mock hardware dependencies** for unit tests
4. **Include negative test cases** (expected failures)
5. **Verify error handling paths** are tested

### Test Organization

```cpp
class ComponentTest {
    void setUp();           // Initialize test environment
    void tearDown();        // Clean up after tests
    void testFeature1();    // Individual test methods
    void testFeature2(); 
    void runAllTests();     // Execute all tests for this component
};
```

### Assertion Guidelines

```cpp
// Good assertions with meaningful messages
assert(result.isValid == true);                    // Clear expectation
assert(position >= 0.0f && position <= 500.0f);   // Range validation
assert(stateMachine.getCurrentState() == STATE_HOMING_A);  // State validation
```

---

## Continuous Testing

### Development Workflow

1. **Write tests first** for new features (TDD approach)
2. **Run unit tests** before each commit
3. **Integration tests** before releases
4. **HIL testing** for hardware changes

### Automated Testing

Consider setting up automated testing using:
- **PlatformIO Unit Testing** framework
- **GitHub Actions** for continuous integration
- **Hardware test rigs** for automated HIL testing

---

## Testing Best Practices

### Unit Testing
- **Test one thing at a time** - focused test methods
- **Use descriptive test names** - `testPositionBelowMinimum()`
- **Mock external dependencies** - hardware abstraction
- **Test edge cases** - boundary conditions and error paths

### Integration Testing  
- **Test component interactions** - verify interfaces work together
- **Validate end-to-end scenarios** - complete operation sequences
- **Error propagation testing** - ensure errors bubble up correctly

### Hardware Testing
- **Real-world conditions** - test on actual hardware setup
- **Environmental factors** - temperature, vibration, power variations
- **Long-term reliability** - extended operation testing
- **Safety validation** - emergency scenarios and failure modes

This comprehensive testing approach ensures the MinimaSlidrAutomat system is reliable, safe, and maintainable in production environments.