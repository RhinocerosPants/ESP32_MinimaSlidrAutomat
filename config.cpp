/*
 * MinimaSlidrAutomat - Configuration Implementation
 * v2.0 - Production Release
 * 
 * Global variables for runtime calculated values
 */

#include "config.h"

// ========== CALCULATED VALUES (Set at runtime) ==========
// These are calculated automatically during dual sensor homing
float measuredTravelMM = 0.0f;        // Actual distance between sensors
float pingPongStartMM = 0.0f;          // Calculated start position (SENSOR_CLEARANCE_MM)
float pingPongEndMM = 0.0f;            // Calculated end position (measuredTravelMM - SENSOR_CLEARANCE_MM)