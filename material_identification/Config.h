#pragma once
#include "Arduino.h"

// 1 = connect to the vision system on startup and send results.
// 0 = bench-test with the Serial Monitor only (no Wi-Fi connection needed).
#define USE_VISION 1

#define TEAM_NAME   "The Harvesters"
#define MARKER_ID   3        // set to your team's ArUco marker number
#define ROOM_NUMBER 1116

// TCRT5000 analog output
const int IR_PIN = A0;

// Sampling
const int NUM_SAMPLES     = 25;  // readings averaged per sample
const int SAMPLE_DELAY_MS = 4;   // ms between readings

// Minimum gap between the two reference readings (0-1023 scale)
const int MIN_SEPARATION = 40;