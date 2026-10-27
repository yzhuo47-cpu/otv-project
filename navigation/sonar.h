// sonar.h - HC-SR04 ultrasonic sensor with a median filter.
#pragma once
#include <Arduino.h>

namespace sonar {

void begin();

// One raw ping in cm, or SONAR_NO_ECHO (-1) if nothing came back in time.
float pingCm();

// Median of SONAR_SAMPLES pings (ignores single bad readings).
// Returns SONAR_NO_ECHO only if most pings got no echo.
float readCm();

// true if something is closer than limitCm (no echo counts as "clear").
bool blocked(float limitCm);

}  // namespace sonar
