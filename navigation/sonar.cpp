#include "sonar.h"
#include "config.h"

namespace sonar {

void begin() {
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
  digitalWrite(SONAR_TRIG_PIN, LOW);
}

float pingCm() {
  digitalWrite(SONAR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_PIN, LOW);

  // The timeout matters: without it pulseIn() waits a full second when
  // nothing is in front of the sensor and the whole robot freezes.
  unsigned long us = pulseIn(SONAR_ECHO_PIN, HIGH, SONAR_TIMEOUT_US);
  if (us == 0) return SONAR_NO_ECHO;
  return us * 0.0343f / 2.0f;  // speed of sound 343 m/s, there and back
}

float readCm() {
  float s[SONAR_SAMPLES];
  for (uint8_t i = 0; i < SONAR_SAMPLES; i++) {
    unsigned long t0 = millis();
    s[i] = pingCm();
    while (millis() - t0 < SONAR_PING_GAP_MS) { /* let old echoes die out */ }
  }
  // Insertion sort (tiny array). SONAR_NO_ECHO (-1) sorts to the front.
  for (uint8_t i = 1; i < SONAR_SAMPLES; i++) {
    float v = s[i];
    int8_t j = i - 1;
    while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; }
    s[j + 1] = v;
  }
  return s[SONAR_SAMPLES / 2];
}

bool blocked(float limitCm) {
  float d = readCm();
  return d != SONAR_NO_ECHO && d < limitCm;
}

}  // namespace sonar
