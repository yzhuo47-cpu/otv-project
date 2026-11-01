// =====================================================================
//  Ultrasonic_Calibration.ino  -  HC-SR04 calibration (MS5 sub-task 8)
//  Arduino UNO Rev3.  Serial Monitor: 9600 baud, line ending "New Line".
//
//  1. Upload, open the Serial Monitor.
//  2. Put a flat box in front of the sensor, square to it. Measure the
//     distance with a tape from the front of the sensor (the two silver
//     cans), type it in cm and press Enter. Do this at 5 distances
//     spread out over the range you care about, e.g. 10 - 50 cm.
//  3. It prints the result: tape vs sensor vs calibrated, the calibration
//     line, and the largest error before and after calibration.
//  4. Check it: put the box at a distance you did NOT use, type it and
//     press Enter -> it shows the error before and after calibration.
//     Repeat as often as you like.   RESET = calibrate again.
// =====================================================================
#include <Arduino.h>

// Placeholders until the new pin map is final - use the OTV's real pins
const uint8_t TRIG_PIN = 2;
const uint8_t ECHO_PIN = 4;

const uint8_t N = 5;                  // how many distances to measure
float tapeCm[N];                      // what you typed (tape measure)
float sensorCm[N];                    // what the sensor read there
float scaleK = 1.0, offsetCm = 0.0;   // true = scaleK * sensor + offsetCm
bool calibrated = false;              // true once the line is worked out

// One ping -> distance in cm (same conversion as sonar.cpp). -1 = no echo.
float pingCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL);  // give up after ~5 m
  if (us == 0) return -1;
  return us * 0.0343 / 2.0;
}

// Median of `count` pings (count <= 20). The median ignores the odd bad
// echo. Returns -1 if most pings got no echo.
float readCm(uint8_t count) {
  float s[20];
  uint8_t n = 0;
  for (uint8_t i = 0; i < count; i++) {
    float d = pingCm();
    if (d > 0) s[n++] = d;
    delay(60);                                 // HC-SR04 needs ~60 ms between pings
  }
  if (n < count / 2) return -1;
  for (uint8_t i = 1; i < n; i++)              // sort small to large
    for (uint8_t j = i; j > 0 && s[j - 1] > s[j]; j--) {
      float t = s[j]; s[j] = s[j - 1]; s[j - 1] = t;
    }
  return s[n / 2];
}

// Waits for a number typed in the Serial Monitor (e.g. 25 or 25.5).
float readNumber() {
  while (Serial.available()) Serial.read();    // throw away old input
  while (!Serial.available()) {}               // wait for typing
  float v = Serial.parseFloat();
  delay(50);
  while (Serial.available()) Serial.read();    // drop the rest of the line
  return v;
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println(F("=== HC-SR04 calibration ==="));

  // 1. You type the tape distance, the sensor measures there
  float nearest = 1000, farthest = 0;
  for (uint8_t i = 0; i < N; ) {
    Serial.print(i + 1);
    Serial.print('/');
    Serial.print(N);
    Serial.println(F("  Put the box down, type its distance in cm, press Enter:"));
    float d = readNumber();
    if (d < 2 || d > 400) {
      Serial.println(F("   Type a number between 2 and 400."));
      continue;
    }
    float s = readCm(20);
    if (s < 0) {
      Serial.println(F("   No echo - is the box square to the sensor? Try again."));
      continue;
    }
    tapeCm[i] = d;
    sensorCm[i] = s;
    if (d < nearest) nearest = d;
    if (d > farthest) farthest = d;
    Serial.print(F("   tape "));
    Serial.print(d, 1);
    Serial.print(F(" cm, sensor reads "));
    Serial.print(s, 1);
    Serial.println(F(" cm"));
    i++;
  }
  if (farthest - nearest < 20) {
    Serial.println(F("Distances too close together - spread them out (e.g. 10 to 50 cm). Press RESET to redo."));
    return;
  }

  // 2. Best straight line through the points (least squares)
  float mx = 0, my = 0;
  for (uint8_t i = 0; i < N; i++) { mx += sensorCm[i]; my += tapeCm[i]; }
  mx /= N;
  my /= N;
  float sxy = 0, sxx = 0;
  for (uint8_t i = 0; i < N; i++) {
    sxy += (sensorCm[i] - mx) * (tapeCm[i] - my);
    sxx += (sensorCm[i] - mx) * (sensorCm[i] - mx);
  }
  scaleK = sxy / sxx;
  offsetCm = my - scaleK * mx;

  // 3. Result: tape vs sensor vs calibrated at every point
  Serial.println();
  Serial.println(F("tape\tsensor\tcalibrated   (cm)"));
  float worstBefore = 0, worstAfter = 0;
  for (uint8_t i = 0; i < N; i++) {
    float cal = scaleK * sensorCm[i] + offsetCm;
    if (fabs(sensorCm[i] - tapeCm[i]) > worstBefore) worstBefore = fabs(sensorCm[i] - tapeCm[i]);
    if (fabs(cal - tapeCm[i]) > worstAfter) worstAfter = fabs(cal - tapeCm[i]);
    Serial.print(tapeCm[i], 1);
    Serial.print('\t');
    Serial.print(sensorCm[i], 1);
    Serial.print('\t');
    Serial.println(cal, 1);
  }
  Serial.print(F("Calibration: true = "));
  Serial.print(scaleK, 4);
  Serial.print(offsetCm < 0 ? F(" x sensor - ") : F(" x sensor + "));
  Serial.print(fabs(offsetCm), 2);
  Serial.println(F(" cm"));
  Serial.print(F("Largest error: before "));
  Serial.print(worstBefore, 1);
  Serial.print(F(" cm, after "));
  Serial.print(worstAfter, 1);
  Serial.println(F(" cm"));
  if (worstAfter > 2) Serial.println(F("A point is off by more than 2 cm (box moved? typo?) - press RESET and redo."));

  calibrated = true;
  Serial.println();
  Serial.println(F("Now check it at distances you did NOT use above."));
}

// 4. Check at a new distance: you type the tape distance, it shows the
//    error before and after calibration. Waits for you every time.
void loop() {
  if (!calibrated) return;                     // calibration failed: press RESET
  Serial.println(F("Check: put the box at a new distance, type it in cm, press Enter:"));
  float d = readNumber();
  if (d < 2 || d > 400) {
    Serial.println(F("   Type a number between 2 and 400."));
    return;
  }
  float s = readCm(20);
  if (s < 0) {
    Serial.println(F("   No echo - is the box square to the sensor? Try again."));
    return;
  }
  float cal = scaleK * s + offsetCm;
  Serial.print(F("   tape "));
  Serial.print(d, 1);
  Serial.print(F(" cm:  sensor "));
  Serial.print(s, 1);
  Serial.print(F(" (error "));
  Serial.print(fabs(s - d), 1);
  Serial.print(F(")  ->  calibrated "));
  Serial.print(cal, 1);
  Serial.print(F(" (error "));
  Serial.print(fabs(cal - d), 1);
  Serial.println(F(")"));
}
