// =====================================================================
//  Ultrasonic_Calibration.ino - HC-SR04 calibration, Arduino UNO Rev3
//  ENES100 MS5 sub-task 8: "the sensors used for obstacle avoidance have
//  been calibrated"
//
//  WHY: The HC-SR04 measures TIME (how long a sound pulse takes to reach
//  the obstacle and echo back). The code converts that time to cm using
//  the speed of sound. The conversion has two small errors:
//    SCALE  - same PERCENT at every distance: the speed of sound changes
//             with air temperature (about 0.6 m/s per deg C).
//    OFFSET - same AMOUNT at every distance: the sound does not start
//             exactly at the front of the sensor, and the electronics
//             add a small fixed delay.
//  Together they give a straight line:
//        true distance = SCALE x sensor reading + OFFSET
//  We compare the sensor with a ruler at 5 distances and find the SCALE
//  and OFFSET that fit best (least squares). Our navigation treats an
//  obstacle closer than 15 cm as blocked, so an error of about 1 cm is
//  small enough for obstacle avoidance.
//
//  HOW TO USE (Serial Monitor at 9600 baud):
//  1. Put a flat box square to the sensor. Measure from the front of the
//     sensor (the two silver cans) to the box, type the distance in cm,
//     press Enter. Do 5 distances; nearest and farthest at least 20 cm
//     apart (e.g. 10, 15, 20, 25, 30 cm).
//  2. It prints ruler vs sensor vs calibrated, the calibration line, and
//     the largest error before and after calibration.
//  3. Check at NEW distances (not one of the 5): type the distance -> it
//     shows the error before and after calibration. RESET = start over.
// =====================================================================
#include <Arduino.h>

// Wiring: VCC -> 5V, GND -> GND, TRIG and ECHO -> the pins below.
// (Placeholders until the new pin map is final - change to the OTV's pins.)
const uint8_t TRIG_PIN = 2;           // Arduino -> sensor: "send a pulse now"
const uint8_t ECHO_PIN = 4;           // sensor -> Arduino: HIGH while the sound travels

// 5 points: 2 would already define a line, but more points average out
// small ruler mistakes and show that the points really lie on a line.
const uint8_t N = 5;
float tapeCm[N];                      // true distances, typed in from the ruler
float sensorCm[N];                    // sensor readings at those distances
float scaleK = 1.0, offsetCm = 0.0;   // the line: true = scaleK * sensor + offsetCm
bool calibrated = false;              // set to true once the line is calculated

// ---------------------------------------------------------------------
// ONE MEASUREMENT -> distance in cm, or -1 if no echo came back.
// (Same formula as sonar.cpp in our navigation code.)
float pingCm() {
  // A 10 us HIGH pulse on TRIG makes the sensor send a burst of 40 kHz sound.
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // The sensor then holds ECHO HIGH until the echo returns, so the HIGH time
  // is the round-trip time of the sound. pulseIn() measures it in microseconds.
  // Give up after 30 ms (about 5 m, beyond the sensor's 4 m range) so the
  // code never hangs when nothing is in front.
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (us == 0) return -1;
  // Speed of sound 343 m/s (at about 20 deg C) = 0.0343 cm per microsecond.
  // Divide by 2 because the sound travels to the obstacle AND back.
  return us * 0.0343 / 2.0;
}

// ---------------------------------------------------------------------
// MEDIAN OF `count` MEASUREMENTS (count <= 20). Returns -1 if most got no echo.
// Why the median and not the average: now and then the sound bounces off
// something else and one reading is far off. One bad reading pulls the
// average away, but the median (the middle value after sorting) ignores it.
float readCm(uint8_t count) {
  float s[20];
  uint8_t n = 0;                               // number of good readings
  for (uint8_t i = 0; i < count; i++) {
    float d = pingCm();
    if (d > 0) s[n++] = d;                     // keep only readings that got an echo
    delay(60);                                 // let the old echo die out (datasheet: >= 60 ms)
  }
  if (n < count / 2) return -1;
  for (uint8_t i = 1; i < n; i++)              // sort small to large (insertion sort)
    for (uint8_t j = i; j > 0 && s[j - 1] > s[j]; j--) {
      float t = s[j]; s[j] = s[j - 1]; s[j - 1] = t;
    }
  return s[n / 2];                             // middle value = median
}

// ---------------------------------------------------------------------
// WAITS FOR A NUMBER typed in the Serial Monitor (e.g. 25 or 25.5) + Enter.
// Returns 0 if no number was typed (it then gets rejected as less than 2).
float readNumber() {
  while (Serial.available()) Serial.read();    // clear anything typed earlier
  while (!Serial.available()) {}               // wait until something is typed
  float v = Serial.parseFloat();               // read the number
  delay(50);
  while (Serial.available()) Serial.read();    // clear the rest of the line (Enter)
  return v;
}

// =====================================================================
// setup() runs once: collect the 5 points, fit the line, print the results.
void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // F("...") keeps fixed text in flash memory instead of RAM (the Uno has
  // only 2 KB of RAM).
  Serial.println(F("=== HC-SR04 calibration ==="));

  // ---- STEP 1: 5 points of (ruler distance, sensor reading) ----
  float nearest = 1000, farthest = 0;
  for (uint8_t i = 0; i < N; ) {               // i only goes up when a point is accepted
    Serial.print(i + 1);
    Serial.print('/');
    Serial.print(N);
    Serial.println(F("  Put the box down, type its distance in cm, press Enter:"));
    float d = readNumber();
    if (d < 2 || d > 400) {                    // the HC-SR04 is rated for 2 - 400 cm
      Serial.println(F("   Type a number between 2 and 400."));
      continue;                                // ask for the same point again
    }
    float s = readCm(20);                      // median of 20 measurements
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
  // Points bunched close together give an unreliable line: a small ruler
  // mistake tilts it a lot (like drawing a line through two dots that are
  // very close). So nearest and farthest must be at least 20 cm apart.
  if (farthest - nearest < 20) {
    Serial.println(F("Distances too close together - spread them out (e.g. 10 to 50 cm). Press RESET to redo."));
    return;                                    // calibrated stays false -> loop() does nothing
  }

  // ---- STEP 2: best-fit straight line (least squares) ----
  // x = sensor reading, y = true (ruler) distance. Least squares finds the
  // line that makes the sum of the squared gaps between the line and the
  // points as small as possible:
  //   slope     = sxy / sxx,   sxy = sum of (x - mx)(y - my),
  //                            sxx = sum of (x - mx)^2
  //   intercept = my - slope * mx           (mx, my = averages of x and y)
  float mx = 0, my = 0;
  for (uint8_t i = 0; i < N; i++) { mx += sensorCm[i]; my += tapeCm[i]; }
  mx /= N;
  my /= N;
  float sxy = 0, sxx = 0;
  for (uint8_t i = 0; i < N; i++) {
    sxy += (sensorCm[i] - mx) * (tapeCm[i] - my);
    sxx += (sensorCm[i] - mx) * (sensorCm[i] - mx);
  }
  scaleK = sxy / sxx;                          // slope = SCALE
  offsetCm = my - scaleK * mx;                 // intercept = OFFSET

  // ---- STEP 3: print the results ----
  // Each row: ruler distance, raw sensor reading, reading after calibration.
  // Error before = |sensor - ruler|, error after = |calibrated - ruler|.
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
  // After calibration every point should be within about 1 cm. More than
  // 2 cm means one point was wrong (box moved, or a wrong number typed).
  if (worstAfter > 2) Serial.println(F("A point is off by more than 2 cm (box moved? typo?) - press RESET and redo."));

  calibrated = true;
  Serial.println();
  Serial.println(F("Now check it at distances you did NOT use above."));
}

// =====================================================================
// loop() repeats forever. STEP 4: check at NEW distances.
// The errors in the table come from the same 5 points used to make the
// line, so they always look good. A distance that was NOT used is a fair
// test of whether the calibration really works.
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
  float cal = scaleK * s + offsetCm;           // apply the calibration line
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
