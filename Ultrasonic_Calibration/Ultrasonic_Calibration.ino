// Ultrasonic_Calibration.ino - HC-SR04 calibration (ENES100 MS5 sub-task 8)
// Arduino UNO (R3 or R4 WiFi), Serial Monitor at 9600 baud. How to use: README.md
#include <Arduino.h>

// Sensor pins (placeholders - change to the OTV's pin map)
const uint8_t TRIG_PIN = 2;           // Arduino -> sensor: start a measurement
const uint8_t ECHO_PIN = 4;           // sensor -> Arduino: echo time

// Calibration points and result
const uint8_t N = 5;                  // number of calibration points
float tapeCm[N];                      // ruler distances (typed in)
float sensorCm[N];                    // sensor readings at those distances
float scaleK = 1.0, offsetCm = 0.0;   // calibration line: true = scaleK * sensor + offsetCm
bool calibrated = false;              // true once the line is calculated

// Obstacle threshold (same 15 cm as our navigation code)
const float OBSTACLE_CM = 15.0;       // closer than this = obstacle

// Measures the distance once, in cm. Returns -1 if there is no echo.
float pingCm() {
  // a 10 us pulse on TRIG makes the sensor send out sound
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // time until the echo comes back, in microseconds (0 = no echo within 30 ms)
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (us == 0) return -1;
  // time -> cm: speed of sound 0.0343 cm/us, divided by 2 for there and back
  return us * 0.0343 / 2.0;
}

// Measures `count` times (max 20) and returns the median (middle value),
// so one bad reading doesn't matter. Returns -1 if most got no echo.
float readCm(uint8_t count) {
  float s[20];
  uint8_t n = 0;
  for (uint8_t i = 0; i < count; i++) {
    float d = pingCm();
    if (d > 0) s[n++] = d;                     // keep readings that got an echo
    delay(60);                                 // wait for the old echo to fade
  }
  if (n < count / 2) return -1;
  for (uint8_t i = 1; i < n; i++)              // sort small to large
    for (uint8_t j = i; j > 0 && s[j - 1] > s[j]; j--) {
      float t = s[j]; s[j] = s[j - 1]; s[j - 1] = t;
    }
  return s[n / 2];                             // middle value
}

// Waits for you to type a number in the Serial Monitor and press Enter.
float readNumber() {
  while (Serial.available()) Serial.read();    // clear old input
  while (!Serial.available()) {}               // wait for typing
  float v = Serial.parseFloat();
  delay(50);
  while (Serial.available()) Serial.read();    // clear the rest of the line
  return v;
}

// setup() runs once: calibrates the sensor against the ruler (steps 1-3)
void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println(F("=== HC-SR04 calibration ==="));  // F() keeps text in flash to save RAM

  // STEP 1: get 5 points - you type the ruler distance, the sensor measures
  float nearest = 1000, farthest = 0;          // to check the points are spread out
  for (uint8_t i = 0; i < N; ) {               // i goes up only when a point is OK
    Serial.print(i + 1);
    Serial.print('/');
    Serial.print(N);
    Serial.println(F("  Put the box down, type its distance in cm, press Enter:"));
    float d = readNumber();
    if (d < 2 || d > 400) {                    // outside the sensor's 2-400 cm range -> ask again
      Serial.println(F("   Type a number between 2 and 400."));
      continue;
    }
    float s = readCm(20);                      // sensor reading (median of 20)
    if (s < 0) {                               // no echo -> ask again
      Serial.println(F("   No echo - is the box square to the sensor? Try again."));
      continue;
    }
    // save the point and show it
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
  // stop if the points are less than 20 cm apart (the line would be unreliable)
  if (farthest - nearest < 20) {
    Serial.println(F("Distances too close together - spread them out (e.g. 10 to 50 cm). Press RESET to redo."));
    return;
  }

  // STEP 2: best straight line through the 5 points (least squares)
  float mx = 0, my = 0;                        // averages of sensor and ruler values
  for (uint8_t i = 0; i < N; i++) { mx += sensorCm[i]; my += tapeCm[i]; }
  mx /= N;
  my /= N;
  float sxy = 0, sxx = 0;                      // sums for the least-squares formula
  for (uint8_t i = 0; i < N; i++) {
    sxy += (sensorCm[i] - mx) * (tapeCm[i] - my);
    sxx += (sensorCm[i] - mx) * (sensorCm[i] - mx);
  }
  scaleK = sxy / sxx;                          // SCALE (slope)
  offsetCm = my - scaleK * mx;                 // OFFSET (intercept)

  // STEP 3: print ruler / sensor / calibrated for each point,
  //         the calibration line, and the largest error before / after
  Serial.println();
  Serial.println(F("tape\tsensor\tcalibrated   (cm)"));
  float worstBefore = 0, worstAfter = 0;       // largest error before / after
  for (uint8_t i = 0; i < N; i++) {
    float cal = scaleK * sensorCm[i] + offsetCm;  // calibrated reading
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
  // warn if a point doesn't fit the line (box moved or typo)
  if (worstAfter > 2) Serial.println(F("A point is off by more than 2 cm (box moved? typo?) - press RESET and redo."));

  calibrated = true;
  Serial.println();
  Serial.println(F("Now check it at distances you did NOT use above."));
}

// loop() repeats: STEP 4, check the calibration at a new distance
// (a distance not used to make the line = a fair test),
// and report an obstacle if it is closer than 15 cm
void loop() {
  if (!calibrated) return;                     // calibration failed -> do nothing
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
  // show the error before and after calibration at this distance
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
  // obstacle check: closer than 15 cm counts as an obstacle
  if (cal < OBSTACLE_CM) {
    Serial.print(F("   -> OBSTACLE detected (closer than "));
    Serial.print(OBSTACLE_CM, 0);
    Serial.println(F(" cm)"));
  }
}
