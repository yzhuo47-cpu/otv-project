// =====================================================================
//  ultrasonic_calibration.ino  -  HC-SR04 calibration (MS5 item 8:
//  "sensors used for obstacle avoidance have been calibrated")
//  Arduino UNO Rev3.  Serial Monitor: 115200 baud, line ending "Newline".
//
//  What it does
//    You put a flat target at distances you measured with a tape. At each
//    distance the sketch takes 30 readings. Then it fits a straight line
//        true_cm = SCALE * measured_cm + OFFSET
//    and prints a before/after error table (for the demo) and the two
//    numbers to paste into navigation/config.h, so the navigation code
//    uses the calibrated distances.
//
//  Steps
//    1. Wire: VCC -> 5V, GND -> GND, TRIG -> TRIG_PIN, ECHO -> ECHO_PIN.
//    2. Hold a flat target (a box or an obstacle block) square to the sensor.
//       Measure from the FRONT of the sensor (the two silver cans).
//    3. Type the distance in cm and press Enter. Do this for about 6
//       distances between 10 and 100 cm (the OTV uses 15 - 100 cm).
//    4. Type  c  -> calibration table and the numbers for config.h.
//    5. Type  v  -> live calibrated readings; press Enter to stop.
//       Demo: put the target anywhere, compare with the tape measure.
//    Other:  u = undo last point,  r = start over,  h = help
// =====================================================================
#include <Arduino.h>
#include <stdlib.h>

// ---- pins: placeholders until the new pin map is final (keep = config.h)
const uint8_t TRIG_PIN = 2;
const uint8_t ECHO_PIN = 4;

const uint8_t       SAMPLES       = 30;     // readings per distance
const uint16_t      PING_GAP_MS   = 60;     // HC-SR04 needs ~60 ms between pings
const unsigned long TIMEOUT_US    = 30000;  // ~5 m; no echo -> reading thrown away
const float         OUTLIER_CM    = 3.0;    // ignore readings this far from the median
const float         NAV_MARGIN_CM = 15.0;   // BLOCKED_MARGIN in the navigation code
const uint8_t       MAX_POINTS    = 12;

float   trueCm[MAX_POINTS];   // tape measure
float   measCm[MAX_POINTS];   // sensor (average of the good readings)
float   stdCm[MAX_POINTS];    // how much the readings jumped around
uint8_t usedN[MAX_POINTS];    // how many of the 30 readings were used
uint8_t nPoints = 0;

float scaleK = 1.0;           // current calibration (1 and 0 = none yet)
float offsetCm = 0.0;
bool  fitted = false;

char line[16];
uint8_t lineLen = 0;
unsigned long lastCharMs = 0;

// ---------------------------------------------------------------------
// Same conversion as sonar.cpp in the navigation code, so the fitted
// numbers can be used there directly.
float pingCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);
  if (us == 0) return -1;            // no echo
  return us * 0.0343 / 2.0;
}

float calibrated(float raw) { return scaleK * raw + offsetCm; }

void sortFloats(float* a, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    float v = a[i];
    int8_t j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
}

// Prints a number right-aligned in `width` characters.
void printNum(float v, int8_t width, uint8_t decimals) {
  char buf[16];
  dtostrf(v, width, decimals, buf);
  Serial.print(buf);
}

void printSigned(float v, int8_t width, uint8_t decimals) {
  char buf[16];
  dtostrf(v, 0, decimals, buf);
  char out[18];
  out[0] = (v >= 0) ? '+' : 0;
  out[1] = 0;
  strcat(out, buf);
  for (int8_t pad = width - (int8_t)strlen(out); pad > 0; pad--) Serial.print(' ');
  Serial.print(out);
}

// ---------------------------------------------------------------------
// Take SAMPLES readings. Result = average of the readings within
// OUTLIER_CM of the median (one bad echo can't spoil the point).
bool measure(float& mean, float& sd, uint8_t& used) {
  float s[SAMPLES];
  uint8_t n = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) {
    float d = pingCm();
    if (d > 0) s[n++] = d;
    delay(PING_GAP_MS);
  }
  if (n < SAMPLES / 2) return false;   // mostly no echo: target missing / too far
  sortFloats(s, n);
  float med = s[n / 2];
  float sum = 0;
  used = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (fabs(s[i] - med) <= OUTLIER_CM) { sum += s[i]; used++; }
  }
  mean = sum / used;
  float ss = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (fabs(s[i] - med) <= OUTLIER_CM) ss += (s[i] - mean) * (s[i] - mean);
  }
  sd = (used > 1) ? sqrt(ss / (used - 1)) : 0;
  return true;
}

void addPoint(float tapeCm) {
  if (nPoints >= MAX_POINTS) { Serial.println(F("Table full - type c, or r to start over.")); return; }
  Serial.print(F("Measuring at "));
  Serial.print(tapeCm, 1);
  Serial.println(F(" cm - keep the target still..."));
  float mean, sd;
  uint8_t used;
  if (!measure(mean, sd, used)) {
    Serial.println(F("  Not enough echoes. Is the target square to the sensor and within ~4 m?"));
    return;
  }
  trueCm[nPoints] = tapeCm;
  measCm[nPoints] = mean;
  stdCm[nPoints] = sd;
  usedN[nPoints] = used;
  nPoints++;
  Serial.print(F("  Point "));
  Serial.print(nPoints);
  Serial.print(F(": sensor says "));
  Serial.print(mean, 1);
  Serial.print(F(" cm (spread +/- "));
  Serial.print(sd, 1);
  Serial.print(F(" cm, "));
  Serial.print(used);
  Serial.print(F("/"));
  Serial.print(SAMPLES);
  Serial.println(F(" readings used)"));
  if (sd > 1.5) Serial.println(F("  Readings jumpy - target not flat or not square? Type u to redo."));
  if (nPoints >= 2) Serial.println(F("  Add more distances, or type c to calculate."));
}

// Least-squares straight line  true = scaleK * measured + offsetCm.
bool fitLine() {
  if (nPoints < 2) { Serial.println(F("Need at least 2 distances (better 5-6).")); return false; }
  float mx = 0, my = 0;
  for (uint8_t i = 0; i < nPoints; i++) { mx += measCm[i]; my += trueCm[i]; }
  mx /= nPoints;
  my /= nPoints;
  float sxy = 0, sxx = 0, syy = 0;
  for (uint8_t i = 0; i < nPoints; i++) {
    sxy += (measCm[i] - mx) * (trueCm[i] - my);
    sxx += (measCm[i] - mx) * (measCm[i] - mx);
    syy += (trueCm[i] - my) * (trueCm[i] - my);
  }
  if (sxx < 1e-3) { Serial.println(F("Distances too close together - spread them out (10 to 100 cm).")); return false; }
  scaleK = sxy / sxx;
  offsetCm = my - scaleK * mx;
  fitted = true;
  return true;
}

void report() {
  if (!fitLine()) return;
  Serial.println();
  Serial.println(F("=== HC-SR04 calibration ==="));
  Serial.println(F(" tape(cm)  sensor(cm)  +/-   error   calibrated  error   used"));
  float maxBefore = 0, maxAfter = 0;
  for (uint8_t i = 0; i < nPoints; i++) {
    float errRaw = measCm[i] - trueCm[i];
    float cal = calibrated(measCm[i]);
    float errCal = cal - trueCm[i];
    if (fabs(errRaw) > maxBefore) maxBefore = fabs(errRaw);
    if (fabs(errCal) > maxAfter) maxAfter = fabs(errCal);
    printNum(trueCm[i], 8, 1);
    printNum(measCm[i], 11, 1);
    printNum(stdCm[i], 7, 1);
    printSigned(errRaw, 8, 1);
    printNum(cal, 12, 1);
    printSigned(errCal, 8, 1);
    Serial.print(F("   "));
    Serial.print(usedN[i]);
    Serial.print('/');
    Serial.println(SAMPLES);
  }
  Serial.println();
  Serial.print(F("Calibration line: true = "));
  Serial.print(scaleK, 4);
  Serial.print(F(" x sensor "));
  if (offsetCm >= 0) Serial.print(F("+ "));
  else Serial.print(F("- "));
  Serial.print(fabs(offsetCm), 2);
  Serial.println(F(" cm"));
  Serial.print(F("Largest error: before "));
  Serial.print(maxBefore, 1);
  Serial.print(F(" cm, after "));
  Serial.print(maxAfter, 1);
  Serial.print(F(" cm  (navigation needs well under "));
  Serial.print(NAV_MARGIN_CM, 0);
  Serial.print(F(" cm) -> "));
  Serial.println(maxAfter < NAV_MARGIN_CM / 3 ? F("OK") : F("CHECK SETUP"));
  Serial.println();
  Serial.println(F("Paste into navigation/config.h:"));
  Serial.print(F("#define SONAR_SCALE      "));
  Serial.print(scaleK, 4);
  Serial.println('f');
  Serial.print(F("#define SONAR_OFFSET_CM  "));
  Serial.print(offsetCm, 2);
  Serial.println('f');
  Serial.println();
}

// Live readings (median of 3) until a key is pressed.
void live() {
  if (!fitted) Serial.println(F("(No calibration yet - showing raw values. Type c first.)"));
  Serial.println(F("Live readings - press Enter to stop."));
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {
    float s[3];
    for (uint8_t i = 0; i < 3; i++) { s[i] = pingCm(); delay(PING_GAP_MS); }
    sortFloats(s, 3);
    float raw = s[1];
    if (raw < 0) {
      Serial.println(F("  no echo"));
    } else {
      Serial.print(F("  sensor "));
      printNum(raw, 6, 1);
      Serial.print(F(" cm   calibrated "));
      printNum(calibrated(raw), 6, 1);
      Serial.println(F(" cm"));
    }
    delay(150);
  }
  delay(20);
  while (Serial.available()) Serial.read();
  Serial.println(F("Stopped."));
}

void help() {
  Serial.println();
  Serial.println(F("=== HC-SR04 calibration ==="));
  Serial.println(F(" <number> : target is this many cm from the sensor front -> measure"));
  Serial.println(F(" c        : calculate calibration + print table"));
  Serial.println(F(" v        : live calibrated readings (Enter stops)"));
  Serial.println(F(" u        : undo last point     r : start over     h : help"));
  Serial.println(F("Tip: about 6 distances between 10 and 100 cm, target flat and square."));
  Serial.println();
}

// Reads one line from Serial. Works with "Newline", "Both NL & CR" and
// "No line ending" (a pause of 300 ms ends the line).
bool readLine() {
  while (Serial.available()) {
    char ch = Serial.read();
    lastCharMs = millis();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[lineLen] = 0;
      lineLen = 0;
      return true;
    }
    if (lineLen < sizeof(line) - 1) line[lineLen++] = ch;
  }
  if (lineLen > 0 && millis() - lastCharMs > 300) {
    line[lineLen] = 0;
    lineLen = 0;
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  help();
}

void loop() {
  if (!readLine()) return;
  char* p = line;
  while (*p == ' ') p++;
  if (*p == 0) return;
  char cmd = *p;
  if ((cmd >= '0' && cmd <= '9') || cmd == '.') {
    float d = atof(p);
    if (d < 2 || d > 400) Serial.println(F("Distance must be between 2 and 400 cm."));
    else addPoint(d);
  } else if (cmd == 'c' || cmd == 'C') {
    report();
  } else if (cmd == 'v' || cmd == 'V') {
    live();
  } else if (cmd == 'u' || cmd == 'U') {
    if (nPoints > 0) { nPoints--; Serial.println(F("Last point removed.")); }
  } else if (cmd == 'r' || cmd == 'R') {
    nPoints = 0;
    scaleK = 1.0;
    offsetCm = 0.0;
    fitted = false;
    Serial.println(F("All points cleared."));
  } else if (cmd == 'h' || cmd == 'H') {
    help();
  } else {
    Serial.println(F("Unknown command - type h for help."));
  }
}
