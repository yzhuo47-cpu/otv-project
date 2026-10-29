// =====================================================================
//  ir_calibration.ino  -  TCRT5000 calibration for the seed mission
//  (MS5 item 9: "sensors used for a mission-specific objective have been
//  calibrated"). Works on the UNO R4 WiFi and the UNO Rev3.
//  Serial Monitor: 115200 baud, line ending "New Line".
//
//  What it does
//    Records IR readings over orzo (plantable) and over rocks
//    (non-plantable), at the height the IR arm holds the sensor. Then it
//    shows how far apart the two materials read, the dividing value, and
//    whether every recorded reading would be classified correctly. Test
//    mode checks new readings and keeps a score for the demo.
//    Readings are taken exactly like material_identification/IRSensor.cpp
//    (average of 25 analogReads), and classified the same way (closer
//    reference wins), so the results apply to the mission code directly.
//
//  Steps
//    1. Hold the sensor at the IR-arm height above the substrate (use a
//       fixed stand - the reading changes a lot with height).
//    2. Over orzo: type  p  + Enter. Move to a NEW spot, repeat ~8 times.
//    3. Over rocks: type  n  + Enter. Move to a new spot, repeat ~8 times.
//    4. Type  c  -> calibration table and the numbers for Config.h.
//    5. Demo / check:  tp = test over orzo,  tn = test over rocks
//       (each prints the result and keeps a score),  v = live readings.
//    Other:  u = undo last sample,  r = start over,  h = help
//    Tip: repeat the tests under bright light (phone flashlight) too.
// =====================================================================
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

// ---- same settings as material_identification/Config.h ----
const int IR_PIN          = A0;
const int NUM_SAMPLES     = 25;   // analogReads averaged per reading
const int SAMPLE_DELAY_MS = 4;
const int MIN_SEPARATION  = 40;   // the mission code refuses references closer than this

const uint8_t MAX_PER_CLASS = 20;

int orzo[MAX_PER_CLASS];   uint8_t nOrzo = 0;   // plantable
int rocks[MAX_PER_CLASS];  uint8_t nRocks = 0;  // non-plantable
char history[2 * MAX_PER_CLASS];                // 'p' / 'n' in the order recorded (for undo)
uint8_t nHistory = 0;

uint8_t testsRun = 0, testsRight = 0;

char line[16];
uint8_t lineLen = 0;
unsigned long lastCharMs = 0;

// ---------------------------------------------------------------------
// Same as irRead() in IRSensor.cpp.
int irRead() {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(IR_PIN);
    delay(SAMPLE_DELAY_MS);
  }
  return (int)(sum / NUM_SAMPLES);
}

float mean(const int* v, uint8_t n) {
  float s = 0;
  for (uint8_t i = 0; i < n; i++) s += v[i];
  return n ? s / n : 0;
}

float stdDev(const int* v, uint8_t n) {
  if (n < 2) return 0;
  float m = mean(v, n), ss = 0;
  for (uint8_t i = 0; i < n; i++) ss += (v[i] - m) * (v[i] - m);
  return sqrt(ss / (n - 1));
}

int minOf(const int* v, uint8_t n) { int m = v[0]; for (uint8_t i = 1; i < n; i++) if (v[i] < m) m = v[i]; return m; }
int maxOf(const int* v, uint8_t n) { int m = v[0]; for (uint8_t i = 1; i < n; i++) if (v[i] > m) m = v[i]; return m; }

bool haveBoth() { return nOrzo >= 3 && nRocks >= 3; }

// Same rule as irIsPlantable() in IRSensor.cpp, with the averages as references.
bool isPlantable(int reading) {
  float po = mean(orzo, nOrzo), pr = mean(rocks, nRocks);
  return fabs(reading - po) < fabs(reading - pr);
}

void printNum(float v, int8_t width, uint8_t decimals) {
  char buf[16];
  dtostrf(v, width, decimals, buf);
  Serial.print(buf);
}

// ---------------------------------------------------------------------
void addSample(char cls) {
  int* arr = (cls == 'p') ? orzo : rocks;
  uint8_t& n = (cls == 'p') ? nOrzo : nRocks;
  if (n >= MAX_PER_CLASS) { Serial.println(F("That list is full - type c, or r to start over.")); return; }
  int v = irRead();
  arr[n++] = v;
  history[nHistory++] = cls;
  Serial.print(cls == 'p' ? F("  orzo  sample ") : F("  rocks sample "));
  Serial.print(n);
  Serial.print(F(": "));
  Serial.print(v);
  Serial.println(F("   (move to a new spot before the next one)"));
}

void undo() {
  if (nHistory == 0) { Serial.println(F("Nothing to undo.")); return; }
  char cls = history[--nHistory];
  if (cls == 'p') nOrzo--; else nRocks--;
  Serial.println(cls == 'p' ? F("Last orzo sample removed.") : F("Last rocks sample removed."));
}

void printRow(const __FlashStringHelper* name, const int* v, uint8_t n) {
  Serial.print(name);
  printNum(n, 4, 0);
  printNum(mean(v, n), 9, 1);
  printNum(stdDev(v, n), 8, 1);
  printNum(minOf(v, n), 7, 0);
  printNum(maxOf(v, n), 7, 0);
  Serial.println();
}

void report() {
  if (!haveBoth()) { Serial.println(F("Need at least 3 orzo (p) and 3 rocks (n) samples - better 8 each.")); return; }
  float mo = mean(orzo, nOrzo), mr = mean(rocks, nRocks);
  bool orzoHigher = mo > mr;
  int oMin = minOf(orzo, nOrzo), oMax = maxOf(orzo, nOrzo);
  int rMin = minOf(rocks, nRocks), rMax = maxOf(rocks, nRocks);
  int gap = orzoHigher ? (oMin - rMax) : (rMin - oMax);   // space between the closest readings
  float threshold = (mo + mr) / 2;                         // where "closer reference" flips
  float separation = fabs(mo - mr);

  // How close does any recorded reading come to the dividing value?
  float margin = 1e9;
  uint8_t right = 0;
  for (uint8_t i = 0; i < nOrzo; i++) { margin = min(margin, (float)fabs(orzo[i] - threshold)); if (isPlantable(orzo[i])) right++; }
  for (uint8_t i = 0; i < nRocks; i++) { margin = min(margin, (float)fabs(rocks[i] - threshold)); if (!isPlantable(rocks[i])) right++; }

  Serial.println();
  Serial.println(F("=== TCRT5000 calibration: orzo vs rocks (0-1023) ==="));
  Serial.println(F("               n     mean     std    min    max"));
  printRow(F("orzo  (p)  "), orzo, nOrzo);
  printRow(F("rocks (n)  "), rocks, nRocks);
  Serial.println();
  Serial.println(orzoHigher ? F("Orzo reads HIGHER than rocks.") : F("Orzo reads LOWER than rocks."));
  Serial.print(F("Difference of the averages: "));
  Serial.print(separation, 1);
  Serial.print(F("  (mission code needs >= "));
  Serial.print(MIN_SEPARATION);
  Serial.println(F(")"));
  Serial.print(F("Gap between the closest orzo and rocks readings: "));
  Serial.println(gap);
  Serial.print(F("Dividing value: "));
  Serial.print(threshold, 1);
  Serial.println(orzoHigher ? F("  (above = orzo/plantable, below = rocks)") : F("  (below = orzo/plantable, above = rocks)"));
  Serial.print(F("Recorded samples classified correctly: "));
  Serial.print(right);
  Serial.print('/');
  Serial.println(nOrzo + nRocks);
  Serial.print(F("Closest any sample came to the dividing value: "));
  Serial.println(margin, 1);

  bool ok = gap > 0 && separation >= MIN_SEPARATION && right == nOrzo + nRocks;
  if (ok) {
    Serial.println(F("-> OK: every sample is classified correctly at this height."));
  } else {
    if (right < nOrzo + nRocks) Serial.println(F("-> NOT RELIABLE: some recorded samples would be classified wrong."));
    else if (gap <= 0)          Serial.println(F("-> NOT RELIABLE: orzo and rocks readings overlap."));
    else                        Serial.println(F("-> NOT RELIABLE: the averages are closer than the mission code allows."));
    Serial.println(F("   Try: a fixed, lower sensor height; shade the sensor from room light;"));
    Serial.println(F("   check that the sensor points straight down; then r and record again."));
    Serial.println();
    return;   // don't offer numbers for Config.h from an unreliable calibration
  }
  Serial.println();
  Serial.println(F("Paste into material_identification/Config.h (default references,"));
  Serial.println(F("used until the robot reads the near plots at the mission site):"));
  Serial.print(F("const int IR_PLANTABLE_DEFAULT    = "));
  Serial.print((int)(mo + 0.5));
  Serial.println(';');
  Serial.print(F("const int IR_NONPLANTABLE_DEFAULT = "));
  Serial.print((int)(mr + 0.5));
  Serial.println(';');
  Serial.println();
}

// Test with a known material: classify one reading and keep score.
void test(bool truthPlantable) {
  if (!haveBoth()) { Serial.println(F("Calibrate first: at least 3 p and 3 n samples.")); return; }
  int v = irRead();
  bool saysPlantable = isPlantable(v);
  testsRun++;
  bool correct = (saysPlantable == truthPlantable);
  if (correct) testsRight++;
  Serial.print(F("  test "));
  Serial.print(testsRun);
  Serial.print(F(": reading "));
  Serial.print(v);
  Serial.print(saysPlantable ? F(" -> PLANTABLE (orzo)") : F(" -> NON-PLANTABLE (rocks)"));
  Serial.print(correct ? F("   correct") : F("   WRONG"));
  Serial.print(F("   score "));
  Serial.print(testsRight);
  Serial.print('/');
  Serial.println(testsRun);
}

// Live readings until Enter.
void live() {
  if (!haveBoth()) Serial.println(F("(Not calibrated yet - showing raw readings only.)"));
  Serial.println(F("Live readings - press Enter to stop."));
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {
    int v = irRead();
    Serial.print(F("  reading "));
    printNum(v, 5, 0);
    if (haveBoth()) Serial.print(isPlantable(v) ? F("   PLANTABLE (orzo)") : F("   NON-PLANTABLE (rocks)"));
    Serial.println();
    delay(250);
  }
  delay(20);
  while (Serial.available()) Serial.read();
  Serial.println(F("Stopped."));
}

void help() {
  Serial.println();
  Serial.println(F("=== TCRT5000 calibration (orzo vs rocks) ==="));
  Serial.println(F(" p   : record a reading over ORZO   (new spot each time, ~8 times)"));
  Serial.println(F(" n   : record a reading over ROCKS  (new spot each time, ~8 times)"));
  Serial.println(F(" c   : calibration table + numbers for Config.h"));
  Serial.println(F(" tp  : test over orzo     tn : test over rocks   (keeps a score)"));
  Serial.println(F(" v   : live readings (Enter stops)"));
  Serial.println(F(" u   : undo last sample   r : start over   h : help"));
  Serial.println(F("Keep the sensor at the IR-arm height the whole time."));
  Serial.println();
}

// Reads one line. Works with "New Line", "Both NL & CR" and "No Line Ending".
bool readLine() {
  while (Serial.available()) {
    char ch = Serial.read();
    lastCharMs = millis();
    if (ch == '\r') continue;
    if (ch == '\n') { line[lineLen] = 0; lineLen = 0; return true; }
    if (lineLen < sizeof(line) - 1) line[lineLen++] = ch;
  }
  if (lineLen > 0 && millis() - lastCharMs > 300) { line[lineLen] = 0; lineLen = 0; return true; }
  return false;
}

void setup() {
  Serial.begin(115200);
  pinMode(IR_PIN, INPUT);
  help();
}

void loop() {
  if (!readLine()) return;
  char* p = line;
  while (*p == ' ') p++;
  for (char* q = p; *q; q++) if (*q >= 'A' && *q <= 'Z') *q += 'a' - 'A';   // accept capitals
  if (*p == 0) return;
  if (strcmp(p, "p") == 0)       addSample('p');
  else if (strcmp(p, "n") == 0)  addSample('n');
  else if (strcmp(p, "c") == 0)  report();
  else if (strcmp(p, "tp") == 0) test(true);
  else if (strcmp(p, "tn") == 0) test(false);
  else if (strcmp(p, "v") == 0)  live();
  else if (strcmp(p, "u") == 0)  undo();
  else if (strcmp(p, "r") == 0) {
    nOrzo = nRocks = nHistory = 0;
    testsRun = testsRight = 0;
    Serial.println(F("All samples and test scores cleared."));
  }
  else if (strcmp(p, "h") == 0)  help();
  else Serial.println(F("Unknown command - type h for help."));
}
