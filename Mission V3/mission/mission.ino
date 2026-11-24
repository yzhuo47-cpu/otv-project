#include "Enes100.h"
#include <Servo.h>

// ===================== SETTINGS =====================

// Which part to run (change, then upload again)
// 0 = Calibration: find the servo angles (and re-check the IR) with Serial Monitor commands
// 1 = Front of the site: Mission II (plant seed) + Mission III (collect rocks)
// 2 = Back of the site:  Mission I (read the two far plots + send the result)
const int MODE = 0;

// Vision system (UNO R4 WiFi: Wi-Fi is built in)
const int MARKER_ID = 147;  // number of the ArUco marker on top of the OTV
const int ROOM = 1201;      // arena room: 1201 open lab

// Motor pins (L298N driver)
const int ENA = 10, IN1 = 9, IN2 = 8;   // right side: enable + direction
const int ENB = 5,  IN3 = 7, IN4 = 6;   // left side:  enable + direction

// Mission pins
const int ARM_PIN = A1;     // upper servo: lifts / lowers the arm
const int BUCKET_PIN = A2;  // lower servo: turns the bucket
const int IR_PIN = A0;      // IR reflectance sensor (TCRT5000, analog AO pin)

// Servo angles (0-180). Best guess until MODE 0 prints the real ones: replace these 5 lines.
const int ARM_UP = 75;        // 15 deg above touching: start position, safe to drive
const int ARM_DOWN = 102;     // 12 deg below touching: digs about 2 cm (more hits the wrist servo)
const int BUCKET_HOLD = 90;   // bucket level: start position, holds the bean
const int BUCKET_SCOOP = 150; // scoops substrate; the bean drops out the other side
const int BUCKET_DUMP = 30;   // pours the scooped orzo back on top of the bean

// IR, from our calibration: orzo reads about 19-21, rocks about 23-25
const float IR_THRESHOLD = 21.5;   // between orzo and rocks
const bool ORZO_IS_HIGHER = false; // orzo reads lower than rocks
const float IR_CHECK_ZONE = 0.5;   // this close to the threshold = too close to call: scan again

// Movement
const float PLOT_SPACING_CM = 8.4; // center-to-center distance of two plots

// ====================================================

Servo arm;
Servo bucket;

void setup() {
  Serial.begin(9600);         // with USB in, progress shows in the Serial Monitor
  motorsOff();                // motors can't move by accident

  arm.attach(ARM_PIN);
  bucket.attach(BUCKET_PIN);

  if (MODE == 0) {            // calibration: no Wi-Fi, motors stay off
    calibrationStart();
    return;
  }

  // Arm up, bucket level, before anything else moves
  arm.write(ARM_UP);
  bucket.write(BUCKET_HOLD);

  // begin() returns right away; the library connects in the background.
  // backUp() waits until the position is valid before moving.
  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM);
  delay(3000);                // time to take your hands away

  if (MODE == 1) frontSide();
  if (MODE == 2) missionOne();
}

void loop() {
  if (MODE == 0) calibrationLoop();
  // MODE 1 and 2 run once in setup(). Press RESET to run again.
}

// ================ MISSIONS ================

// MODE 1. Start: OTV at the front of the site, right side facing it,
// bucket above the near-left plot (always orzo).
void frontSide() {
  missionTwo();              // plant the seed in the near-left plot
  Serial.println("Mission II done");
  backUp(PLOT_SPACING_CM);   // move one plot: bucket now above the near-right plot
  missionThree();            // collect rocks from the near-right plot
  Serial.println("Mission III done");
}

// MODE 2. Mission I: find which far plots are plantable and send the pattern.
// Start: OTV at the back of the site, IR above the far plot toward the OTV's front.
// The IR arm only reaches the row closest to the OTV, so we read the far row from the back.
// From the back, left and right are flipped: the first plot is far-RIGHT.
void missionOne() {
  float rightIR = readPlot();   // far-right plot
  backUp(PLOT_SPACING_CM);      // IR now above the far-left plot
  float leftIR = readPlot();    // far-left plot

  bool farRight = isOrzo(rightIR);
  bool farLeft = isOrzo(leftIR);

  // The pattern is described from the seed plot (near-left, always orzo):
  // ADJACENT = the plot right behind it, DIAGONAL = the one across from it
  int plot;
  if (farLeft && farRight) plot = BOTH;
  else if (farLeft)        plot = ADJACENT;
  else if (farRight)       plot = DIAGONAL;
  else                     plot = NEITHER;

  Enes100.mission(LOCATION, plot);   // official result, shown on the vision system page

  // Same result in words, with the IR numbers, so we can check it against the real plots.
  // One println per line: each print call takes a slot in the library's 8-message queue.
  String left = String("far-left: ") + (farLeft ? "orzo" : "rocks") + " (IR " + String(leftIR, 1) + ")";
  String right = String("far-right: ") + (farRight ? "orzo" : "rocks") + " (IR " + String(rightIR, 1) + ")";
  Enes100.println("Seen from the front of the site:");
  Enes100.println(left);
  Enes100.println(right);
  Serial.println(left);
  Serial.println(right);
}

// Mission II: bury the lima bean at least 1 cm deep in the near-left plot.
// That plot is always orzo (official rule), so no sensing is needed.
void missionTwo() {
  moveServo(arm, ARM_DOWN);         // bucket goes into the orzo
  moveServo(bucket, BUCKET_SCOOP);  // scoop out a hole; the bean drops into it
  delay(500);                       // let the bean settle
  moveServo(bucket, BUCKET_DUMP);   // pour the scooped orzo back over the bean
  moveServo(arm, ARM_UP);
  moveServo(bucket, BUCKET_HOLD);
}

// Mission III: collect at least 10 g of rocks from the near-right plot.
// That plot is always rocks (official rule), so no sensing is needed.
void missionThree() {
  moveServo(arm, ARM_DOWN);         // bucket goes into the rocks
  moveServo(bucket, BUCKET_SCOOP);  // scoop rocks
  moveServo(arm, ARM_UP);           // lift; bucket stays tilted so the rocks stay in
}

// ================ IR SENSOR ================

// Orzo and rocks differ by only about 4, so one plot is read carefully:
// one scan first; if it is within IR_CHECK_ZONE of the threshold,
// scan again (up to 4 scans in total) and use the average of all scans.
float readPlot() {
  float total = readIR();
  int scans = 1;
  while (scans < 4 && fabs(total / scans - IR_THRESHOLD) < IR_CHECK_ZONE) {
    total += readIR();
    scans++;
  }
  return total / scans;
}

// One scan = average of 50 samples over 0.5 s at the same spot (removes sensor noise).
// The OTV stands still while reading.
float readIR() {
  long sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogRead(IR_PIN);
    delay(10);
  }
  return sum / 50.0;
}

// Orzo and rocks reflect IR differently, so one threshold separates them
bool isOrzo(float value) {
  if (ORZO_IS_HIGHER) return value > IR_THRESHOLD;
  else                return value < IR_THRESHOLD;
}

// ================ MOVEMENT ================

// Back up a set distance (cm), measured with the vision system.
// The vision system only gives position, so the distance is worked out
// from the start position: sqrt(dx^2 + dy^2).
void backUp(float cm) {
  // Wait for a valid start position (-1 = not connected yet, or marker not seen)
  Serial.println("Waiting for position from the vision system...");
  float x0 = -1, y0 = -1;
  while (x0 < 0 || y0 < 0) {
    x0 = Enes100.getX();
    y0 = Enes100.getY();
    delay(50);
  }

  Serial.println("Backing up");
  float moved = 0;
  unsigned long start = millis();
  while (millis() - start < 15000) {        // safety: give up after 15 s
    float x = Enes100.getX();
    float y = Enes100.getY();
    if (x >= 0 && y >= 0) {                 // skip readings where the marker isn't seen
      float dx = x - x0;
      float dy = y - y0;
      moved = sqrt(dx * dx + dy * dy) * 100;   // meters to cm
      if (moved >= cm) break;
    }
    // Short full-power pulse, then stop: slow and easy to stop on target
    driveBackward();
    delay(80);
    stopMotors();
    delay(350);                             // position updates every 250 ms: wait for a fresh one
  }
  stopMotors();
  Serial.print("Moved ");
  Serial.print(moved, 1);
  Serial.println(" cm");
}

// Both sides backward at full power (pulses are short, so no speed control needed).
// Goes forward instead? Swap HIGH and LOW on IN1-IN4.
// Spins in place? One side is wired the other way: swap HIGH/LOW for that side only.
void driveBackward() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);   // right side backward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);   // left side backward
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
}

// Turn both enable pins off
void stopMotors() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

// All motor pins as outputs and LOW (called first in setup)
void motorsOff() {
  int pins[] = {ENA, IN1, IN2, ENB, IN3, IN4};
  for (int p : pins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
}

// ================ SERVO HELPER ================

// Move a servo 1 degree every 15 ms (about 67 deg per second),
// so the bucket doesn't throw the bean or the substrate.
void moveServo(Servo &s, int target) {
  int pos = s.read();
  while (pos != target) {
    pos += (target > pos) ? 1 : -1;
    s.write(pos);
    delay(15);
  }
}

// ================ CALIBRATION (MODE 0) ================
// Serial Monitor: 9600 baud, line ending = Newline. Commands:
//   a 85 / b 90   move the arm / bucket servo to an angle (slowly)
//   t             save: the bucket JUST touches the orzo now
//   d+ / d-       bigger (+) or smaller (-) numbers move the arm DOWN
//   h             save: the bucket is level now
//   s             save: the bucket is in the scoop position now
//   o / r         the IR is above orzo / rocks: record one reading (optional re-check)
//   c             clear the IR readings
// After every command it prints the settings it can work out, ready to copy.

int armPos = 90, bucketPos = 90;          // current servo angles
int touchAngle = -1, downDir = 0;         // from t and d+ / d-
int holdAngle = -1, scoopAngle = -1;      // from h and s
float orzoMin = 9999, orzoMax = -1, orzoSum = 0;   // orzo readings
float rockMin = 9999, rockMax = -1, rockSum = 0;   // rock readings
int orzoCount = 0, rockCount = 0;

// Both servos start in the middle (90). Hold the arm: this first move jumps.
void calibrationStart() {
  arm.write(armPos);
  bucket.write(bucketPos);
  Serial.println("Calibration. Commands: a <angle>, b <angle>, t, d+, d-, h, s, o, r, c");
}

// Show live values once per second, and handle one command at a time
void calibrationLoop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.print("IR: ");
    Serial.print(readIR(), 1);
    Serial.print("   arm: ");
    Serial.print(armPos);
    Serial.print("   bucket: ");
    Serial.println(bucketPos);
  }

  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  char c = cmd.charAt(0);
  bool hasNumber = cmd.length() > 1 && isDigit(cmd.charAt(cmd.length() - 1));  // "ab" is ignored
  int value = constrain((int)cmd.substring(1).toInt(), 0, 180);

  if (c == 'a' && hasNumber) {               // move the arm
    moveServo(arm, value);
    armPos = value;
  } else if (c == 'b' && hasNumber) {        // move the bucket
    moveServo(bucket, value);
    bucketPos = value;
  } else if (cmd == "t") {
    touchAngle = armPos;
    Serial.println("Touch angle saved. Now type d+ if bigger numbers lowered the arm, d- if smaller numbers did");
  } else if (cmd == "d+") {
    downDir = 1;
  } else if (cmd == "d-") {
    downDir = -1;
  } else if (cmd == "h") {
    holdAngle = bucketPos;
  } else if (cmd == "s") {
    scoopAngle = bucketPos;
  } else if (cmd == "o" || cmd == "r") {     // record one IR reading (lowest, highest, sum)
    float v = readIR();
    if (cmd == "o") {
      orzoCount++;
      orzoSum += v;
      if (v < orzoMin) orzoMin = v;
      if (v > orzoMax) orzoMax = v;
    } else {
      rockCount++;
      rockSum += v;
      if (v < rockMin) rockMin = v;
      if (v > rockMax) rockMax = v;
    }
    Serial.print("Recorded ");
    Serial.print(cmd == "o" ? "orzo: " : "rocks: ");
    Serial.println(v, 1);
  } else if (cmd == "c") {
    orzoMin = rockMin = 9999;
    orzoMax = rockMax = -1;
    orzoSum = rockSum = 0;
    orzoCount = rockCount = 0;
    Serial.println("IR readings cleared");
  } else {
    Serial.println("Commands: a <angle>, b <angle>, t, d+, d-, h, s, o, r, c");
    return;
  }
  printSettings();
}

// Print every setting we can work out so far, as code lines to copy.
void printSettings() {
  Serial.println();
  Serial.println("--- Copy into the settings at the top ---");

  // Arm: from the CAD simulation, dig 12 deg past touching, lift 15 deg above it
  if (touchAngle >= 0 && downDir != 0) {
    Serial.print("const int ARM_UP = ");
    Serial.print(constrain(touchAngle - 15 * downDir, 0, 180));
    Serial.println(";");
    Serial.print("const int ARM_DOWN = ");
    Serial.print(constrain(touchAngle + 12 * downDir, 0, 180));
    Serial.println(";");
  } else {
    Serial.println("(ARM_UP / ARM_DOWN: type t, then d+ or d-)");
  }

  // Bucket: DUMP is the same distance from HOLD as SCOOP, on the other side
  if (holdAngle >= 0) {
    Serial.print("const int BUCKET_HOLD = ");
    Serial.print(holdAngle);
    Serial.println(";");
  } else {
    Serial.println("(BUCKET_HOLD: level the bucket, then type h)");
  }
  if (holdAngle >= 0 && scoopAngle >= 0) {
    Serial.print("const int BUCKET_SCOOP = ");
    Serial.print(scoopAngle);
    Serial.println(";");
    Serial.print("const int BUCKET_DUMP = ");
    Serial.print(constrain(2 * holdAngle - scoopAngle, 0, 180));
    Serial.println(";");
  } else {
    Serial.println("(BUCKET_SCOOP / BUCKET_DUMP: turn the bucket to scoop, then type s)");
  }

  // IR (only if o and r were used): halfway between the CLOSEST orzo and rock readings.
  // If they overlap, use halfway between the two averages (best possible guess).
  if (orzoCount > 0 && rockCount > 0) {
    bool orzoHigher;
    float threshold;
    if (orzoMin > rockMax || rockMin > orzoMax) {     // the two groups don't overlap
      orzoHigher = orzoMin > rockMax;
      threshold = orzoHigher ? (orzoMin + rockMax) / 2 : (orzoMax + rockMin) / 2;
      Serial.print("Gap between the closest orzo and rock readings: ");
      Serial.println(orzoHigher ? orzoMin - rockMax : rockMin - orzoMax, 1);
    } else {                                          // they overlap
      float orzoAvg = orzoSum / orzoCount;
      float rockAvg = rockSum / rockCount;
      orzoHigher = orzoAvg > rockAvg;
      threshold = (orzoAvg + rockAvg) / 2;
      Serial.println("Note: orzo and rock readings overlap. Using halfway between the averages.");
    }
    Serial.print("const float IR_THRESHOLD = ");
    Serial.print(threshold, 1);
    Serial.println(";");
    Serial.print("const bool ORZO_IS_HIGHER = ");
    Serial.print(orzoHigher ? "true" : "false");
    Serial.println(";");
  }
  Serial.println();
}
