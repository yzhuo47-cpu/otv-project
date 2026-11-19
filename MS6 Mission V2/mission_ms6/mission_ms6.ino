#include "Enes100.h"
#include <Servo.h>

// ===================== SETTINGS =====================

// Which part to run (change, then upload again)
// 1 = Front of the site: Mission II (plant seed) + Mission III (collect rocks)
// 2 = Back of the site:  Mission I (read the two far plots + send the result)
const int MODE = 2;

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

// Servo angles (0-180). Best guess: arm mounted at 90 when the bucket just
// touches the orzo, and bigger numbers lower the arm.
// If the arm goes UP when it should dig:  ARM_UP = 105, ARM_DOWN = 78
// If the bucket turns the wrong way:       swap BUCKET_SCOOP and BUCKET_DUMP
const int ARM_UP = 75;        // 15 deg above touching: start position, safe to drive
const int ARM_DOWN = 102;     // 12 deg below touching: digs about 2 cm (more hits the wrist servo)
const int BUCKET_HOLD = 90;   // bucket level: start position, holds the bean
const int BUCKET_SCOOP = 150; // scoops substrate; the bean drops out the other side
const int BUCKET_DUMP = 30;   // pours the scooped orzo back on top of the bean

// IR, from our calibration: orzo reads about 19-21, rocks about 23-25
const float IR_THRESHOLD = 21.5;   // below = orzo, above = rocks (exact middle would be 22)
const float IR_CHECK_ZONE = 0.5;   // this close to the threshold = too close to call: scan again

// Movement
const float PLOT_SPACING_CM = 8.4; // center-to-center distance of two plots

// ====================================================

Servo arm;
Servo bucket;

void setup() {
  motorsOff();                // motors can't move by accident

  arm.attach(ARM_PIN);
  bucket.attach(BUCKET_PIN);

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
  // Everything runs once in setup(). Press RESET to run again.
}

// ================ MISSIONS ================

// MODE 1. Start: OTV at the front of the site, right side facing it,
// bucket above the near-left plot (always orzo).
void frontSide() {
  missionTwo();              // plant the seed in the near-left plot
  backUp(PLOT_SPACING_CM);   // move one plot: bucket now above the near-right plot
  missionThree();            // collect rocks from the near-right plot
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
  Enes100.println("Seen from the front of the site:");
  Enes100.println(String("far-left: ") + (farLeft ? "orzo" : "rocks") + " (IR " + String(leftIR, 1) + ")");
  Enes100.println(String("far-right: ") + (farRight ? "orzo" : "rocks") + " (IR " + String(rightIR, 1) + ")");
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

// Orzo reads lower than rocks (calibration above)
bool isOrzo(float value) {
  return value < IR_THRESHOLD;
}

// ================ MOVEMENT ================

// Back up a set distance (cm), measured with the vision system.
// The vision system only gives position, so the distance is worked out
// from the start position: sqrt(dx^2 + dy^2).
void backUp(float cm) {
  // Wait for a valid start position (-1 = not connected yet, or marker not seen)
  float x0 = -1, y0 = -1;
  while (x0 < 0 || y0 < 0) {
    x0 = Enes100.getX();
    y0 = Enes100.getY();
    delay(50);
  }

  unsigned long start = millis();
  while (millis() - start < 15000) {        // safety: give up after 15 s
    float x = Enes100.getX();
    float y = Enes100.getY();
    if (x >= 0 && y >= 0) {                 // skip readings where the marker isn't seen
      float dx = x - x0;
      float dy = y - y0;
      if (sqrt(dx * dx + dy * dy) * 100 >= cm) break;   // meters to cm
    }
    // Short full-power pulse, then stop: slow and easy to stop on target
    driveBackward();
    delay(80);
    stopMotors();
    delay(350);                             // position updates every 250 ms: wait for a fresh one
  }
  stopMotors();
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
