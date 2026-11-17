#include "Enes100.h"
#include <Servo.h>

// ===================== SETTINGS =====================

// Which part to run
// 0 = IR calibration (prints IR numbers, nothing moves)
// 1 = Front side: Mission II (seed) + Mission III (rocks)
// 2 = Back side:  Mission I (read far plots + send result)
const int MODE = 1;

// Vision system
const int MARKER_ID = 0;    // TODO: our ArUco marker number
const int ROOM = 1116;      // TODO: 1116, 1120 or 1201
const int WIFI_TX = 3;      // Wi-Fi module TX wire
const int WIFI_RX = 2;      // Wi-Fi module RX wire

// Motor pins (L298N)
const int ENA = 10, IN1 = 9, IN2 = 8;   // right side
const int ENB = 5,  IN3 = 7, IN4 = 6;   // left side

// Mission pins
const int ARM_PIN = A1;     // upper servo: lifts / lowers the arm
const int BUCKET_PIN = A2;  // lower servo: turns the bucket
const int IR_PIN = A0;      // IR sensor (TCRT5000, AO pin)

// Servo angles (TODO: tune on the real OTV)
const int ARM_UP = 30;        // arm up (start position, safe for driving)
const int ARM_DOWN = 100;     // arm down, bucket in the substrate
const int BUCKET_HOLD = 90;   // bucket level (start position, bean inside)
const int BUCKET_SCOOP = 150; // bucket scoops (and drops the bean)
const int BUCKET_DUMP = 40;   // bucket pours orzo back on the bean

// IR calibration result (TODO: from MODE 0)
const int IR_THRESHOLD = 500;      // halfway between orzo and rock numbers
const bool ORZO_IS_HIGHER = true;  // true if orzo gives the bigger number

// Movement
const float PLOT_SPACING_CM = 8.4; // distance between two plot centers

// ====================================================

Servo arm;
Servo bucket;

void setup() {
  Serial.begin(9600);

  // Motors off
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  // Servos to start position
  arm.attach(ARM_PIN);
  bucket.attach(BUCKET_PIN);
  arm.write(ARM_UP);
  bucket.write(BUCKET_HOLD);
  delay(1000);

  if (MODE == 0) return;   // calibration: no Wi-Fi needed

  // Connect to the vision system (waits here until connected)
  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM, WIFI_TX, WIFI_RX);

  if (MODE == 1) frontSide();
  if (MODE == 2) missionOne();
}

void loop() {
  // Calibration: print the IR number 3 times per second
  if (MODE == 0) {
    Serial.println(readIR());
    delay(300);
  }
}

// ================ MISSIONS ================

// Front side: plant the seed, move one plot, collect rocks.
// Start: bucket above the near-left plot.
void frontSide() {
  missionTwo();
  backUp(PLOT_SPACING_CM);   // bucket now above the near-right plot
  missionThree();
}

// Mission I: read both far plots and send the pattern.
// Start: at the back of the site, IR above the far plot toward the OTV's front.
// From the back, left and right are flipped, so the first plot is far-RIGHT.
void missionOne() {
  bool farRight = isPlantable();
  backUp(PLOT_SPACING_CM);   // IR now above the far-left plot
  bool farLeft = isPlantable();

  // Pattern is described from the seed plot (near-left)
  int plot;
  if (farLeft && farRight) plot = BOTH;
  else if (farLeft)        plot = ADJACENT;  // directly behind the seed plot
  else if (farRight)       plot = DIAGONAL;  // diagonal from the seed plot
  else                     plot = NEITHER;

  Enes100.mission(LOCATION, plot);
}

// Mission II: dig a hole, drop the bean, cover it with orzo.
// Near-left plot is always orzo, so no sensing is needed.
void missionTwo() {
  moveServo(arm, ARM_DOWN);
  moveServo(bucket, BUCKET_SCOOP);  // scoop orzo, bean falls into the hole
  delay(500);
  moveServo(bucket, BUCKET_DUMP);   // pour orzo on top of the bean
  moveServo(arm, ARM_UP);
  moveServo(bucket, BUCKET_HOLD);
}

// Mission III: scoop rocks and keep them.
// Near-right plot is always rocks, so no sensing is needed.
void missionThree() {
  moveServo(arm, ARM_DOWN);
  moveServo(bucket, BUCKET_SCOOP);
  moveServo(arm, ARM_UP);           // bucket stays tilted so rocks stay in
}

// ================ IR SENSOR ================

// Average of 10 readings to reduce noise
int readIR() {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(IR_PIN);
    delay(10);
  }
  return sum / 10;
}

// Orzo and rocks reflect IR differently, so one threshold separates them
bool isPlantable() {
  int value = readIR();
  if (ORZO_IS_HIGHER) return value > IR_THRESHOLD;
  else                return value < IR_THRESHOLD;
}

// ================ MOVEMENT ================

// Back up a set distance, measured by the overhead camera.
// Short pulses keep the OTV slow so it stops close to the target.
void backUp(float cm) {
  while (!Enes100.isVisible()) delay(50);   // wait until the camera sees us
  float x0 = Enes100.getX();
  float y0 = Enes100.getY();

  unsigned long start = millis();
  while (millis() - start < 15000) {        // give up after 15 s
    if (Enes100.isVisible()) {
      float dx = Enes100.getX() - x0;
      float dy = Enes100.getY() - y0;
      if (sqrt(dx * dx + dy * dy) * 100 >= cm) break;   // meters to cm
    }
    driveBackward();
    delay(80);
    stopMotors();
    delay(200);
  }
  stopMotors();
}

// Both sides backward at full speed.
// digitalWrite, not analogWrite: the Servo library turns off PWM on pin 10.
// If the OTV goes forward, swap HIGH and LOW on IN1-IN4.
void driveBackward() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
}

void stopMotors() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

// ================ SERVO HELPER ================

// Move 1 degree at a time so the bucket doesn't throw the substrate
void moveServo(Servo &s, int target) {
  int pos = s.read();
  while (pos != target) {
    pos += (target > pos) ? 1 : -1;
    s.write(pos);
    delay(15);
  }
}
