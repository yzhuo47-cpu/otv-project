#include "Enes100.h"
#include <Servo.h>

// ===================== SETTINGS =====================

// Which servo to test (change, then upload again)
// 1 = arm:    find the angle where the bucket JUST touches the orzo
// 2 = bucket: find the level angle and the scoop angle
const int TEST = 1;

// Arm angle while testing the bucket: put the touch angle from TEST 1 here
const int ARM_FIXED = 90;

// Vision system: the angle is printed on the vision system page (no USB needed)
const int MARKER_ID = 147;
const int ROOM = 1201;

// Pins
const int ARM_PIN = A1;      // upper servo: lifts / lowers the arm
const int BUCKET_PIN = A2;   // lower servo: turns the bucket
const int MOTOR_PINS[] = {10, 9, 8, 5, 7, 6};   // L298N pins, kept off

// ====================================================

Servo arm;
Servo bucket;

void setup() {
  // Motors off, so the OTV can't drive away
  for (int p : MOTOR_PINS) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  // Start positions: arm up (bigger number = up on our OTV), bucket in the middle
  arm.attach(ARM_PIN);
  bucket.attach(BUCKET_PIN);
  arm.write(TEST == 1 ? 105 : ARM_FIXED);
  bucket.write(90);

  // Wait until the page can show our prints (Wi-Fi only; the marker doesn't need to be seen)
  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM);
  while (!Enes100.isConnected()) delay(100);

  // 10 s to put the OTV in place and get ready to watch
  Enes100.println("Connected. Test starts in 10 s");
  delay(10000);

  if (TEST == 1) sweep(arm, "arm", 105, 70, 1, 2000);      // arm: 105 down to 70, 1 deg every 2 s
  if (TEST == 2) sweep(bucket, "bucket", 30, 150, 3, 1500); // bucket: 30 to 150, 3 deg every 1.5 s
}

void loop() {
  // Runs once. Press RESET to run again.
}

// Move one servo from 'from' to 'to' in small steps and print each angle on the page.
// Watch the OTV: when it reaches the position you want, write down the number shown.
void sweep(Servo &s, const char *name, int from, int to, int step, int waitMs) {
  // Move slowly to the start angle first (no sudden jump)
  for (int angle = s.read(); angle != from; angle += (from > angle) ? 1 : -1) {
    s.write(angle);
    delay(15);
  }

  int dir = (to > from) ? 1 : -1;
  for (int angle = from; dir * (to - angle) >= 0; angle += dir * step) {
    s.write(angle);
    Enes100.println(String(name) + " " + String(angle));
    delay(waitMs);
  }

  // Go back slowly to the start, so the servo doesn't stay pressed into the plot
  for (int angle = to; dir * (angle - from) > 0; angle -= dir) {
    s.write(angle);
    delay(15);
  }
  s.write(from);
  Enes100.println("Done. Press RESET to run again");
}
