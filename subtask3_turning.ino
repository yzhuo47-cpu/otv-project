#include "Enes100.h"

// ===== Pins and settings (change to match your setup) =====
const int LEFT_A  = 5;    // Left L298N IN1 + IN3
const int LEFT_B  = 6;    // Left L298N IN2 + IN4
const int RIGHT_A = 3;    // Right L298N IN1 + IN3
const int RIGHT_B = 11;   // Right L298N IN2 + IN4

const int WIFI_TX = 8;    // Connected to Wi-Fi module TX
const int WIFI_RX = 12;   // Connected to Wi-Fi module RX
const int MARKER_ID = 0;  // ArUco marker ID
const int ROOM = 1116;    // Room number (1116, 1120, or 1201)

const int SPEED = 255;          // Full speed: the 10 RPM motors are slow and need full torque
const float TOLERANCE = 0.05;   // Allowed error in radians (about 3 degrees)

// Drive one side: positive = forward, negative = backward, 0 = stop
void setSide(int pinA, int pinB, int speed) {
  if (speed > 0) {
    analogWrite(pinA, speed);
    digitalWrite(pinB, LOW);
  } else if (speed < 0) {
    digitalWrite(pinA, LOW);
    analogWrite(pinB, -speed);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
}

// Drive both sides at once
void drive(int left, int right) {
  setSide(LEFT_A, LEFT_B, left);
  setSide(RIGHT_A, RIGHT_B, right);
}

// Keep an angle between -PI and PI (same range as the Vision System's theta)
float wrapAngle(float a) {
  while (a > PI)  a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

// Turn 90 degrees left in place, reading theta until the target is reached
void turn90() {
  while (!Enes100.isVisible()) delay(100);                 // Wait until the marker is seen
  float target = wrapAngle(Enes100.getTheta() + PI / 2);   // Target = current heading + 90 degrees

  while (true) {
    if (!Enes100.isVisible()) {        // Marker lost: stop and wait
      drive(0, 0);
      delay(100);
      continue;
    }
    float error = wrapAngle(target - Enes100.getTheta());
    if (fabs(error) < TOLERANCE) break;   // Close enough: done

    if (error > 0) drive(-SPEED, SPEED);  // Not far enough: keep turning left
    else           drive(SPEED, -SPEED);  // Overshot: turn back right
    delay(50);
  }
  drive(0, 0);
}

void setup() {
  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);

  // Connect to the Vision System
  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM, WIFI_TX, WIFI_RX);
  Enes100.println("Connected");

  // Turn 3 times, pausing 2 seconds before each turn
  for (int i = 1; i <= 3; i++) {
    delay(2000);
    turn90();
    Enes100.print("Turn ");
    Enes100.print(i);
    Enes100.println(" done");
  }
}

void loop() {
  // Nothing to do after the 3 turns
}
