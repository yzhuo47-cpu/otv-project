// OTV hardware test: forward, backward, turn left/right, then move both arm servos.
// Runs once after power-on or reset. Press the Arduino RESET button to run it again.

#include <Servo.h>

// ----- Motor pins (L298N) -----
const int ENA = 10;  // right side speed
const int IN1 = 9;   // right side direction
const int IN2 = 8;
const int ENB = 5;   // left side speed
const int IN3 = 7;   // left side direction
const int IN4 = 6;

// ----- Servo pins -----
const int ARM_PIN = A1;     // upper servo: moves the arm
const int BUCKET_PIN = A2;  // lower servo: turns the bucket

// ----- Test settings (change these) -----
const bool TEST_DRIVE = true;    // false = skip driving (e.g. servo test on the table)
const bool TEST_SERVOS = true;   // false = skip servos
const int SPEED = 255;           // motor speed, 0-255
const int MOVE_TIME = 1500;      // how long each drive step lasts (ms)
const int ARM_MIN = 60, ARM_MAX = 120;        // arm test range (degrees)
const int BUCKET_MIN = 60, BUCKET_MAX = 120;  // bucket test range (degrees)

Servo arm;
Servo bucket;

void setup() {
  Serial.begin(9600);

  // Motor pins as outputs, motors off
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  drive(0, 0);

  Serial.println("Starting in 3 s - put the OTV down");
  delay(3000);

  // Part 1: drive test (1 = forward, -1 = backward, for right side then left side)
  if (TEST_DRIVE) {
    driveStep("Forward",     1,  1);
    driveStep("Backward",   -1, -1);
    driveStep("Turn left",   1, -1);   // right forward, left backward
    driveStep("Turn right", -1,  1);   // right backward, left forward
  }

  // Part 2: servo test
  // Servos are attached only after driving: the Servo library stops PWM on pin 10 (ENA)
  if (TEST_SERVOS) {
    arm.attach(ARM_PIN);        // servo goes to 90 degrees when attached
    bucket.attach(BUCKET_PIN);
    delay(1000);
    servoStep("Arm", arm, ARM_MIN, ARM_MAX);
    servoStep("Bucket", bucket, BUCKET_MIN, BUCKET_MAX);
  }

  Serial.println("Test done");
}

void loop() {
  // Nothing here: the test runs once in setup()
}

// Drive one step for MOVE_TIME, then stop and pause
void driveStep(const char* name, int rightDir, int leftDir) {
  Serial.println(name);
  drive(rightDir, leftDir);
  delay(MOVE_TIME);
  drive(0, 0);
  delay(1000);
}

// Set both sides: 1 = forward, -1 = backward, 0 = stop
void drive(int rightDir, int leftDir) {
  setSide(ENA, IN1, IN2, rightDir);
  setSide(ENB, IN3, IN4, leftDir);
}

// Set one side's direction pins and speed
void setSide(int en, int inA, int inB, int dir) {
  digitalWrite(inA, dir == 1 ? HIGH : LOW);
  digitalWrite(inB, dir == -1 ? HIGH : LOW);
  analogWrite(en, dir == 0 ? 0 : SPEED);
}

// Move a servo 90 -> min -> max -> back to 90
void servoStep(const char* name, Servo &s, int minAngle, int maxAngle) {
  Serial.println(name);
  sweep(s, 90, minAngle);
  sweep(s, minAngle, maxAngle);
  sweep(s, maxAngle, 90);
  delay(1000);
}

// Turn a servo slowly, 1 degree every 15 ms, so the arm doesn't slam
void sweep(Servo &s, int from, int to) {
  int step = (to > from) ? 1 : -1;
  for (int a = from; a != to; a += step) {
    s.write(a);
    delay(15);
  }
  s.write(to);
  delay(500);
}
