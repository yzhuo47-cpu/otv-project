//Jason's Mission Attempt
#include "Enes100.h"
#include <Arduino_FreeRTOS.h>
#include <Servo.h>

//Variables
//Variables for Connection
const int MARKER_ID = 147;
const int ROOM = 1201;
//Variables for Pins
const int TRIG = 12;
const int ECHO = 11;
const int R_EN = 10, R_IN1 = 9, R_IN2 = 8;
const int L_EN = 5, L_IN3 = 7, L_IN4 = 6;
const int ARM_PIN = A1;
const int BUCKET_PIN = A2;
const int IR_PIN = A0;
//Variables for Arena
//Variables for OTV
const float OffsetX = 0.06;
const float OffsetY = -0.08; //Total Guess
int DriveSpeed = 224;
int TurnSpeed = 224;
float x, y, theta;
//Variables for Timing
const int Settle = 333;
//Variables for Sensing
const float Threshold = 27.0;
float LeftPlot;
float RightPlot;
float OrzoTest; //Not Used Right Now
//Other Stuff
Servo arm;
Servo bucket;

void setup() {
  pinMode(R_EN, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(L_IN3, OUTPUT);
  pinMode(L_IN4, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  arm.attach(ARM_PIN);
  bucket.attach(BUCKET_PIN);
  stopMotors();
  moveServo(arm, 90);
  moveServo(bucket, 90);

  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM);
  while (!canTrack()) {}
  if (y < 1) {
    goTo(0.5 - OffsetY, 0.5);
    turnTo(PI / 2);
    goTo(0.5 - OffsetY, 1.38);
  }
  else {
    goTo(0.5 + OffsetY, 1.5);
    turnTo(-PI / 2);
    goTo(0.5 + OffsetY, 0.62);
  }
  while (median3(medianDistanceCM(), medianDistanceCM(), medianDistanceCM()) <= 9) {
    float distance = median3(medianDistanceCM(), medianDistanceCM(), medianDistanceCM()) / 100.0;
    pulse(0.1 - distance);
  }
  moveServo(arm, 60);
  moveServo(bucket, 60);
  stabilize();
  turnTo(theta + PI / 2);
  pulse(0.06);
  OrzoTest = readIR();
  pulse(-0.06);
  moveServo(arm, 90);
  delay(100);
  moveServo(bucket, 165);
  delay(250);
  moveServo(arm, 60);
  delay(100);
  moveServo(bucket, 60);
  pulse(-0.084);
  moveServo(arm, 90);
  delay(100);
  moveServo(bucket, 165);
  delay(250);
  moveServo(arm, 60);
  pulse(0.25);
  stabilize();
  turnTo(theta - PI / 2);
  pulse(0.4);
  stabilize();
  turnTo(theta - PI / 2);
  pulse(0.3);
  LeftPlot = readIR();
  pulse(-0.084);
  RightPlot = readIR();
  compare();
} //Setup End

void loop() {
  stopMotors();
}



//FUNCTIONS
bool canTrack() {
  if (!Enes100.isVisible()) {
    stopMotors();
    delay(50);
    return false;
  }
  theta = Enes100.getTheta();
  x = Enes100.getX() + (OffsetX * cos(theta)) - (OffsetY * sin(theta));
  y = Enes100.getY() + (OffsetX * sin(theta)) + (OffsetY * cos(theta));
  return true;
}
void stabilize() {
  stopMotors();
  delay(Settle);
  while (!canTrack()) {}
}
void stopMotors() {
  analogWrite(L_EN, 0);
  analogWrite(R_EN, 0);
}
void setMotors(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  digitalWrite(L_IN3, left >= 0 ? HIGH : LOW);
  digitalWrite(L_IN4, left >= 0 ? LOW : HIGH);
  digitalWrite(R_IN1, right >= 0 ? HIGH : LOW);
  digitalWrite(R_IN2, right >= 0 ? LOW : HIGH);
  analogWrite(L_EN, (left == 0) ? 0 : constrain(abs(left), 128, 255));
  analogWrite(R_EN, (right == 0) ? 0 : constrain(abs(right), 128, 255));
}
float scan() {
  static unsigned long lastPing = 0;
  unsigned long since = millis() - lastPing;
  if (since < 60) {
    delay(60 - since);
  }
  lastPing = millis();
  
  vTaskSuspendAll();
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long t = pulseIn(ECHO, HIGH, 30000);
  xTaskResumeAll();
  float cm = (t == 0) ? 999 : t * 0.0343 / 2;
  
  static bool reported = false;
  static unsigned long lastReport = 0;
  if (!reported || millis() - lastReport >= 500) {
    reported = true;
    lastReport = millis();
    if (t == 0) {
      Enes100.println("ultrasonic no echo");
    }
    else {
      Enes100.println("ultrasonic %.1f cm", cm);
    }
  }
  return cm;
}
float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}
float medianDistanceCM() {
  float a = scan();
  float b = scan();
  float c = scan();
  return median3(a, b, c);
}
float distanceTo(float xpos, float ypos) {
  float dx = xpos - x;
  float dy = ypos - y;
  float distance = sqrt(dx * dx + dy * dy);
  return distance;
}
float angleTo(float xpos, float ypos) {
  float dx = xpos - x;
  float dy = ypos - y;
  float angle = atan2(dy, dx);
  return angle;
}
float angleDifference(float target, float current) {
  float difference = target - current;
  while (difference > PI) {
    difference -= 2 * PI;
  }
  while (difference < -PI) {
    difference += 2 * PI;
  }
  return difference;
}
void turnTo(float targetAngle) {
  stabilize();
  while (fabs(angleDifference(targetAngle, theta)) > (PI / 36)) {
    float difference = angleDifference(targetAngle, theta);
    TurnSpeed = (difference >= 0 ? 1 : -1) * (128 + 6144 * pow((fabs(difference) / PI), 3));
    float ms = constrain(500.0f * fabs(difference), 100.0f, 500.0f);
    setMotors(-TurnSpeed, TurnSpeed);
    delay((unsigned long)ms);
    stabilize();
  }
}
void goTo(float xpos, float ypos) {
  stabilize();
  while (distanceTo(xpos, ypos) > 0.1) {
    float distance = distanceTo(xpos, ypos);
    turnTo(angleTo(xpos, ypos));
    DriveSpeed = 224 * pow(2 * distance, 0.75);
    float ms = distance / 0.5 * 1000;
    setMotors(DriveSpeed, DriveSpeed);
    delay((unsigned long)ms);
    stabilize();
  }
}
void pulse(float distance) {
  stabilize();
  int sign = (distance < 0) ? -1 : 1;
  DriveSpeed = 224 * pow(2 * fabs(distance), 0.75);
  DriveSpeed *= sign;
  float ms = fabs(distance) / 0.5 * 1000;
  setMotors(DriveSpeed, DriveSpeed);
  delay((unsigned long)ms);
  stabilize();
}
float readIR() {
  long sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogRead(IR_PIN);
    delay(10);
  }
  return sum / 50.0;
}
void moveServo(Servo &s, int target) {
  int pos = s.read();
  while (pos != target) {
    pos += (target > pos) ? 1 : -1;
    s.write(pos);
    delay(15);
  }
}
void compare() {
  if (fabs(LeftPlot - RightPlot) < 2.00) { //BASICALLY THE SAME
    if (min(LeftPlot, RightPlot) > Threshold) {
      Enes100.mission(LOCATION, NEITHER);
    }
    else {
      Enes100.mission(LOCATION, BOTH);
    }
  }
  else if (LeftPlot < Threshold) {
    Enes100.mission(LOCATION, DIAGONAL);
  }
  else {
    Enes100.mission(LOCATION, ADJACENT);
  }
}
