//Jason's Nav2 Attempt
#include "Enes100.h"
#include <Arduino_FreeRTOS.h>   // R4: the ENES100 library runs WiFi as a FreeRTOS task

//Variables
//Variables for Connection
const int MARKER_ID = 147;
const int ROOM = 1201;
//Variables for Pins
const int TRIG = 12;
const int ECHO = 11;
const int R_EN = 10, R_IN1 = 9, R_IN2 = 8;
const int L_EN = 5, L_IN3 = 7, L_IN4 = 6;
//Variables for Arena
float LanesY[3] = {0.5, 1.0, 1.5};
//Variables for OTV
const float OffsetX = 0.06;
const float OffsetY = -0.08; //Total Guess
int DriveSpeed = 224; //PWM
int TurnSpeed = 224; //PWM
float x, y, theta;
//Variables for Timing
const int Settle = 300;

void setup() {
  pinMode(R_EN, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(L_IN3, OUTPUT);
  pinMode(L_IN4, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  stopMotors();

  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM);
  while (!canTrack()) {}
  Enes100.printf("Nav2: start"); //Starts Navigating Here
  startProcedure();
  establishLaneOrder();
  for (float LaneY : LanesY) {
    goTo(1, LaneY);
    turnTo(0);
    if (median3(medianDistanceCM(), medianDistanceCM(), medianDistanceCM()) >= 50) {
      goTo(1.8, LaneY);
      break;
    }
  }
  LanesY[0] = 0.5;
  LanesY[1] = 1.0;
  LanesY[2] = 1.5;
  establishLaneOrder();
  for (float LaneY : LanesY) {
    goTo(1.8, LaneY);
    turnTo(0);
    if (median3(medianDistanceCM(), medianDistanceCM(), medianDistanceCM()) >= 50) {
      goTo(2.9, LaneY);
      break;
    }
  }
  stopMotors();
  Enes100.printf("Nav2: Past all Obstacles");
} //Setup End

void loop() {
  stopMotors();
}



//FUNCTIONS
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
void turnTo(float targetAngle) {
  stabilize();
  while (fabs(angleDifference(targetAngle, theta)) > (PI / 36)) {
    float difference = angleDifference(targetAngle, theta);
    TurnSpeed = (difference >= 0 ? 1 : -1) * (128 + 6144 * pow((fabs(difference) / PI), 3));
    float ms = 500.0f;
    setMotors(-TurnSpeed, TurnSpeed);
    delay((unsigned long)ms);
    stabilize();
  }
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
bool canTrack() {
  if (!Enes100.isVisible()) {
    stopMotors();
    delay(50);
    return false;
  }
  theta = Enes100.getTheta();
  x = Enes100.getx() + (OffsetX * cos(theta)) - (OffsetY * sin(theta));
  y = Enes100.getY() + (OffsetX * sin(theta)) + (OffsetY * cos(theta));
  return true;
}
void establishLaneOrder() {
  stabilize();
  if (y >= 1) {
    LanesY[0] = 1.5;
    LanesY[1] = 1.0;
    LanesY[2] = 0.5;
  }
  else {
    LanesY[0] = 0.5;
    LanesY[1] = 1.0;
    LanesY[2] = 1.5;
  }
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
      Enes100.printf("Nav2: ultrasonic no echo");
    }
    else {
      Enes100.printf("Nav2: ultrasonic %.1f cm", cm);
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
void startProcedure() { //THIS IS A WEAK VERSION OF goTo()
  stabilize();
  if (fabs(theta) < PI / 2) {
    turnTo(0);
    while (distanceTo(1, y) > 0.1) {
      float distance = distanceTo(1, y);
      DriveSpeed = 224 * pow(2 * distance, 0.75);
      float ms = distance / 0.5 * 1000;
      setMotors(DriveSpeed, DriveSpeed);
      delay((unsigned long)ms);
      stabilize();
    }
  }
  else {
    turnTo(PI);
    while (distanceTo(1, y) > 0.1) {
      float distance = distanceTo(1, y);
      DriveSpeed = 224 * pow(2 * distance, 0.75);
      float ms = distance / 0.5 * 1000;
      setMotors(-DriveSpeed, -DriveSpeed);
      delay((unsigned long)ms);
      stabilize();
    }
  }
}













