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
const float PastObstaclesX = 2.9;
const float LanesY[3] = {0.5, 1.0, 1.5};
const float LanesX[2] = {1.4, 2.2};
//Variables for OTV
const float OffsetX = 0.06;
const float OffsetY = -0.08; //Total Guess
int DriveSpeed = 224; //PWM
int TurnSpeed = 224; //PWM
float x, y, theta;
//Variables for Timing
const int Settle = 300;

void setup() {
  pinMode(R_EN, OUTPUT); pinMode(R_IN1 OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(L_EN, OUTPUT); pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  stopMotors();

  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM);
  while (!canTrack()) {}
  Enes100.printf("Nav2: start"); //Starts Navigating Here
  //PUT FUNCTION HERE TO MOVE OTV SAFELY TO x=1
  establishLaneOrder();
  for (float LaneY : LanesY) {
    goTo(1, LaneY);
    //NEED SENSING FUNCTION HERE
  }
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
  analogWrite(L_EN, constrain(abs(left), 128, 255));
  analogWrite(R_EN, constrain(abs(right), 128, 255));
}
void goTo(float xpos, float ypos) {
  stabilize();
  while (!canTrack()) {}
  while (distanceTo(xpos, ypos) > 0.1) {
    distance = distanceTo(xpos, ypos);
    turnTo(angleTo(xpos, ypos));
    DriveSpeed = 224 * pow(2 * distance, 0.75);
    float ms = distance / 0.4;
    setMotors(DriveSpeed, DriveSpeed);
    delay((unsigned long)ms);
    stabilize();
  }
}
void turnTo(float targetAngle) {
  stabilize();
  while (!canTrack()) {}
  while (fabs(angleDifference(targetAngle, theta)) > (PI / 36)) {
    difference = angleDifference(targetAngle, theta);
    TurnSpeed = (448 / PI) * atan(((difference + pow(difference / (12 * PI), 3)) / (12*PI)));
    float ms = 500.0f
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
  x = Enes100.getx() + (OffsetX * cos(theta));
  y = Enes100.getY() + (OffsetY * sin(theta));
  return true;
}
void establishLaneOrder() {
  stabilize();
  if (y >= 1) {
    float value = LanesY[0];
    LanesY[0] = LanesY[2];
    LanesY[2] = value;
  }
  else {
    float value = LanesY[2];
    LanesY[2] = LanesY[0];
    LanesY[0] = value;
  }
}
















