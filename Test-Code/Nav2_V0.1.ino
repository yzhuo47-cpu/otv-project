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
const float OffsetX;
const float OffsetY;
float x, y, theta, degree;
//Variables for Timing
const int Settle = 500;

void setup() {
  pinMode(R_EN, OUTPUT); pinMode(R_IN1 OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(L_EN, OUTPUT); pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  stopMotors();

  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM);
  while (!canTrack()) {}
  Enes100.printf("Nav2: start");
  //SETUP END
}
void loop() {
  stopMotors();
}

//FUNCTIONS
void stopMotors() {
  analogWrite(L_EN, 0);
  analogWrite(R_EN, 0);
}
void goTo(float xpos, float ypos) {
  
}
void turnTo(float targetAngle) {
  
}
bool canTrack() {
  if (!Enes100.isVisible()) {
    stopMotors();
    delay(50);
    return false;
  }
  getData();
  return true;
}
void getData() {
  theta = Enes100.getTheta();
  x = Enes100.getx() + (OffsetX * cos(theta));
  y = Enes100.getY() + (OffsetY * sin(theta));
}




























