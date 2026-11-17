// MS6 Nav 1: drive to the mission site
#include "Enes100.h"

// Change before uploading
const int MARKER_ID = 0;
const int ROOM      = 1116;
const int WIFI_TX   = 3;
const int WIFI_RX   = 2;

// Mission site position (m)
const float MISSION_X        = 0.55;
const float MISSION_Y_TOP    = 1.45;
const float MISSION_Y_BOTTOM = 0.55;
const float SIDE_DIST        = 0.35;  // parking distance from the site center
const float POINT_TOL        = 0.05;  // close enough to the parking spot

// L298N pins (same as MS5)
const int R_EN = 10, R_IN1 = 9, R_IN2 = 8;   // right motors
const int L_EN = 5,  L_IN3 = 7, L_IN4 = 6;   // left motors

// Driving
const int   DRIVE_SPEED = 200;
const int   TURN_SPEED  = 170;
const float TURN_TOL    = 0.10;  // turning is done within ~6 deg
const float BIG_ERR     = 0.50;  // more than ~30 deg off: stop and turn first
const float K_HEADING   = 100;   // steering strength

float x, y, theta;

void setup() {
  // Set up motor pins
  pinMode(R_EN, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(L_EN, OUTPUT); pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT);
  stopMotors();

  // Connect to the Vision System and wait for the marker
  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM, WIFI_TX, WIFI_RX);
  while (!readPose()) {}

  // 1. The site is on the opposite side of the start; pick a parking spot on our side of it
  bool siteBelow = (y > 1.0);
  float missionY = siteBelow ? MISSION_Y_BOTTOM : MISSION_Y_TOP;
  float parkY = siteBelow ? missionY + SIDE_DIST : missionY - SIDE_DIST;
  Enes100.println("Nav1: going to mission site");

  // 2. Drive to the parking spot
  goToPoint(MISSION_X, parkY, POINT_TOL);

  // 3. Turn so the arm (right side) faces the site
  turnTo(siteBelow ? 0 : PI);
  Enes100.println("Nav1: arrived at mission site");
}

void loop() {
  stopMotors();  // done
}

// Drive to point (tx, ty)
void goToPoint(float tx, float ty, float tol) {
  while (true) {
    if (!readPose()) continue;
    float dx = tx - x, dy = ty - y;
    float dist = sqrt(dx * dx + dy * dy);
    float err = angleDiff(atan2(dy, dx), theta);
    if (dist <= tol) break;                          // arrived
    if (dist < 0.15 && fabs(err) > PI / 2) break;    // just passed it, don't turn back
    driveToward(atan2(dy, dx));
  }
  stopMotors();
}

// Drive forward while steering toward an angle
void driveToward(float targetAngle) {
  float err = angleDiff(targetAngle, theta);
  if (fabs(err) > BIG_ERR) {     // too far off: turn in place first
    turnTo(targetAngle);
    return;
  }
  int corr = K_HEADING * err;    // bigger error, stronger steering
  setMotors(DRIVE_SPEED - corr, DRIVE_SPEED + corr);
}

// Turn in place until facing an angle
void turnTo(float targetAngle) {
  while (true) {
    if (!readPose()) continue;
    float err = angleDiff(targetAngle, theta);
    if (fabs(err) < TURN_TOL) break;
    if (err > 0) setMotors(-TURN_SPEED, TURN_SPEED);   // turn left
    else         setMotors(TURN_SPEED, -TURN_SPEED);   // turn right
  }
  stopMotors();
  delay(200);
}

// Read x, y, theta; stop and return false if the marker is not visible
bool readPose() {
  if (!Enes100.isVisible()) {
    stopMotors();
    delay(50);
    return false;
  }
  x = Enes100.getX();
  y = Enes100.getY();
  theta = Enes100.getTheta();
  return true;
}

// Angle error between -PI and PI, so the OTV turns the short way
float angleDiff(float target, float current) {
  float d = target - current;
  while (d > PI)  d -= 2 * PI;
  while (d < -PI) d += 2 * PI;
  return d;
}

// Set motor speeds from -255 to 255 (negative = backward)
void setMotors(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  digitalWrite(L_IN3, left >= 0 ? HIGH : LOW);    // direction
  digitalWrite(L_IN4, left >= 0 ? LOW : HIGH);
  analogWrite(L_EN, abs(left));                   // speed

  digitalWrite(R_IN1, right >= 0 ? HIGH : LOW);
  digitalWrite(R_IN2, right >= 0 ? LOW : HIGH);
  analogWrite(R_EN, abs(right));
}

// Stop both sides
void stopMotors() {
  analogWrite(L_EN, 0);
  analogWrite(R_EN, 0);
}
