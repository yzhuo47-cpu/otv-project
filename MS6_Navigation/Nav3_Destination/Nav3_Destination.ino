// MS6 Navigation Objective 3: drive under the limbo and completely into the destination zone.
// Start this program anywhere after the obstacles and before the limbo.
#include "Enes100.h"

// ---------- Settings (check these before uploading) ----------
const int MARKER_ID = 0;        // TODO: your ArUco marker ID
const int ROOM      = 1116;     // TODO: 1116, 1120 or 1201
const int WIFI_TX   = 3;        // Wi-Fi module TX pin
const int WIFI_RX   = 2;        // Wi-Fi module RX pin

// Arena (meters)
const float LIMBO_Y   = 1.5;    // center y of the limbo opening (top half of the arena)
const float DEST_X    = 3.8;    // stop here: whole OTV is inside the destination zone (tune on the arena)
const float LANE_TOL  = 0.05;   // how close to LIMBO_Y is "lined up"
const float LOOKAHEAD = 0.5;    // steer toward a point this far ahead in the lane

// L298N pins
const int L_EN = 5,  L_IN1 = 9, L_IN2 = 8;   // left side
const int R_EN = 10, R_IN3 = 7, R_IN4 = 6;   // right side

// Driving
const int   DRIVE_SPEED = 200;   // PWM while driving
const int   TURN_SPEED  = 170;   // PWM while turning in place
const float TURN_TOL    = 0.10;  // rad, "close enough" when turning (~6 deg)
const float BIG_ERR     = 0.50;  // rad, if heading is off by more than this, stop and turn again
const float K_HEADING   = 100;   // steering strength while driving

float x, y, theta;

void setup() {
  // Motor pins
  pinMode(L_EN, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_EN, OUTPUT); pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT);
  stopMotors();

  // Connect to the Vision System and wait until the marker is seen
  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM, WIFI_TX, WIFI_RX);
  while (!readPose()) {}
  Enes100.println("Nav3: going to destination");

  // Line up with the limbo, then face +x
  moveToY(LIMBO_Y, x);
  turnTo(0);

  // Drive straight under the limbo into the destination zone
  while (x < DEST_X) {
    if (!readPose()) continue;
    driveToward(atan2(LIMBO_Y - y, LOOKAHEAD));   // hold the limbo lane
  }
  stopMotors();
  Enes100.println("Nav3: in destination zone");
}

void loop() {
  stopMotors();  // done, stay still
}

// ---------- Navigation ----------

// Move straight up or down until reaching targetY, staying at x = holdX
void moveToY(float targetY, float holdX) {
  while (!readPose()) {}                        // wait until the marker is seen
  if (fabs(targetY - y) < LANE_TOL) return;     // already there
  float dir = (targetY > y) ? 1 : -1;
  float heading = dir * PI / 2;                 // +PI/2 = up, -PI/2 = down
  turnTo(heading);
  while (true) {
    if (!readPose()) continue;
    if ((targetY - y) * dir <= LANE_TOL) break; // reached (or just passed) targetY
    driveToward(atan2(dir * LOOKAHEAD, holdX - x)); // steer back to holdX if drifting
  }
  stopMotors();
}

// Drive forward while steering toward targetAngle; turn in place first if far off
void driveToward(float targetAngle) {
  float err = angleDiff(targetAngle, theta);
  if (fabs(err) > BIG_ERR) {
    turnTo(targetAngle);
    return;
  }
  int corr = K_HEADING * err;                 // err > 0 means target is to the left
  setMotors(DRIVE_SPEED - corr, DRIVE_SPEED + corr);
}

// Turn in place until facing targetAngle
void turnTo(float targetAngle) {
  while (true) {
    if (!readPose()) continue;
    float err = angleDiff(targetAngle, theta);
    if (fabs(err) < TURN_TOL) break;
    if (err > 0) setMotors(-TURN_SPEED, TURN_SPEED);   // turn left (counterclockwise)
    else         setMotors(TURN_SPEED, -TURN_SPEED);   // turn right (clockwise)
  }
  stopMotors();
  delay(200);
}

// ---------- Helpers ----------

// Read position from the Vision System; stop and return false if the marker is not visible
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

// Smallest angle from current to target, in -PI..PI (so the OTV never turns the long way)
float angleDiff(float target, float current) {
  float d = target - current;
  while (d > PI)  d -= 2 * PI;
  while (d < -PI) d += 2 * PI;
  return d;
}

// Set left/right motor speed, -255..255 (negative = backward)
void setMotors(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  digitalWrite(L_IN1, left >= 0 ? HIGH : LOW);
  digitalWrite(L_IN2, left >= 0 ? LOW : HIGH);
  analogWrite(L_EN, abs(left));

  digitalWrite(R_IN3, right >= 0 ? HIGH : LOW);
  digitalWrite(R_IN4, right >= 0 ? LOW : HIGH);
  analogWrite(R_EN, abs(right));
}

void stopMotors() {
  analogWrite(L_EN, 0);
  analogWrite(R_EN, 0);
}
