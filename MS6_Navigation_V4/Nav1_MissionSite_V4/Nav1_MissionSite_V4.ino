// MS6 Nav 1 (V4): drive from the landing zone to the mission site,
// then turn so the arm (right side of the OTV) faces the site.
#include "Enes100.h"

// ===== Settings: check before uploading =====
const int MARKER_ID = 205;    // ArUco ID on our OTV (must appear in the Vision System's "Seen markers")
const int ROOM      = 1201;   // room of the Vision System we connect to
const float MARKER_BACK = 0.06;  // m: marker center is this far behind the OTV center (toward the rear)

// ===== Mission site (m, from Arena Details) =====
// The OTV starts on spot A (top) or B (bottom); the site is always on the other spot.
const float MISSION_X        = 0.5;   // x of both spots
const float MISSION_Y_TOP    = 1.5;   // spot A
const float MISSION_Y_BOTTOM = 0.5;   // spot B
const float SIDE_DIST        = 0.35;  // OTV center parks this far from the site center (tune for 150 mm)
const float POINT_TOL        = 0.05;  // within 5 cm of the parking spot counts as arrived

// ===== L298N pins (same wiring as MS5) =====
const int R_EN = 10, R_IN1 = 9, R_IN2 = 8;   // right motors: speed (PWM), direction, direction
const int L_EN = 5,  L_IN3 = 7, L_IN4 = 6;   // left motors: speed (PWM), direction, direction

// ===== Driving =====
const int   DRIVE_SPEED = 200;   // PWM 0-255 when driving forward
const int   TURN_SPEED  = 170;   // PWM 0-255 when turning in place
const float TURN_TOL    = 0.05;  // rad (~3 deg): a turn is done when the error is smaller than this
const float BIG_ERR     = 0.50;  // rad (~30 deg): bigger heading error -> stop and turn in place first
const float K_HEADING   = 100;   // steering strength: PWM change per rad of heading error

// ===== Camera timing: the R4 library gets a new position only every 250 ms (plus network delay) =====
const int   SETTLE_MS = 500;    // after stopping, wait this long so the position shows where we stopped
const float APPROACH  = 0.15;   // m: drive normally until this close to a stop point, then finish with nudges
float turnRate  = 1.5;    // rad/s at TURN_SPEED: starts high (so the first burst is short), then measured
float driveRate = 0.25;   // m/s at DRIVE_SPEED: starts high (so the first nudge is short), then measured

float x, y, theta;   // latest OTV center position (m) and heading (rad)

void setup() {
  // Motor pins are outputs; keep the motors off until we are ready
  pinMode(R_EN, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(L_EN, OUTPUT); pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT);
  stopMotors();

  // Connect over the R4's built-in Wi-Fi (waits until connected), then wait until the camera sees our marker
  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM);
  while (!readPose()) {}

  // 1. Starting on the top spot (y > 1) means the site is the bottom spot, and the other way around.
  //    Park on the side of the site that faces us, SIDE_DIST from its center.
  bool siteBelow = (y > 1.0);
  float missionY = siteBelow ? MISSION_Y_BOTTOM : MISSION_Y_TOP;
  float parkY = siteBelow ? missionY + SIDE_DIST : missionY - SIDE_DIST;
  Enes100.println("Nav1: going to mission site");

  // 2. Drive to the parking spot
  goToPoint(MISSION_X, parkY, POINT_TOL);

  // 3. The arm is on the right: facing +x (0) points it down, facing -x (PI) points it up
  turnTo(siteBelow ? 0 : PI);
  Enes100.println("Nav1: arrived at mission site");
}

void loop() {
  stopMotors();  // all the work is in setup(); stay still afterwards
}

// Drive to point (tx, ty), always steering toward it, and stop within tol of it
void goToPoint(float tx, float ty, float tol) {
  // 1. Drive toward the point until APPROACH away
  while (true) {
    if (!readPose()) continue;                       // no position: motors already stopped, try again
    float dx = tx - x, dy = ty - y;
    if (sqrt(dx * dx + dy * dy) <= APPROACH) break;
    driveToward(atan2(dy, dx));
  }
  // 2. Stop, look, and finish with short nudges
  look();
  while (true) {
    float dx = tx - x, dy = ty - y;
    float dist = sqrt(dx * dx + dy * dy);            // distance to the target (m)
    float err = angleDiff(atan2(dy, dx), theta);     // angle between our heading and the target
    if (dist <= tol) break;                          // arrived
    if (fabs(err) > PI / 2) break;                   // target is behind us (we just passed it): stop, don't turn back
    if (fabs(err) > BIG_ERR) turnTo(atan2(dy, dx));  // off to the side: face it first
    nudge(dist);
  }
}

// Drive forward while steering toward targetAngle (rad)
void driveToward(float targetAngle) {
  float err = angleDiff(targetAngle, theta);   // > 0: target is to the left, < 0: to the right
  if (fabs(err) > BIG_ERR) {                   // too far off to steer smoothly: turn in place first
    turnTo(targetAngle);
    return;
  }
  int corr = K_HEADING * err;                  // steering amount grows with the error
  setMotors(DRIVE_SPEED - corr, DRIVE_SPEED + corr);   // one side slower, the other faster: curve toward the target
}

// Turn in place until the heading is within TURN_TOL of targetAngle
void turnTo(float targetAngle) {
  // Turn in bursts: turn for the time 80% of the error should take, stop, look, repeat.
  // (Turning until the camera says "done" overshoots, because the position is up to ~0.5 s old.)
  look();
  for (int burst = 0; burst < 12; burst++) {   // at most 12 bursts, so it can never swing back and forth forever
    float err = angleDiff(targetAngle, theta);
    if (fabs(err) < TURN_TOL) break;
    float ms = fabs(err) / turnRate * 800;      // burst time (ms) for 80% of the error
    ms = constrain(ms, 30.0f, 1500.0f);         // too short to move? turnRate drops, so the next burst is longer
    float before = theta;
    if (err > 0) setMotors(-TURN_SPEED, TURN_SPEED);   // left side back, right side forward: turn left (CCW)
    else         setMotors(TURN_SPEED, -TURN_SPEED);   // left side forward, right side back: turn right (CW)
    delay((unsigned long)ms);
    look();
    // Learn how fast this OTV really turns, so the next burst is the right length
    float measured = fabs(angleDiff(theta, before)) / (ms / 1000);
    turnRate = constrain((turnRate + measured) / 2, 0.2f, 4.0f);
  }
}

// Stop, then wait until the camera position shows where we stopped (it updates only every ~250 ms)
void look() {
  stopMotors();
  delay(SETTLE_MS);
  while (!readPose()) {}
}

// Drive straight about dist m in one short burst (negative = backward), then stop and look
void nudge(float dist) {
  float ms = fabs(dist) / driveRate * 1000;     // burst time (ms)
  ms = constrain(ms, 80.0f, 1500.0f);
  float x0 = x, y0 = y;
  int dir = (dist >= 0) ? 1 : -1;
  setMotors(dir * DRIVE_SPEED, dir * DRIVE_SPEED);
  delay((unsigned long)ms);
  look();
  // Learn how fast this OTV really drives, so the next nudge is the right length
  float measured = sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) / (ms / 1000);
  driveRate = constrain((driveRate + measured) / 2, 0.03f, 0.5f);
}

// Read x, y, theta from the Vision System.
// If the marker is not visible the values are not valid (the page shows -1), so stop and return false.
bool readPose() {
  if (!Enes100.isVisible()) {
    stopMotors();
    delay(50);
    return false;
  }
  readCenter();
  return true;
}

// The Vision System gives the marker's position. Move it forward by MARKER_BACK along the heading
// to get the OTV center, which is what all the distances in this program are measured from.
void readCenter() {
  theta = Enes100.getTheta();
  x = Enes100.getX() + MARKER_BACK * cos(theta);
  y = Enes100.getY() + MARKER_BACK * sin(theta);
}

// target - current, wrapped to -PI..PI so the OTV always turns the short way
// (example: from 3.0 to -3.0 rad gives +0.28, not -6.0)
float angleDiff(float target, float current) {
  float d = target - current;
  while (d > PI)  d -= 2 * PI;
  while (d < -PI) d += 2 * PI;
  return d;
}

// Set each side's speed from -255 to 255 (negative = backward).
// Direction pins: IN1/IN3 HIGH + IN2/IN4 LOW = forward. Speed: PWM on ENA/ENB.
void setMotors(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  digitalWrite(L_IN3, left >= 0 ? HIGH : LOW);    // left direction
  digitalWrite(L_IN4, left >= 0 ? LOW : HIGH);
  analogWrite(L_EN, abs(left));                   // left speed

  digitalWrite(R_IN1, right >= 0 ? HIGH : LOW);   // right direction
  digitalWrite(R_IN2, right >= 0 ? LOW : HIGH);
  analogWrite(R_EN, abs(right));                  // right speed
}

// Stop both sides (speed 0)
void stopMotors() {
  analogWrite(L_EN, 0);
  analogWrite(R_EN, 0);
}
