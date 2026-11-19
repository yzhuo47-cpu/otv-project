// MS6 Nav 3: drive under the limbo and completely into the goal zone.
// Start after the obstacles (where Nav2 ends, x = 2.9).
#include "Enes100.h"

// ===== Settings: check before uploading =====
const int MARKER_ID = 205;    // ArUco ID on our OTV (must appear in the Vision System's "Seen markers")
const int ROOM      = 1201;   // room of the Vision System we connect to

// ===== OTV size (m, from the MS3 CAD: measure the real OTV) =====
const float ARM_REACH  = 0.27;   // OTV center to arm tip (arm sticks out on the right)
const float HALF_WIDTH = 0.115;  // OTV center to outside of the left wheels

// ===== Arena (m, from Arena Details) =====
const float LIMBO_Y   = 1.5;    // center of the limbo opening (22 cm side walls leave y = 1.22 to 1.78)
const float DEST_X    = 3.7;    // stop line: middle of the goal zone (3.4-4.0)
const float LANE_TOL  = 0.03;   // within 3 cm of the line counts as lined up
const float LOOKAHEAD = 0.5;    // steer toward a point 0.5 m ahead on the line (smooth correction)
const float BLIND_MAX = 0.40;   // farthest we keep driving while the marker is hidden

// Drive left of the opening's center by half of (arm - wheels), ~8 cm,
// so the arm and the left wheels each have about the same room to the side walls
const float LIMBO_LINE = LIMBO_Y + (ARM_REACH - HALF_WIDTH) / 2;

// ===== L298N pins (same wiring as MS5) =====
const int R_EN = 10, R_IN1 = 9, R_IN2 = 8;   // right motors: speed (PWM), direction, direction
const int L_EN = 5,  L_IN3 = 7, L_IN4 = 6;   // left motors: speed (PWM), direction, direction

// ===== Driving =====
const int   DRIVE_SPEED = 200;   // PWM 0-255 when driving forward
const int   TURN_SPEED  = 170;   // PWM 0-255 when turning in place
const float TURN_TOL    = 0.10;  // rad (~6 deg): a turn is done when the error is smaller than this
const float BIG_ERR     = 0.50;  // rad (~30 deg): bigger heading error -> stop and turn in place first
const float K_HEADING   = 100;   // steering strength: PWM change per rad of heading error

float x, y, theta;   // latest position (m) and heading (rad) from the Vision System

void setup() {
  // Motor pins are outputs; keep the motors off until we are ready
  pinMode(R_EN, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(L_EN, OUTPUT); pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT);
  stopMotors();

  // Connect over the R4's built-in Wi-Fi (waits until connected), then wait until the camera sees our marker
  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM);
  while (!readPose()) {}
  Enes100.println("Nav3: going to destination");

  // 1. Move sideways onto the limbo line, then face +x
  moveToY(LIMBO_LINE, x);
  turnTo(0);

  // 2. Drive straight under the limbo to DEST_X.
  //    The limbo rod can hide the marker from the camera. Instead of stopping, keep driving straight
  //    and estimate x from the speed measured so far on this run.
  float startX = x, lastX = x;
  unsigned long startT = millis(), lastT = startT;
  while (true) {
    if (Enes100.isVisible()) {
      // Marker visible: normal driving, and remember this as the last good position
      x = Enes100.getX();
      y = Enes100.getY();
      theta = Enes100.getTheta();
      lastX = x;
      lastT = millis();
      if (x >= DEST_X) break;
      driveToward(atan2(LIMBO_LINE - y, LOOKAHEAD));   // aim at a point ahead on the limbo line
    } else {
      // Marker hidden: estimate x = last x + speed * time since we last saw it
      float driveTime = (lastT - startT) / 1000.0;                          // s driven while visible
      float speed = (driveTime > 1.0) ? (lastX - startX) / driveTime : 0;   // m/s (needs 1 s of data)
      float guessX = lastX + speed * (millis() - lastT) / 1000.0;
      if (speed <= 0 || guessX - lastX > BLIND_MAX) {   // no speed yet, or hidden too long: stop and wait
        stopMotors();
        delay(50);
      } else if (guessX >= DEST_X) {
        break;                                           // estimate says we reached the stop line
      } else {
        setMotors(DRIVE_SPEED, DRIVE_SPEED);             // keep going straight
      }
    }
  }
  stopMotors();
  Enes100.println("Nav3: in destination zone");
}

void loop() {
  stopMotors();  // all the work is in setup(); stay still afterwards
}

// Move straight up or down to targetY, steering to stay at x = holdX
void moveToY(float targetY, float holdX) {
  while (!readPose()) {}
  if (fabs(targetY - y) < LANE_TOL) return;   // already there
  float dir = (targetY > y) ? 1 : -1;         // +1 = up (+y), -1 = down
  float heading = dir * PI / 2;               // face +PI/2 (up) or -PI/2 (down)
  turnTo(heading);
  while (true) {
    if (!readPose()) continue;
    if ((targetY - y) * dir <= LANE_TOL) break;          // reached (or just passed) targetY
    driveToward(atan2(dir * LOOKAHEAD, holdX - x));      // aim slightly back toward holdX if we drift in x
  }
  stopMotors();
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
  while (true) {
    if (!readPose()) continue;
    float err = angleDiff(targetAngle, theta);
    if (fabs(err) < TURN_TOL) break;
    if (err > 0) setMotors(-TURN_SPEED, TURN_SPEED);   // left side back, right side forward: turn left (CCW)
    else         setMotors(TURN_SPEED, -TURN_SPEED);   // left side forward, right side back: turn right (CW)
  }
  stopMotors();
  delay(200);   // let the OTV settle before the next move
}

// Read x, y, theta from the Vision System.
// If the marker is not visible the values are not valid (the page shows -1), so stop and return false.
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
