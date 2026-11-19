// MS6 Nav 2: drive completely past the three obstacles.
// The arm can't fold, so the OTV first checks all 3 lanes with the ultrasonic, then picks its path.
#include "Enes100.h"

// ===== Settings: check before uploading =====
const int MARKER_ID = 205;    // ArUco ID on our OTV (must appear in the Vision System's "Seen markers")
const int ROOM      = 1201;   // room of the Vision System we connect to

// ===== Ultrasonic sensor (HC-SR04, front center, facing forward) =====
const int TRIG = 12;
const int ECHO = 11;

// ===== OTV size (m, from the MS3 CAD: measure the real OTV) =====
const float ARM_REACH  = 0.27;   // OTV center to arm tip (arm sticks out on the right)
const float HALF_WIDTH = 0.115;  // OTV center to outside of the left wheels
const float FRONT      = 0.10;   // OTV center to the ultrasonic sensor

// FIX: turning in place sweeps the arm tip in a full circle, so the room needed to turn
// is the ARM reach (0.27), not the wheel corner (0.17). The arm is the farthest point.
const float TURN_R = ARM_REACH;
const float MARGIN = 0.03;       // extra clearance for pose error and wobble

// ===== Arena (m, from Arena Details: obstacle columns at x = 1.5 and 2.3) =====
const float LANES[3] = {0.5, 1.0, 1.5};   // center y of the 3 obstacle rows (lane 0, 1, 2)
const float SCAN_X = 1.05;                // where lanes are checked: after the landing zone, before column 1
const float OBS_DEPTH = 0.20;             // obstacle size along x
const float PAST_OBSTACLES_X = 2.9;       // finish line: in the open zone (2.8-3.4), past column 2 (ends at 2.4)
const float LANE_TOL  = 0.03;             // within 3 cm of a lane line counts as in the lane
const float LOOKAHEAD = 0.5;              // steer toward a point 0.5 m ahead on the line (smooth correction)

// Arm reaches 0.27 m right, wheels 0.115 m left: shifting left by half the difference
// (~8 cm) leaves the same room on both sides inside a 0.5 m lane
const float LANE_SHIFT = (ARM_REACH - HALF_WIDTH) / 2;

const float SAFETY_CM = 6;       // backup emergency stop: something this close ahead -> stop and wait

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
float hitX[3];       // x where the ultrasonic saw something in each lane (99 = nothing)

void setup() {
  // Motor and trigger pins are outputs, echo is an input; motors off until ready
  pinMode(R_EN, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(L_EN, OUTPUT); pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  stopMotors();

  // Connect over the R4's built-in Wi-Fi (waits until connected), then wait until the camera sees our marker
  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM);
  while (!readPose()) {}
  Enes100.println("Nav2: start obstacles");

  // 1. Move to the nearest lane, face +x, and drive to the scan point in front of column 1
  int lane = nearestLane(y);
  moveToY(laneY(lane), x);
  turnTo(0);
  driveLaneUntil(lane, SCAN_X);

  // 2. Check all 3 lanes before entering the obstacles (no room to react later with the arm out)
  scanLanes();

  // 3. Sort the hits. The closest hit is the front of column 1. Anything seen before
  //    PAST_OBSTACLES_X is an obstacle (the limbo, log and arena wall are all farther away).
  float col1X = min(hitX[0], min(hitX[1], hitX[2]));
  bool block1[3], block2[3];
  for (int i = 0; i < 3; i++) {
    bool hit = hitX[i] < PAST_OBSTACLES_X;
    block1[i] = hit && hitX[i] < col1X + OBS_DEPTH;   // hit at column 1
    block2[i] = hit && !block1[i];                    // open at column 1, hit at column 2
  }

  // 4. Look for a lane with no hit at all (top lane first, since the limbo is on top).
  //    Arena rule: one obstacle per row. The rumbles are low, so their lane usually looks clear.
  int clear = -1;
  for (int i = 2; i >= 0; i--) {
    if (!block1[i] && !block2[i]) { clear = i; break; }
  }

  if (clear >= 0) {
    // 5a. Clear lane: drive straight through it (over the rumbles if they are there)
    Enes100.print("Nav2: clear lane "); Enes100.println(clear);
    moveToY(laneY(clear), SCAN_X);
    turnTo(0);
    driveLaneUntil(clear, PAST_OBSTACLES_X);
  } else {
    // 5b. No clear lane: start in a lane open at column 1 (first), switch to a lane open at column 2 (second).
    //     A lane blocked at column 1 is open at column 2 (one obstacle per row). Pick the closest pair.
    int first = -1, second = -1;
    for (int a = 0; a < 3; a++) {
      for (int b = 0; b < 3; b++) {
        if (block1[a] || !block1[b]) continue;
        if (first < 0 || abs(a - b) < abs(first - second)) { first = a; second = b; }
      }
    }
    if (first < 0) {   // readings make no sense: stop instead of guessing
      Enes100.println("Nav2: no path found, check the ultrasonic");
      return;
    }

    // Where to switch lanes: in the gap between the columns.
    // FIX: the OTV turns in place twice here (face sideways, then face +x again), and each turn
    // sweeps the arm tip through a circle of radius TURN_R in ALL directions. So both sides of the
    // gap need the same clearance -> the safest spot is the exact middle of the gap.
    float edge1 = col1X + OBS_DEPTH;   // back of column 1
    float face2 = hitX[first];         // front of column 2 (what lane "first" hit)
    float crossX = (edge1 + face2) / 2;
    if (face2 - edge1 < 2 * (TURN_R + MARGIN)) Enes100.println("Nav2: WARNING gap too small to turn with the arm");

    // Drive lane "first" to the gap, move sideways to lane "second", then drive to the finish line
    Enes100.print("Nav2: lane "); Enes100.print(first);
    Enes100.print(" then "); Enes100.println(second);
    moveToY(laneY(first), SCAN_X);
    turnTo(0);
    driveLaneUntil(first, crossX);
    moveToY(laneY(second), crossX);
    turnTo(0);
    driveLaneUntil(second, PAST_OBSTACLES_X);
  }

  stopMotors();
  Enes100.println("Nav2: past all obstacles");
}

void loop() {
  stopMotors();  // all the work is in setup(); stay still afterwards
}

// y the OTV drives at in lane i (lane center shifted left for the arm)
float laneY(int i) {
  return LANES[i] + LANE_SHIFT;
}

// Index of the lane whose driving line is closest to yNow
int nearestLane(float yNow) {
  int best = 0;
  for (int i = 1; i < 3; i++) {
    if (fabs(yNow - laneY(i)) < fabs(yNow - laneY(best))) best = i;
  }
  return best;
}

// Stop in each lane at SCAN_X, face +x, and record where the ultrasonic sees something
void scanLanes() {
  int order[3] = {0, 1, 2};
  if (y > LANES[1]) { order[0] = 2; order[2] = 0; }   // start from the end we are closer to
  for (int k = 0; k < 3; k++) {
    int i = order[k];
    moveToY(laneY(i), SCAN_X);
    turnTo(0);
    while (!readPose()) {}
    // 9 readings (middle of three medians), so bad readings can't fool the plan
    float d = median3(medianDistanceCM(), medianDistanceCM(), medianDistanceCM());
    hitX[i] = (d > 500) ? 99 : x + FRONT + d / 100.0;   // arena x of the hit = OTV x + sensor offset + distance
    Enes100.print("Nav2: lane "); Enes100.print(i);
    Enes100.print(" hit at x="); Enes100.println(hitX[i]);
  }
}

// Drive forward along a lane (holding its line) until x reaches xEnd
void driveLaneUntil(int lane, float xEnd) {
  while (true) {
    if (!readPose()) continue;
    if (x >= xEnd) break;
    while (medianDistanceCM() < SAFETY_CM) stopMotors();   // emergency stop: wait while something is right ahead
    driveToward(atan2(laneY(lane) - y, LOOKAHEAD));         // aim at a point ahead on the lane line
  }
  stopMotors();
}

// Distance in cm: the middle of 3 readings, so a single bad reading is ignored
float medianDistanceCM() {
  float a = readDistanceCM(); delay(10);
  float b = readDistanceCM(); delay(10);
  float c = readDistanceCM();
  return median3(a, b, c);
}

// Middle value of three numbers
float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

// One HC-SR04 reading in cm (999 = no echo, nothing ahead)
float readDistanceCM() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);    // 10 us pulse on TRIG starts a measurement
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long t = pulseIn(ECHO, HIGH, 30000);               // echo time in us (30 ms timeout, about 5 m)
  if (t == 0) return 999;
  return t * 0.0343 / 2;   // sound travels 0.0343 cm/us; divide by 2 for the round trip
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
