// MS6 Nav 2: drive completely past the three obstacles (two boxes and the rumbles, all avoided).
// The OTV first checks all 3 lanes with the ultrasonic, then plans one lane change between the columns.
#include "Enes100.h"

// ===== Settings: check before uploading =====
const int MARKER_ID = 147;    // ArUco ID on our OTV (must appear in the Vision System's "Seen markers")
const int ROOM      = 1201;   // room of the Vision System we connect to
const float MARKER_BACK = 0.06;  // m: marker center is this far behind the OTV center (toward the rear)

// ===== Ultrasonic sensor (HC-SR04, front center, facing forward) =====
const int TRIG = 12;
const int ECHO = 11;

// ===== OTV size (m): arm measured on the real OTV, the rest from the MS3 CAD =====
const float ARM_REACH  = 0.14;   // OTV center to farthest point of the arm/scoop (arm on the right, measured)
const float HALF_WIDTH = 0.115;  // OTV center to outside of the left wheels
const float TURN_R     = 0.17;   // OTV center to farthest wheel corner = room needed to turn in place
const float FRONT      = 0.10;   // OTV center to the ultrasonic sensor

// ===== Arena (m, from Arena Details) =====
const float OBS_DEPTH = 0.20;             // obstacle size along x (0.2 m x 0.5 m boxes)
const float LANES[3] = {0.5, 1.0, 1.5};   // center y of the 3 obstacle rows (lane 0, 1, 2)
const float COL1_X = 1.5, COL2_X = 2.3;  // center x of the 2 obstacle columns
const float COL_SPLIT = (COL1_X + COL2_X) / 2 - OBS_DEPTH / 2;   // 1.8: between the two front faces (1.4, 2.2)
const float SCAN_X = 1.05;                // where lanes are checked: after the landing zone, before column 1
const float PAST_OBSTACLES_X = 2.9;       // finish line: in the open zone (2.8-3.4), past column 2 (ends at 2.4)
const float LANE_TOL  = 0.03;             // within 3 cm of a lane line counts as in the lane
const float LOOKAHEAD = 0.5;              // steer toward a point 0.5 m ahead on the line (smooth correction)

// Arm reaches ARM_REACH to the right, wheels HALF_WIDTH to the left: shifting left by half the
// difference leaves the same room on both sides inside a 0.5 m lane
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

float x, y, theta;   // latest OTV center position (m) and heading (rad)
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

  // 2. Check all 3 lanes before entering the obstacles (plan first, then drive)
  scanLanes();

  // 3. Which column blocks each lane? Columns are fixed (x = 1.5 and 2.3), so a hit before COL_SPLIT
  //    is column 1 and a hit before PAST_OBSTACLES_X is column 2 (the limbo, log and wall are farther).
  int col[3];          // 1 or 2 = column of this lane's obstacle, 0 = nothing seen
  int n1 = 0, n2 = 0;  // how many obstacles we saw in column 1 and column 2
  for (int i = 0; i < 3; i++) {
    if (hitX[i] < COL_SPLIT) col[i] = 1;
    else if (hitX[i] < PAST_OBSTACLES_X) col[i] = 2;
    else col[i] = 0;
    if (col[i] == 1) n1++;
    if (col[i] == 2) n2++;
  }
  if (n1 + n2 < 2) {   // the two boxes are always visible, so fewer hits means a sensor problem
    Enes100.println("Nav2: saw fewer than 2 obstacles, check the ultrasonic");
    return;
  }

  // 4. Arena rules: one obstacle per row, two in one column and one in the other. The rumbles are low,
  //    so the ultrasonic may miss them: a lane with no hit is the rumbles lane. If both boxes are in the
  //    same column, the rumbles must be in the other one. If not, we don't know where the rumbles are,
  //    so that lane stays 0 and is never used.
  for (int i = 0; i < 3; i++) {
    if (col[i] == 0 && n1 == 2) col[i] = 2;
    if (col[i] == 0 && n2 == 2) col[i] = 1;
  }

  // 5. Every lane is blocked once, so switch lanes once in the gap between the columns:
  //    start in a lane blocked at column 2 (open at column 1), switch to one blocked at column 1
  //    (open at column 2). Pick the pair of lanes closest together.
  int first = -1, second = -1;
  for (int a = 0; a < 3; a++) {
    for (int b = 0; b < 3; b++) {
      if (col[a] != 2 || col[b] != 1) continue;
      if (first < 0 || abs(a - b) < abs(first - second)) { first = a; second = b; }
    }
  }
  if (first < 0) {   // readings don't fit the arena rules: stop instead of guessing
    Enes100.println("Nav2: no path found, check the ultrasonic");
    return;
  }

  // Where to switch lanes. Moving sideways, the arm points along x (forward when moving up, backward when
  // moving down). Give that side room for the arm (or the wheel corners if bigger), the other side TURN_R.
  float edge1 = COL1_X + OBS_DEPTH / 2;    // back face of column 1 (1.6)
  float face2 = COL2_X - OBS_DEPTH / 2;    // front face of column 2 (2.2)
  float armRoom = max(ARM_REACH, TURN_R);
  float crossX;
  if (second > first) crossX = (edge1 + TURN_R + face2 - armRoom) / 2;   // moving up: arm points forward
  else                crossX = (edge1 + armRoom + face2 - TURN_R) / 2;   // moving down: arm points back
  if (face2 - edge1 < armRoom + TURN_R) Enes100.println("Nav2: WARNING gap too small for the arm");

  // Drive lane "first" to the gap, move sideways to lane "second", then drive to the finish line
  Enes100.print("Nav2: lane "); Enes100.print(first);
  Enes100.print(" then "); Enes100.println(second);
  moveToY(laneY(first), SCAN_X);
  turnTo(0);
  driveLaneUntil(first, crossX);
  moveToY(laneY(second), crossX);
  turnTo(0);
  driveLaneUntil(second, PAST_OBSTACLES_X);

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
