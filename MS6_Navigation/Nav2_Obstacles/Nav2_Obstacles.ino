// MS6 Navigation Objective 2: drive completely past the three obstacles.
// The arm sticks out on the right side and can't fold, so the OTV plans before the obstacles:
// it looks down all 3 lanes, then takes a lane that is clear the whole way. If no lane is clear,
// it changes lanes once, in the gap between the two obstacle columns.
#include "Enes100.h"

// ---------- Settings (check these before uploading) ----------
const int MARKER_ID = 0;        // TODO: your ArUco marker ID
const int ROOM      = 1116;     // TODO: 1116, 1120 or 1201
const int WIFI_TX   = 3;        // Wi-Fi module TX pin
const int WIFI_RX   = 2;        // Wi-Fi module RX pin

// Ultrasonic sensor (HC-SR04) on the front of the OTV
const int TRIG = 12;
const int ECHO = 11;

// OTV size (meters, from the MS3 CAD - measure the real OTV)
const float ARM_REACH  = 0.27;   // OTV center to arm tip (arm is on the right side)
const float HALF_WIDTH = 0.115;  // OTV center to outside of the left wheels
const float TURN_R     = 0.17;   // OTV center to farthest wheel corner (room needed to turn)
const float FRONT      = 0.10;   // OTV center to the ultrasonic sensor

// Arena (meters)
const float LANES[3] = {0.5, 1.0, 1.5};   // center y of each obstacle row: bottom, middle, top
const float SCAN_X = 1.05;                // where the OTV looks down the lanes (needs ARM_REACH of room in front and behind)
const float OBS_DEPTH = 0.20;             // obstacle size along x
const float PAST_OBSTACLES_X = 2.9;       // stop here: whole OTV is past the last obstacle (tune on the arena)
const float LANE_TOL  = 0.03;             // how close to the lane line is "in the lane"
const float LOOKAHEAD = 0.5;              // steer toward a point this far ahead in the lane

// Drive a bit left of the lane center so the arm (right) and the left wheels have equal room
const float LANE_SHIFT = (ARM_REACH - HALF_WIDTH) / 2;

const float SAFETY_CM = 6;      // something this close while driving -> stop and wait (keep small, see README)

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
float hitX[3];   // x where the ultrasonic saw something in each lane (99 = nothing)

void setup() {
  // Motor and sensor pins
  pinMode(L_EN, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_EN, OUTPUT); pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  stopMotors();

  // Connect to the Vision System and wait until the marker is seen
  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM, WIFI_TX, WIFI_RX);
  while (!readPose()) {}
  Enes100.println("Nav2: start obstacles");

  // 1. Drive to SCAN_X in the nearest lane
  int lane = nearestLane(y);
  moveToY(laneY(lane), x);
  turnTo(0);
  driveLaneUntil(lane, SCAN_X);

  // 2. Look down all 3 lanes
  scanLanes();

  // 3. Sort what we saw: first column = nearest hit; anything before PAST_OBSTACLES_X is an obstacle
  float col1X = min(hitX[0], min(hitX[1], hitX[2]));
  bool block1[3], block2[3];
  for (int i = 0; i < 3; i++) {
    bool hit = hitX[i] < PAST_OBSTACLES_X;
    block1[i] = hit && hitX[i] < col1X + OBS_DEPTH;   // blocked at the first column
    block2[i] = hit && !block1[i];                    // open at first column, blocked at second
  }

  // 4a. A lane clear the whole way? (top lane first, since the limbo is on top)
  int clear = -1;
  for (int i = 2; i >= 0; i--) {
    if (!block1[i] && !block2[i]) { clear = i; break; }
  }

  if (clear >= 0) {
    Enes100.print("Nav2: clear lane "); Enes100.println(clear);
    moveToY(laneY(clear), SCAN_X);
    turnTo(0);
    driveLaneUntil(clear, PAST_OBSTACLES_X);
  } else {
    // 4b. No clear lane: start in a lane open at column 1, switch to a lane open at column 2
    int first = -1, second = -1;
    for (int a = 0; a < 3; a++) {
      for (int b = 0; b < 3; b++) {
        if (block1[a] || !block1[b]) continue;                 // a open at col 1, b open at col 2
        if (first < 0 || abs(a - b) < abs(first - second)) { first = a; second = b; }
      }
    }
    if (first < 0) {                      // scan didn't make sense: stop safely
      Enes100.println("Nav2: no path found, check the ultrasonic");
      return;
    }

    // Where to cross: between the columns, leaving ARM_REACH on the side the arm points to
    float edge1 = col1X + OBS_DEPTH;      // back of column 1
    float face2 = hitX[first];            // front of column 2
    float crossX;
    if (second > first) crossX = (edge1 + TURN_R + face2 - ARM_REACH) / 2;   // moving up: arm points +x
    else                crossX = (edge1 + ARM_REACH + face2 - TURN_R) / 2;   // moving down: arm points -x
    if (face2 - edge1 < ARM_REACH + TURN_R) Enes100.println("Nav2: WARNING gap too small for the arm");

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
  stopMotors();  // done, stay still
}

// ---------- Lanes ----------

// y the OTV drives at in lane i (shifted left to make room for the arm)
float laneY(int i) {
  return LANES[i] + LANE_SHIFT;
}

// Index of the lane closest to y
int nearestLane(float yNow) {
  int best = 0;
  for (int i = 1; i < 3; i++) {
    if (fabs(yNow - laneY(i)) < fabs(yNow - laneY(best))) best = i;
  }
  return best;
}

// Visit each lane at SCAN_X, face +x, and record where the ultrasonic sees something
void scanLanes() {
  int order[3] = {0, 1, 2};
  if (y > LANES[1]) { order[0] = 2; order[2] = 0; }   // start with the closest end
  for (int k = 0; k < 3; k++) {
    int i = order[k];
    moveToY(laneY(i), SCAN_X);
    turnTo(0);
    while (!readPose()) {}
    float d = median3(medianDistanceCM(), medianDistanceCM(), medianDistanceCM());   // extra careful: 9 readings
    hitX[i] = (d > 500) ? 99 : x + FRONT + d / 100.0;
    Enes100.print("Nav2: lane "); Enes100.print(i);
    Enes100.print(" hit at x="); Enes100.println(hitX[i]);
  }
}

// Drive along lane i (facing +x) until x reaches xEnd
void driveLaneUntil(int lane, float xEnd) {
  while (true) {
    if (!readPose()) continue;
    if (x >= xEnd) break;
    while (medianDistanceCM() < SAFETY_CM) stopMotors();   // something in the way: wait
    driveToward(atan2(laneY(lane) - y, LOOKAHEAD));         // hold the lane line
  }
  stopMotors();
}

// ---------- Ultrasonic ----------

// Middle of 3 readings in cm, to ignore bad readings
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
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long t = pulseIn(ECHO, HIGH, 30000);
  if (t == 0) return 999;
  return t * 0.0343 / 2;
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
