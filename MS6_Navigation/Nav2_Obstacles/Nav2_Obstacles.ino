// MS6 Nav 2: drive past the three obstacles
#include "Enes100.h"

// Change before uploading
const int MARKER_ID = 0;
const int ROOM      = 1116;
const int WIFI_TX   = 3;
const int WIFI_RX   = 2;

// Ultrasonic sensor pins
const int TRIG = 12;
const int ECHO = 11;

// OTV size (m)
const float ARM_REACH  = 0.27;   // center to arm tip (arm on the right)
const float HALF_WIDTH = 0.115;  // center to outside of left wheels
const float TURN_R     = 0.17;   // center to farthest wheel corner
const float FRONT      = 0.10;   // center to ultrasonic sensor

// Arena (m)
const float LANES[3] = {0.5, 1.0, 1.5};   // y of the 3 obstacle rows
const float SCAN_X = 1.05;                // where the lanes are checked
const float OBS_DEPTH = 0.20;             // obstacle size along x
const float PAST_OBSTACLES_X = 2.9;       // past all obstacles
const float LANE_TOL  = 0.03;
const float LOOKAHEAD = 0.5;

// Drive left of lane center so the arm has room
const float LANE_SHIFT = (ARM_REACH - HALF_WIDTH) / 2;

const float SAFETY_CM = 6;       // emergency stop distance

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
float hitX[3];   // where each lane is blocked (99 = not blocked)

void setup() {
  // Set up motor and sensor pins
  pinMode(R_EN, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(L_EN, OUTPUT); pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  stopMotors();

  // Connect to the Vision System and wait for the marker
  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM, WIFI_TX, WIFI_RX);
  while (!readPose()) {}
  Enes100.println("Nav2: start obstacles");

  // 1. Drive to the scan point in the nearest lane
  int lane = nearestLane(y);
  moveToY(laneY(lane), x);
  turnTo(0);
  driveLaneUntil(lane, SCAN_X);

  // 2. The arm can't fold, so plan first: check all 3 lanes with the ultrasonic
  scanLanes();

  // 3. Find which lanes are blocked at the first and second obstacle column
  float col1X = min(hitX[0], min(hitX[1], hitX[2]));
  bool block1[3], block2[3];
  for (int i = 0; i < 3; i++) {
    bool hit = hitX[i] < PAST_OBSTACLES_X;
    block1[i] = hit && hitX[i] < col1X + OBS_DEPTH;   // blocked at column 1
    block2[i] = hit && !block1[i];                    // blocked at column 2
  }

  // 4. Look for a lane that is clear all the way (top lane first, the limbo is on top)
  int clear = -1;
  for (int i = 2; i >= 0; i--) {
    if (!block1[i] && !block2[i]) { clear = i; break; }
  }

  if (clear >= 0) {
    // 5a. Clear lane: drive straight through
    Enes100.print("Nav2: clear lane "); Enes100.println(clear);
    moveToY(laneY(clear), SCAN_X);
    turnTo(0);
    driveLaneUntil(clear, PAST_OBSTACLES_X);
  } else {
    // 5b. No clear lane: pick a lane open at column 1 and a lane open at column 2
    int first = -1, second = -1;
    for (int a = 0; a < 3; a++) {
      for (int b = 0; b < 3; b++) {
        if (block1[a] || !block1[b]) continue;
        if (first < 0 || abs(a - b) < abs(first - second)) { first = a; second = b; }
      }
    }
    if (first < 0) {   // scan makes no sense: stop
      Enes100.println("Nav2: no path found, check the ultrasonic");
      return;
    }

    // Stop between the columns, leaving room on the side the arm points to
    float edge1 = col1X + OBS_DEPTH;   // back of column 1
    float face2 = hitX[first];         // front of column 2
    float crossX;
    if (second > first) crossX = (edge1 + TURN_R + face2 - ARM_REACH) / 2;   // moving up: arm points forward
    else                crossX = (edge1 + ARM_REACH + face2 - TURN_R) / 2;   // moving down: arm points back
    if (face2 - edge1 < ARM_REACH + TURN_R) Enes100.println("Nav2: WARNING gap too small for the arm");

    // Drive the first lane, switch lanes in the gap, finish in the second lane
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
  stopMotors();  // done
}

// y to drive at in lane i (shifted left for the arm)
float laneY(int i) {
  return LANES[i] + LANE_SHIFT;
}

// Lane closest to the current y
int nearestLane(float yNow) {
  int best = 0;
  for (int i = 1; i < 3; i++) {
    if (fabs(yNow - laneY(i)) < fabs(yNow - laneY(best))) best = i;
  }
  return best;
}

// Stop in each lane, face forward, and record where the lane is blocked
void scanLanes() {
  int order[3] = {0, 1, 2};
  if (y > LANES[1]) { order[0] = 2; order[2] = 0; }   // start from the closer end
  for (int k = 0; k < 3; k++) {
    int i = order[k];
    moveToY(laneY(i), SCAN_X);
    turnTo(0);
    while (!readPose()) {}
    float d = median3(medianDistanceCM(), medianDistanceCM(), medianDistanceCM());   // 9 readings
    hitX[i] = (d > 500) ? 99 : x + FRONT + d / 100.0;
    Enes100.print("Nav2: lane "); Enes100.print(i);
    Enes100.print(" hit at x="); Enes100.println(hitX[i]);
  }
}

// Drive forward along a lane until x reaches xEnd
void driveLaneUntil(int lane, float xEnd) {
  while (true) {
    if (!readPose()) continue;
    if (x >= xEnd) break;
    while (medianDistanceCM() < SAFETY_CM) stopMotors();   // emergency stop
    driveToward(atan2(laneY(lane) - y, LOOKAHEAD));         // stay on the lane line
  }
  stopMotors();
}

// Distance in cm: middle of 3 readings, to ignore bad readings
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

// One ultrasonic reading in cm (999 = nothing ahead)
float readDistanceCM() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long t = pulseIn(ECHO, HIGH, 30000);
  if (t == 0) return 999;
  return t * 0.0343 / 2;   // echo time to distance
}

// Move straight up or down to targetY, staying at x = holdX
void moveToY(float targetY, float holdX) {
  while (!readPose()) {}
  if (fabs(targetY - y) < LANE_TOL) return;   // already there
  float dir = (targetY > y) ? 1 : -1;
  float heading = dir * PI / 2;               // face up or down
  turnTo(heading);
  while (true) {
    if (!readPose()) continue;
    if ((targetY - y) * dir <= LANE_TOL) break;
    driveToward(atan2(dir * LOOKAHEAD, holdX - x));   // steer back to holdX if drifting
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
