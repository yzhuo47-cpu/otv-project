// MS6 Navigation Objective 2: drive completely past the three obstacles.
// The obstacles sit in three rows (lanes). Drive along a lane; if the ultrasonic sensor sees
// an obstacle, switch to another lane and keep going.
#include "Enes100.h"

// ---------- Settings (check these before uploading) ----------
const int MARKER_ID = 0;        // TODO: your ArUco marker ID
const int ROOM      = 1116;     // TODO: 1116, 1120 or 1201
const int WIFI_TX   = 3;        // Wi-Fi module TX pin
const int WIFI_RX   = 2;        // Wi-Fi module RX pin

// Ultrasonic sensor (HC-SR04) on the front of the OTV
const int TRIG = 12;            // TODO: your TRIG pin
const int ECHO = 11;            // TODO: your ECHO pin

// Arena (meters)
const float LANES[3] = {0.5, 1.0, 1.5};   // center y of each obstacle row: bottom, middle, top
const float PAST_OBSTACLES_X = 2.9;       // stop here: whole OTV is past the last obstacle (tune on the arena)
const float LANE_TOL = 0.05;              // how close to the lane center is "in the lane"
const float LOOKAHEAD = 0.5;              // steer toward a point this far ahead in the lane

// Obstacle detection (cm)
const float OBSTACLE_CM = 15;   // obstacle closer than this while driving -> stop and switch lanes
                                // (close enough that moving sideways clears the obstacle behind us)
const float CHECK_CM    = 35;   // after switching, lane counts as blocked if obstacle closer than this

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
int lane;   // current lane index: 0 bottom, 1 middle, 2 top

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

  // Start in the nearest lane, facing +x
  lane = nearestLane(y);
  moveToY(LANES[lane]);
  turnTo(0);

  // Drive along the lane; switch lanes whenever an obstacle is ahead
  while (x < PAST_OBSTACLES_X) {
    if (!readPose()) continue;
    if (obstacleAhead(OBSTACLE_CM)) {
      stopMotors();
      switchLane();
    } else {
      driveToward(atan2(LANES[lane] - y, LOOKAHEAD));   // hold the lane center
    }
  }
  stopMotors();
  Enes100.println("Nav2: past all obstacles");
}

void loop() {
  stopMotors();  // done, stay still
}

// ---------- Obstacle avoidance ----------

// Try the other lanes (top lane first, since the limbo is on top) until one is clear
void switchLane() {
  int order[3] = {2, 1, 0};
  int tried = lane;
  for (int i = 0; i < 3; i++) {
    int next = order[i];
    if (next == lane) continue;
    moveToY(LANES[next]);
    turnTo(0);
    tried = next;
    if (!obstacleAhead(CHECK_CM)) {
      lane = next;
      Enes100.print("Nav2: switched to lane ");
      Enes100.println(lane);
      return;
    }
  }
  lane = tried;   // all blocked (should not happen): stay in the last lane tried
}

// Index of the lane closest to y
int nearestLane(float yNow) {
  int best = 0;
  for (int i = 1; i < 3; i++) {
    if (fabs(yNow - LANES[i]) < fabs(yNow - LANES[best])) best = i;
  }
  return best;
}

// Ultrasonic: true if something is closer than limitCM (middle of 3 readings, to ignore noise)
bool obstacleAhead(float limitCM) {
  float a = readDistanceCM(); delay(10);
  float b = readDistanceCM(); delay(10);
  float c = readDistanceCM();
  float mid = max(min(a, b), min(max(a, b), c));
  return mid < limitCM;
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

// Move straight up or down (keeping x) until reaching targetY
void moveToY(float targetY) {
  while (!readPose()) {}                        // wait until the marker is seen
  if (fabs(targetY - y) < LANE_TOL) return;     // already there
  float dir = (targetY > y) ? 1 : -1;
  float heading = dir * PI / 2;                 // +PI/2 = up, -PI/2 = down
  turnTo(heading);
  while (true) {
    if (!readPose()) continue;
    if ((targetY - y) * dir <= LANE_TOL) break; // reached (or just passed) targetY
    driveToward(heading);
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
