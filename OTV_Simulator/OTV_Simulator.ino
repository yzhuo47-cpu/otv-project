// OTV navigation - ENES100 web simulator version (enes100.umd.edu/tools/simulator)
// The Harvesters, mission: Seed Planting. How to run: see README.md.

// Libraries: Enes100 = vision system (position, heading, printing),
// Tank = the simulator's robot (motors and sonar).
#include <cmath>
#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"

// ======================== SETTINGS ========================
// Plain variables, not #define: the simulator mishandles
// "#define NAME value // comment" lines.

// Speed and accuracy (simulator: speed = PWM/510 m/s)
int   DRIVE_PWM   = 150;    // driving speed, ~0.29 m/s
int   TURN_PWM    = 80;     // turning speed, ~36 deg/s
float STEP_M      = 0.12;   // longest drive before checking position again (m)
float ANGLE_TOL   = 0.035;  // heading counts as correct within ~2 deg
float ARRIVE_TOL  = 0.015;  // point counts as reached within 1.5 cm

// Robot size (simulator robot: 15 x 12 cm)
float HALF_LEN     = 0.075; // robot center to front
float SONAR_OFFSET = 0.075; // sonar is at the front

// Obstacles: 3 lanes (y) x 2 columns (x), in meters
int   LANES = 3;                    // number of lanes
int   COLS  = 2;                    // number of obstacle columns
float LANE_Y[3] = {0.5, 1.0, 1.5};  // lane centers
float COL_X[2]  = {1.5, 2.3};       // column centers
float OBS_DEPTH    = 0.2;   // obstacle thickness in x
float PREF_GAP     = 0.15;  // gap between robot front and column when checking
float BLOCK_MARGIN = 0.15;  // reading this far past a column face still = blocked
float EXIT_MARGIN  = 0.10;  // extra distance driven past the last column

// Mission site = the landing spot we did NOT start on (not drawn in the
// simulator). The robot parks beside it with the arm side facing it.
float SITE_X        = 0.55; // site center x
float SITE_Y_TOP    = 1.45; // top site center y
float SITE_Y_BOT    = 0.55; // bottom site center y
float SITE_STANDOFF = 0.20; // parking spot: this far from the site center (y)
float SITE_LEADIN   = 0.25; // straight run into the parking spot
float SITE_EXIT_X   = 0.92; // x to reach when leaving the site
bool  ARM_ON_RIGHT  = true; // arm is on the robot's right side

// Limbo
float LIMBO_APPROACH_X = 3.0;  // stop point right before the limbo and log (MS5)
float LIMBO_Y          = 1.5;  // center of the limbo opening (y 1.0 - 2.0)
float GOAL_X           = 3.75; // finish point past the limbo

// Possible results of one sonar check
int LV_BLOCKED = 0;  // obstacle in this column -> change lane
int LV_NEXT    = 1;  // clear here, obstacle in the next column
int LV_THROUGH = 2;  // clear in both columns

// ======================== STATE ========================
float px, py, pth;      // latest position (m) and heading (rad) from the vision system
unsigned long startMs;  // start time, for printing elapsed time
float overheadMs = 30;  // how long the robot keeps moving after a stop command (learned)
bool siteOnTop;         // true = mission site is the top landing spot
int lane;               // current lane (0, 1, 2)
bool tried[3];          // lanes already checked at the current column

// ======================== HELPERS ========================

// Read position and heading from the vision system (received wirelessly).
void readPose() {
  px = Enes100.getX();      // x in meters
  py = Enes100.getY();      // y in meters
  pth = Enes100.getTheta(); // heading in radians (0 = facing +x, toward the limbo)
}

// Keep an angle between -PI and PI, so the robot always turns the short way.
float wrapAngle(float a) {
  while (a > PI) a = a - TWO_PI;   // too big: subtract a full turn
  while (a < -PI) a = a + TWO_PI;  // too small: add a full turn
  return a;
}

// Print elapsed time (s) to the console.
void sayTime() {
  Enes100.print("   t = ");
  Enes100.print((millis() - startMs) / 1000.0);
  Enes100.println(" s");
}

// Set wheel speeds (negative = backward). Returns extra ms to wait.
// The left wheel is set first unless it goes backward (that can break the
// simulator's position); setting the right wheel first adds a 110 ms delay.
int setMotors(int left, int right) {
  if (left >= 0) {                 // left wheel forward or stopped: set it first
    Tank.setLeftMotorPWM(left);
    Tank.setRightMotorPWM(right);
    return 0;                      // no extra delay
  }
  Tank.setRightMotorPWM(right);    // left wheel backward: set the right one first
  Tank.setLeftMotorPWM(left);
  return 110;                      // simulator's start delay (ms)
}

// Turn both wheels off.
void stopMotors() {
  Tank.turnOffMotors();
}

// ======================== MOVEMENT ========================
// The robot moves in short steps: move, stop, read the camera, correct.
// Each step's time is planned from the speed, minus overheadMs.

// Update overheadMs from how much extra the last move went (averaged).
void learnOverhead(float extraMs) {
  if (extraMs < 0) extraMs = 0;                   // ignore impossible values
  if (extraMs > 3000) extraMs = 3000;
  overheadMs = 0.7 * overheadMs + 0.3 * extraMs;  // 70% old + 30% new, smooths out noise
}

// Turn in place to face "target" (rad): turn, stop, check, repeat
// until within ANGLE_TOL. Returns false if it gives up after 30 tries.
bool turnTo(float target) {
  float maxRate = 2.0 * TURN_PWM / 255.0;  // fastest turn, rad/s
  for (int i = 0; i < 30; i++) {           // at most 30 tries
    readPose();
    float err = wrapAngle(target - pth);   // angle still to turn (+ = left)
    if (fabs(err) < ANGLE_TOL) return true;  // close enough: done
    // small error -> slower turn, so the overshoot stays small
    float rate = fabs(err) / (2.0 * overheadMs / 1000.0);
    if (rate > maxRate) rate = maxRate;
    int pwm = (int)(rate * 255.0 / 2.0);   // turn rate -> PWM
    if (pwm < 3) pwm = 3;                  // minimum PWM so it still moves
    rate = 2.0 * pwm / 255.0;              // actual rate for that PWM
    // turn time = angle / speed, minus the expected overshoot
    int ms = (int)(fabs(err) / rate * 1000.0 - overheadMs);
    if (ms < 1) ms = 1;
    float before = pth;                    // heading before turning
    int wait = 0;
    if (err > 0) wait = setMotors(-pwm, pwm);   // turn left (counter-clockwise)
    else         wait = setMotors(pwm, -pwm);   // turn right
    delay(ms + wait);                      // turn for the planned time
    stopMotors();
    // how much longer it really turned than planned -> learnOverhead
    readPose();
    learnOverhead(fabs(wrapAngle(pth - before)) / rate * 1000.0 - ms);
  }
  return false;
}

// Drive to point (tx, ty): aim, drive one short step, check, repeat.
// Done when less than ARRIVE_TOL is left along the start-to-target line.
bool driveTo(float tx, float ty) {
  readPose();
  float sx = px;                // start point of this leg
  float sy = py;
  float len = sqrt((tx - sx) * (tx - sx) + (ty - sy) * (ty - sy));  // leg length (m)
  if (len < ARRIVE_TOL) return true;  // already there
  float ux = (tx - sx) / len;   // direction of this leg (unit vector)
  float uy = (ty - sy) / len;
  float maxSpeed = DRIVE_PWM / 255.0 / 2.0;  // m/s
  for (int i = 0; i < 400; i++) {            // at most 400 steps
    readPose();
    float dx = tx - px;                      // vector from robot to target
    float dy = ty - py;
    float along = dx * ux + dy * uy;         // distance left along the leg
    if (along < ARRIVE_TOL) return true;     // arrived
    // re-aim at the target (very close: atan2 jumps around, so keep the leg direction)
    float want = atan2(dy, dx);              // heading that points at the target
    if (along < 0.05) want = atan2(uy, ux);
    if (fabs(wrapAngle(want - pth)) > ANGLE_TOL) turnTo(want);  // off by > 2 deg: turn first
    // step length and speed (short steps go slower)
    float step = along;
    if (step > STEP_M) step = STEP_M;        // at most 12 cm per step
    float speed = step / (2.0 * overheadMs / 1000.0);
    if (speed > maxSpeed) speed = maxSpeed;
    int pwm = (int)(speed * 2.0 * 255.0);    // speed -> PWM
    if (pwm < 3) pwm = 3;                    // minimum PWM so it still moves
    speed = pwm / 255.0 / 2.0;               // actual speed for that PWM
    // drive time = distance / speed, minus the expected overshoot
    int ms = (int)(step / speed * 1000.0 - overheadMs);
    if (ms < 1) ms = 1;
    readPose();
    float bx = px;                           // position before the step
    float by = py;
    int wait = setMotors(pwm, pwm);          // both wheels forward
    delay(ms + wait);                        // drive for the planned time
    stopMotors();
    // how much further it really drove than planned -> learnOverhead
    readPose();
    learnOverhead(sqrt((px - bx) * (px - bx) + (py - by) * (py - by)) / speed * 1000.0 - ms);
  }
  return false;
}

// Back up about "meters" (timed only, no position check).
void reverseBy(float meters) {
  float speed = DRIVE_PWM / 255.0 / 2.0;                 // m/s at DRIVE_PWM
  int ms = (int)(meters / speed * 1000.0 - overheadMs);  // time = distance / speed, minus overshoot
  if (ms < 1) ms = 1;
  int wait = setMotors(-DRIVE_PWM, -DRIVE_PWM);          // both wheels backward
  delay(ms + wait);
  stopMotors();
}

// ======================== OBSTACLES ========================
// Obstacles stand in 2 columns. At each column the robot stops at a
// checkpoint, faces +x and reads the ultrasonic sensor. If its lane is
// blocked, it slides sideways to another lane and checks again.

// Front and back x of column c.
float colFront(int c) { return COL_X[c] - OBS_DEPTH / 2; }
float colBack(int c)  { return COL_X[c] + OBS_DEPTH / 2; }

// x where the robot stops to check column c: just in front of it,
// or halfway between the two columns if that is further forward.
float checkpointX(int c) {
  float x = colFront(c) - HALF_LEN - PREF_GAP;       // robot front stops PREF_GAP before the column
  if (c > 0) {                                       // not the first column:
    float mid = (colBack(c - 1) + colFront(c)) / 2;  // halfway between the two columns
    if (mid > x) x = mid;                            // don't stop too close to the previous column
  }
  return x;
}

// Lane closest to y.
int nearestLane(float y) {
  int best = 0;
  for (int i = 1; i < LANES; i++) {
    if (fabs(LANE_Y[i] - y) < fabs(LANE_Y[best] - y)) best = i;  // found a closer lane
  }
  return best;
}

// Next lane to try: the closest one not checked yet; on a tie, the one
// nearer the limbo. Returns -1 if every lane was checked.
int pickLane(int from) {
  int best = -1;                       // -1 = none found yet
  for (int i = 0; i < LANES; i++) {
    if (tried[i]) continue;            // skip lanes already checked
    if (best < 0) {
      best = i;                        // first unchecked lane
    } else {
      int dNew = i - from;             // how many lanes away (made positive below)
      if (dNew < 0) dNew = -dNew;
      int dOld = best - from;
      if (dOld < 0) dOld = -dOld;
      if (dNew < dOld) best = i;       // closer lane wins
      if (dNew == dOld && fabs(LANE_Y[i] - LIMBO_Y) < fabs(LANE_Y[best] - LIMBO_Y)) best = i;  // tie: nearer the limbo wins
    }
  }
  return best;
}

// Read the sonar down the lane. The column positions are known, so the
// distance shows whether the obstacle is in this column, the next one,
// or not in this lane at all.
int lookDownLane(int c) {
  readPose();
  float d = Tank.readDistanceSensor(1);  // forward sonar, meters, -1 = nothing within 1 m
  Enes100.print("   sonar reading in m: ");
  Enes100.println(d);
  if (d < 0) return LV_THROUGH;          // nothing seen: lane is clear
  float sensorX = px + SONAR_OFFSET;     // x of the sonar
  if (d < colFront(c) - sensorX + BLOCK_MARGIN) return LV_BLOCKED;  // something at this column
  if (c + 1 < COLS) {
    if (d < colFront(c + 1) - sensorX + BLOCK_MARGIN) return LV_NEXT;  // something at the next column
  }
  return LV_THROUGH;                     // whatever it saw is past both columns
}

// Get past both columns, then drive on until the last one is cleared.
void crossObstacles() {
  for (int c = 0; c < COLS; c++) {                     // one column at a time
    float cx = checkpointX(c);                         // where to stop for this column
    for (int i = 0; i < LANES; i++) tried[i] = false;  // no lanes checked yet here
    driveTo(cx, LANE_Y[lane]);                         // go to the checkpoint in the current lane
    for (int k = 0; k < 2 * LANES; k++) {              // at most 6 checks per column
      turnTo(0);                                       // face +x so the sonar looks down the lane
      Enes100.print("   column ");                     // print which column and lane we check
      Enes100.print(c + 1);
      Enes100.print("   lane y = ");
      Enes100.println(LANE_Y[lane]);
      int view = lookDownLane(c);
      tried[lane] = true;                              // this lane is now checked
      if (view != LV_BLOCKED) break;  // clear here; next column is checked at its own checkpoint
      int to = pickLane(lane);                         // lane to move to
      if (to < 0) {                   // every lane looked blocked: check them again
        Enes100.println("   all lanes blocked? checking again");
        for (int i = 0; i < LANES; i++) tried[i] = false;
        tried[lane] = true;
        to = pickLane(lane);
      }
      Enes100.println("   blocked -> changing lane");
      driveTo(cx, LANE_Y[to]);        // slide sideways at the checkpoint
      lane = to;
    }
  }
  // drive on until the robot's back is past the last column
  driveTo(colBack(COLS - 1) + HALF_LEN + EXIT_MARGIN, LANE_Y[lane]);
}

// ======================== MISSION SITE ========================

// Parking heading: the arm side must face the site.
float siteHeading() {
  if (siteOnTop == ARM_ON_RIGHT) return PI;  // face -x
  return 0;                                  // face +x
}

// Drive to a lead-in point, then straight into the parking spot beside
// the site, then turn to the parking heading.
void goToSite() {
  float stopY = SITE_Y_BOT + SITE_STANDOFF;   // park just above the bottom site
  if (siteOnTop) {
    stopY = SITE_Y_TOP - SITE_STANDOFF;       // or just below the top site
  }
  float h = siteHeading();                        // parking heading
  driveTo(SITE_X - SITE_LEADIN * cos(h), stopY);  // lead-in point, behind the spot along h
  driveTo(SITE_X, stopY);                         // straight in, already facing h
  turnTo(h);                                      // final heading correction
}

// Placeholder for the arm + IR code (plant seed, collect rocks, classify
// the far plots). The simulator cannot simulate the arm.
void doSiteTask() {
  Enes100.println("   [site] arm/IR task would run here");
  delay(2000);                          // pause in place of the real arm task
}

// Leave the site along x (forward or backward, depending on the parking
// heading) before turning toward the obstacles.
void leaveSite() {
  readPose();
  if (cos(siteHeading()) > 0) {          // cos > 0 means parked facing +x
    driveTo(SITE_EXIT_X, py);            // facing +x: drive forward
  } else {
    reverseBy(SITE_EXIT_X - px);         // facing -x: back up, no turn needed
  }
}

// ======================== MAIN PROGRAM ========================
// Runs once: mission site -> obstacles -> limbo, printing each stage and the time.
void setup() {
  // connect to the vision system: team name, mission, marker ID, room, WiFi TX/RX pins
  Enes100.begin("OTV Team", SEED, 3, 1116, 8, 9);
  Tank.begin();  // simulator motors and sonar
  startMs = millis();  // start the clock

  // the start spot decides the mission site: started low -> site is on top
  readPose();
  siteOnTop = (py < 1.0);
  Enes100.print("Start x = ");           // print start position and chosen site
  Enes100.print(px);
  Enes100.print("   y = ");
  Enes100.println(py);
  if (siteOnTop) Enes100.println("Mission site: TOP");
  else Enes100.println("Mission site: BOTTOM");

  Enes100.println("== 1. Landing zone -> mission site");
  goToSite();
  sayTime();
  Enes100.println("== 2. At the mission site");
  doSiteTask();
  leaveSite();

  Enes100.println("== 3. Obstacles");
  lane = nearestLane(py);               // start in the lane closest to the robot
  crossObstacles();
  sayTime();

  Enes100.println("== 4. Right before the limbo and log");
  driveTo(LIMBO_APPROACH_X, LIMBO_Y);   // line up with the middle of the limbo opening
  turnTo(0);                            // face the limbo (+x)
  sayTime();

  Enes100.println("== 5. Through the limbo");
  driveTo(GOAL_X, LIMBO_Y);             // straight through the limbo
  stopMotors();                         // make sure the robot is stopped
  sayTime();
  Enes100.println("== DONE");
}

// Nothing to repeat: the whole run happens once in setup().
void loop() {
  delay(1000);
}
