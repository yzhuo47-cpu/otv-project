// =====================================================================
//  OTV navigation - ENES100 WEB SIMULATOR version
//  (enes100.umd.edu/tools/simulator)
//
//  Same strategy as the real-robot code in navigation/ :
//    landing zone -> park sideways next to the mission site -> leave ->
//    check each obstacle column from a checkpoint with the ultrasonic ->
//    change lane if blocked -> line up before the limbo -> through it.
//
//  Differences, because the simulator is not the real robot:
//    - one file, plain C++ (the simulator interprets a subset of C++)
//    - motors and sonar go through the Tank library
//    - the simulator's robot is fast (0.5 m/s at PWM 255) but the code runs
//      one step at a time, so it moves in short "drive, stop, measure"
//      steps instead of steering continuously
//    - arena numbers are the simulator's: obstacle columns at x 1.4-1.6
//      and 2.2-2.4, robot 15 x 12 cm, landing spots y = 0.55 / 1.45
//
//  How to run: paste into the simulator editor, set the speed button to
//  "instant" (three chevrons - slower settings also work, just much slower),
//  press Run, then press Continue (the program pauses on its first line).
//  Use "Randomize" for a new layout and start position.
// =====================================================================
#include <cmath>
#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"

// ---------------- settings
// (plain global variables: the simulator's preprocessor mishandles
//  "#define NAME value // comment" lines)

// speeds - simulator: v = PWM/510 m/s, turning = 2*PWM/255 rad/s
int   DRIVE_PWM   = 150;    // ~0.29 m/s
int   TURN_PWM    = 80;     // ~0.63 rad/s (36 deg/s)
float STEP_M      = 0.12;   // drive at most this far before re-measuring
float ANGLE_TOL   = 0.035;  // ~2 degrees
float ARRIVE_TOL  = 0.015;  // stop when less than 1.5 cm is left

// simulator robot
float HALF_LEN     = 0.075; // center -> front (robot is 0.15 m long)
float SONAR_OFFSET = 0.075; // the simulator measures from the robot front

// arena (simulator)
int   LANES = 3;
int   COLS  = 2;
float LANE_Y[3] = {0.5, 1.0, 1.5};
float COL_X[2]  = {1.5, 2.3};
float OBS_DEPTH    = 0.2;   // obstacle size along x
float PREF_GAP     = 0.15;  // room between robot front and a column at a checkpoint
float BLOCK_MARGIN = 0.15;  // sonar reading up to this much past a column face = blocked
float EXIT_MARGIN  = 0.10;

// Mission site: the other landing spot (x 0.55, y 0.55 or 1.45). It is not
// drawn in the simulator; the robot parks sideways next to where it is.
float SITE_X        = 0.55;
float SITE_Y_TOP    = 1.45;
float SITE_Y_BOT    = 0.55;
float SITE_STANDOFF = 0.20; // robot center this far from the site center (in y)
float SITE_LEADIN   = 0.25; // last straight stretch into the parking spot
float SITE_EXIT_X   = 0.92; // leave the site along x up to here before turning
bool  ARM_ON_RIGHT  = true; // arm sticks out of the robot's right side

float LIMBO_APPROACH_X = 3.0;  // "a point right before the limbo and log" (MS5)
float LIMBO_Y          = 1.5;  // middle of the limbo opening (y 1.0 - 2.0)
float GOAL_X           = 3.75;

// lane check results
int LV_BLOCKED = 0;  // blocked at this column
int LV_NEXT    = 1;  // free here, blocked at the next column
int LV_THROUGH = 2;  // free here and at the next column

// ---------------- state
float px, py, pth;          // latest pose from the vision system
unsigned long startMs;
// The robot keeps moving while the lines between "start motors" and "stop"
// run. How long that is depends on the simulator's speed setting, so the
// program measures it after every move and learns it.
float overheadMs = 30;
bool siteOnTop;
int lane;
bool tried[3];

void readPose() {
  px = Enes100.getX();
  py = Enes100.getY();
  pth = Enes100.getTheta();
}

float wrapAngle(float a) {
  while (a > PI) a = a - TWO_PI;
  while (a < -PI) a = a + TWO_PI;
  return a;
}

void sayTime() {
  Enes100.print("   t = ");
  Enes100.print((millis() - startMs) / 1000.0);
  Enes100.println(" s");
}

// Starts the motors and returns how many ms the simulator waits before the
// robot actually moves. The wheel with the non-negative PWM is set first:
// the simulator can briefly run only the first wheel, and if that one went
// backwards over a rumble strip the simulator's position would become NaN.
// Setting the right wheel first costs the simulator's 110 ms start delay.
int setMotors(int left, int right) {
  if (left >= 0) {
    Tank.setLeftMotorPWM(left);
    Tank.setRightMotorPWM(right);
    return 0;
  }
  Tank.setRightMotorPWM(right);
  Tank.setLeftMotorPWM(left);
  return 110;
}

void stopMotors() {
  Tank.turnOffMotors();
}

// ---------------- movement
void learnOverhead(float extraMs) {
  if (extraMs < 0) extraMs = 0;
  if (extraMs > 3000) extraMs = 3000;
  overheadMs = 0.7 * overheadMs + 0.3 * extraMs;
}

// Turn on the spot to face target (radians): rotate for the time the error
// needs, stop, measure again. Small errors use a slower turn so the extra
// motion from the overhead stays small.
bool turnTo(float target) {
  float maxRate = 2.0 * TURN_PWM / 255.0;  // rad/s
  for (int i = 0; i < 30; i++) {
    readPose();
    float err = wrapAngle(target - pth);
    if (fabs(err) < ANGLE_TOL) return true;
    float rate = fabs(err) / (2.0 * overheadMs / 1000.0);
    if (rate > maxRate) rate = maxRate;
    int pwm = (int)(rate * 255.0 / 2.0);
    if (pwm < 3) pwm = 3;
    rate = 2.0 * pwm / 255.0;
    int ms = (int)(fabs(err) / rate * 1000.0 - overheadMs);
    if (ms < 1) ms = 1;
    float before = pth;
    int wait = 0;
    if (err > 0) wait = setMotors(-pwm, pwm);   // left (counter-clockwise)
    else         wait = setMotors(pwm, -pwm);   // right
    delay(ms + wait);
    stopMotors();
    readPose();
    learnOverhead(fabs(wrapAngle(pth - before)) / rate * 1000.0 - ms);
  }
  return false;
}

// Drive to (tx, ty) in short straight steps, re-aiming after each step.
// "Arrived" = less than ARRIVE_TOL left along the line from start to target.
bool driveTo(float tx, float ty) {
  readPose();
  float sx = px;
  float sy = py;
  float len = sqrt((tx - sx) * (tx - sx) + (ty - sy) * (ty - sy));
  if (len < ARRIVE_TOL) return true;
  float ux = (tx - sx) / len;
  float uy = (ty - sy) / len;
  float maxSpeed = DRIVE_PWM / 255.0 / 2.0;  // m/s
  for (int i = 0; i < 400; i++) {
    readPose();
    float dx = tx - px;
    float dy = ty - py;
    float along = dx * ux + dy * uy;
    if (along < ARRIVE_TOL) return true;
    float want = atan2(dy, dx);
    if (along < 0.05) want = atan2(uy, ux);  // very close: keep the leg direction
    if (fabs(wrapAngle(want - pth)) > ANGLE_TOL) turnTo(want);
    float step = along;
    if (step > STEP_M) step = STEP_M;
    float speed = step / (2.0 * overheadMs / 1000.0);
    if (speed > maxSpeed) speed = maxSpeed;
    int pwm = (int)(speed * 2.0 * 255.0);
    if (pwm < 3) pwm = 3;
    speed = pwm / 255.0 / 2.0;
    int ms = (int)(step / speed * 1000.0 - overheadMs);
    if (ms < 1) ms = 1;
    readPose();
    float bx = px;
    float by = py;
    int wait = setMotors(pwm, pwm);
    delay(ms + wait);
    stopMotors();
    readPose();
    learnOverhead(sqrt((px - bx) * (px - bx) + (py - by) * (py - by)) / speed * 1000.0 - ms);
  }
  return false;
}

void reverseBy(float meters) {
  float speed = DRIVE_PWM / 255.0 / 2.0;
  int ms = (int)(meters / speed * 1000.0 - overheadMs);
  if (ms < 1) ms = 1;
  int wait = setMotors(-DRIVE_PWM, -DRIVE_PWM);
  delay(ms + wait);
  stopMotors();
}

// ---------------- obstacle columns
float colFront(int c) { return COL_X[c] - OBS_DEPTH / 2; }
float colBack(int c)  { return COL_X[c] + OBS_DEPTH / 2; }

// Where the robot center parks (x) to look at column c.
float checkpointX(int c) {
  float x = colFront(c) - HALF_LEN - PREF_GAP;
  if (c > 0) {
    float mid = (colBack(c - 1) + colFront(c)) / 2;
    if (mid > x) x = mid;
  }
  return x;
}

int nearestLane(float y) {
  int best = 0;
  for (int i = 1; i < LANES; i++) {
    if (fabs(LANE_Y[i] - y) < fabs(LANE_Y[best] - y)) best = i;
  }
  return best;
}

// Closest lane not tried yet; ties -> the lane nearer the limbo. -1 if none.
int pickLane(int from) {
  int best = -1;
  for (int i = 0; i < LANES; i++) {
    if (tried[i]) continue;
    if (best < 0) {
      best = i;
    } else {
      int dNew = i - from;
      if (dNew < 0) dNew = -dNew;
      int dOld = best - from;
      if (dOld < 0) dOld = -dOld;
      if (dNew < dOld) best = i;
      if (dNew == dOld && fabs(LANE_Y[i] - LIMBO_Y) < fabs(LANE_Y[best] - LIMBO_Y)) best = i;
    }
  }
  return best;
}

// Facing +x at column c's checkpoint: what does the sonar see?
// The map says where both columns are, so one reading tells us whether
// the block is in this column, in the next one, or not in this lane.
int lookDownLane(int c) {
  readPose();
  float d = Tank.readDistanceSensor(1);  // meters, -1 = nothing within 1 m
  Enes100.print("   sonar reading in m: ");
  Enes100.println(d);
  if (d < 0) return LV_THROUGH;
  float sensorX = px + SONAR_OFFSET;
  if (d < colFront(c) - sensorX + BLOCK_MARGIN) return LV_BLOCKED;
  if (c + 1 < COLS) {
    if (d < colFront(c + 1) - sensorX + BLOCK_MARGIN) return LV_NEXT;
  }
  return LV_THROUGH;
}

void crossObstacles() {
  for (int c = 0; c < COLS; c++) {
    float cx = checkpointX(c);
    for (int i = 0; i < LANES; i++) tried[i] = false;
    driveTo(cx, LANE_Y[lane]);
    for (int k = 0; k < 2 * LANES; k++) {
      turnTo(0);
      Enes100.print("   column ");
      Enes100.print(c + 1);
      Enes100.print("   lane y = ");
      Enes100.println(LANE_Y[lane]);
      int view = lookDownLane(c);
      tried[lane] = true;
      if (view != LV_BLOCKED) break;  // free here (switch later if the next column is blocked)
      int to = pickLane(lane);
      if (to < 0) {                   // every lane looked blocked: look again
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
  // past the last column: drive on until the robot's back has cleared it
  driveTo(colBack(COLS - 1) + HALF_LEN + EXIT_MARGIN, LANE_Y[lane]);
}

// ---------------- mission site
float siteHeading() {
  // arm side must face the site: bottom site + arm on right -> face +x
  if (siteOnTop == ARM_ON_RIGHT) return PI;
  return 0;
}

void goToSite() {
  float siteY = SITE_Y_BOT;
  float stopY = SITE_Y_BOT + SITE_STANDOFF;
  if (siteOnTop) {
    siteY = SITE_Y_TOP;
    stopY = SITE_Y_TOP - SITE_STANDOFF;
  }
  float h = siteHeading();
  driveTo(SITE_X - SITE_LEADIN * cos(h), stopY);  // lead-in point
  driveTo(SITE_X, stopY);                         // straight in, already facing h
  turnTo(h);
}

void doSiteTask() {
  // Placeholder for the arm + IR code (plant seed, collect rocks, classify
  // the far plots). The simulator cannot simulate the arm.
  Enes100.println("   [site] arm/IR task would run here");
  delay(2000);
}

void leaveSite() {
  readPose();
  if (cos(siteHeading()) > 0) {
    driveTo(SITE_EXIT_X, py);            // facing +x: drive forward
  } else {
    reverseBy(SITE_EXIT_X - px);         // facing -x: back up
  }
}

// ---------------- main program
void setup() {
  Enes100.begin("OTV Team", SEED, 3, 1116, 8, 9);
  Tank.begin();
  startMs = millis();

  readPose();
  siteOnTop = (py < 1.0);  // started low -> mission site is on top
  Enes100.print("Start x = ");
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
  lane = nearestLane(py);
  crossObstacles();
  sayTime();

  Enes100.println("== 4. Right before the limbo and log");
  driveTo(LIMBO_APPROACH_X, LIMBO_Y);
  turnTo(0);
  sayTime();

  Enes100.println("== 5. Through the limbo");
  driveTo(GOAL_X, LIMBO_Y);
  stopMotors();
  sayTime();
  Enes100.println("== DONE");
}

void loop() {
  delay(1000);
}
