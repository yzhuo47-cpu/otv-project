// OTV navigation - ENES100 web simulator (enes100.umd.edu/tools/simulator)
// MS5 sub-task 6: random start -> mission site -> right before the limbo and log.
// MS5 sub-task 7: get past the obstacles. How to run: see README.md.
#include <cmath>
#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"

// ======================== SETTINGS ========================
// Plain variables, not #define (the simulator mishandles #define lines with comments).

// Speeds. Simulator: driving speed = PWM / 510 m/s, turning speed = 2 * PWM / 255 rad/s
int   DRIVE_PWM  = 150;    // normal driving, ~0.29 m/s
int   TURN_PWM   = 80;     // normal turning, ~36 deg/s
int   SLOW_PWM   = 40;     // used close to the target so the robot doesn't overshoot
float STEP_M     = 0.15;   // longest drive before checking the position again (m)
float ANGLE_TOL  = 0.035;  // heading counts as correct within ~2 deg
float ARRIVE_TOL = 0.03;   // point counts as reached within 3 cm

// Arena (simulator, meters)
float SITE_X     = 0.55;   // mission site center x
float SITE_Y_TOP = 1.45;   // top landing spot center y
float SITE_Y_BOT = 0.55;   // bottom landing spot center y
float SITE_GAP   = 0.20;   // park this far from the site center (in y)

float LANE_Y[3]    = {0.5, 1.0, 1.5};  // the 3 lanes (bottom, middle, top)
float COL_FRONT[2] = {1.4, 2.2};       // front face (x) of the 2 obstacle columns
float CHECK_GAP    = 0.25;  // robot center stops this far before a column to check it
float BLOCKED_DIST = 0.35;  // sonar reading closer than this = lane blocked
float EXIT_X       = 2.6;   // x safely past the last column (it ends at 2.4)

float LIMBO_X = 3.0;        // point right before the limbo and log
float LIMBO_Y = 1.5;        // middle of the limbo opening

// Latest position (m) and heading (rad) from the vision system
float px, py, pth;

// ======================== HELPERS ========================

// Read position and heading from the vision system.
void readPose() {
  px = Enes100.getX();
  py = Enes100.getY();
  pth = Enes100.getTheta();   // 0 = facing +x (toward the limbo)
}

// Keep an angle between -PI and PI, so the robot always turns the short way.
float wrapAngle(float a) {
  while (a > PI) a = a - TWO_PI;
  while (a < -PI) a = a + TWO_PI;
  return a;
}

// Run the wheels for "ms" milliseconds, then stop (negative PWM = backward).
// Simulator quirk: setting a backward left wheel first can glitch the robot's
// position, so then the right wheel goes first and the simulator waits 110 ms.
void runFor(int left, int right, int ms) {
  if (left >= 0) {
    Tank.setLeftMotorPWM(left);
    Tank.setRightMotorPWM(right);
  } else {
    Tank.setRightMotorPWM(right);
    Tank.setLeftMotorPWM(left);
    ms = ms + 110;              // make up for the simulator's start delay
  }
  delay(ms);
  Tank.turnOffMotors();
}

// ======================== MOVEMENT ========================
// The robot moves in short steps: move, stop, read the position, correct.

// Turn in place to face "target" (radians).
void turnTo(float target) {
  for (int i = 0; i < 20; i++) {                 // at most 20 tries
    readPose();
    float err = wrapAngle(target - pth);         // angle left to turn (+ = left)
    if (fabs(err) < ANGLE_TOL) return;           // facing the target: done
    int pwm = TURN_PWM;
    if (fabs(err) < 0.3) pwm = SLOW_PWM;         // last ~17 deg: turn slowly
    int ms = (int)(fabs(err) / (2.0 * pwm / 255.0) * 1000.0);  // time = angle / turning speed
    if (err > 0) runFor(-pwm, pwm, ms);          // turn left
    else         runFor(pwm, -pwm, ms);          // turn right
  }
}

// Drive straight to point (tx, ty).
void driveTo(float tx, float ty) {
  for (int i = 0; i < 100; i++) {                // at most 100 steps
    readPose();
    float dx = tx - px;
    float dy = ty - py;
    float dist = sqrt(dx * dx + dy * dy);        // distance left
    if (dist < ARRIVE_TOL) return;               // arrived
    turnTo(atan2(dy, dx));                       // face the target
    int pwm = DRIVE_PWM;
    if (dist < 0.1) pwm = SLOW_PWM;              // last 10 cm: drive slowly
    float step = dist;
    if (step > STEP_M) step = STEP_M;            // drive at most STEP_M, then check again
    int ms = (int)(step / (pwm / 510.0) * 1000.0);  // time = distance / speed
    runFor(pwm, pwm, ms);                        // both wheels forward for one step
  }
}

// ======================== OBSTACLES ========================

// Look straight ahead with the sonar: is there an obstacle right in front?
bool laneBlocked() {
  float d = Tank.readDistanceSensor(1);  // front sonar, meters (-1 = nothing within 1 m)
  Enes100.print("   sonar (m): ");
  Enes100.println(d);
  if (d < 0) return false;               // nothing seen
  if (d < BLOCKED_DIST) return true;     // obstacle right in front
  return false;                          // something farther away (e.g. the next column)
}

// Get past both obstacle columns. At each column: stop in front of it, face +x
// and check the lane with the sonar. If blocked, move over to the next lane
// and check again.
void passObstacles() {
  int lane = 1;                                  // start in the middle lane
  for (int c = 0; c < 2; c++) {                  // column 1, then column 2
    float checkX = COL_FRONT[c] - CHECK_GAP;     // stop point in front of this column
    driveTo(checkX, LANE_Y[lane]);               // go to the stop point in the current lane
    turnTo(0);                                   // face +x: the sonar looks down the lane
    int tries = 0;                               // max 3 lane changes, so it can't loop forever
    while (laneBlocked() && tries < 3) {         // blocked: try another lane
      lane = lane + 1;                           // next lane (after the top, back to the bottom)
      if (lane > 2) lane = 0;
      driveTo(checkX, LANE_Y[lane]);             // move to that lane, in front of the column
      turnTo(0);                                 // face +x again
      tries = tries + 1;
    }
  }
  driveTo(EXIT_X, LANE_Y[lane]);                 // drive on until past the last column
}

// ======================== MAIN PROGRAM ========================
void setup() {
  // connect to the vision system: team name, mission, ArUco marker ID, room, WiFi TX/RX pins
  Enes100.begin("OTV Team", SEED, 3, 1116, 8, 9);
  Tank.begin();                                     // simulator motors and sonar

  // 1. Mission site = the landing spot we did NOT start on.
  readPose();
  bool siteOnTop = (py < 1.0);           // started at the bottom -> site is on top
  float parkY = SITE_Y_BOT + SITE_GAP;   // park just above the bottom site,
  float faceSite = 0;                    // facing +x (the arm on the right side points at the site)
  if (siteOnTop) {
    parkY = SITE_Y_TOP - SITE_GAP;       // or just below the top site,
    faceSite = PI;                       // facing -x
  }
  Enes100.println("1. Going to the mission site");
  driveTo(SITE_X, parkY);                // drive to the parking spot
  turnTo(faceSite);                      // turn so the arm faces the site
  delay(1000);                           // the mission task (arm, IR) would run here

  // 2. Get past the obstacles.
  Enes100.println("2. Obstacles");
  passObstacles();

  // 3. Stop right before the limbo and log, facing it.
  Enes100.println("3. Going to the limbo");
  driveTo(LIMBO_X, LIMBO_Y);
  turnTo(0);
  Enes100.println("Done");
}

// Everything runs once in setup().
void loop() {
  delay(1000);
}
