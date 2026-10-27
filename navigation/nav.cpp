#include "nav.h"
#include "config.h"
#include "drive.h"
#include "sonar.h"
#include "vision.h"

namespace nav {

float wrapAngle(float a) {
  while (a > PI)  a -= TWO_PI;
  while (a < -PI) a += TWO_PI;
  return a;
}

bool waitForPose(vision::Pose& p, unsigned long timeoutMs) {
  if (vision::readPose(p)) return true;
  drive::stop();  // never drive on stale coordinates
  unsigned long t0 = millis();
  while (!vision::readPose(p)) {
    if (millis() - t0 > timeoutMs) return false;
    delay(50);
  }
  return true;
}

bool waitForVision(unsigned long timeoutMs) {
  vision::Pose p;
  return waitForPose(p, timeoutMs);
}

NavStatus turnTo(float target) {
  target = wrapAngle(target);
  unsigned long t0 = millis();
  vision::Pose p;

  while (true) {
    if (millis() - t0 > TURN_TIMEOUT_MS) { drive::stop(); return NAV_TIMEOUT; }
    if (!waitForPose(p, VISION_LOST_TIMEOUT_MS)) { drive::stop(); return NAV_LOST; }

    // wrapAngle makes the OTV take the SHORT way round (e.g. 170deg -> -170deg
    // is a 20deg turn, not 340deg).
    float err = wrapAngle(target - p.theta);

    if (fabs(err) < ANGLE_TOL_RAD) {
      drive::stop();
      delay(TURN_SETTLE_MS);  // chassis stops rolling, camera catches up
      if (vision::readPose(p) && fabs(wrapAngle(target - p.theta)) < ANGLE_TOL_RAD) {
        return NAV_OK;
      }
      continue;  // drifted out of tolerance while settling: correct again
    }

    // Proportional turn speed: fast when far off, slow near the target so
    // it does not overshoot (the camera position lags a little).
    int pwm = constrain((int)(TURN_KP * fabs(err)), TURN_PWM_MIN, TURN_PWM_MAX);
    if (err > 0) drive::set(-pwm, pwm);  // need CCW -> turn left
    else         drive::set(pwm, -pwm);  // need CW  -> turn right
  }
}

NavStatus driveTo(float tx, float ty, DriveOptions opt) {
  unsigned long t0 = millis();
  vision::Pose p;
  if (!waitForPose(p, VISION_LOST_TIMEOUT_MS)) return NAV_LOST;

  // Unit vector of this leg. Used to detect "already drove past the target",
  // so the OTV never circles around a point it just missed by 6 cm.
  float sx = p.x, sy = p.y;  // leg start
  float legLen = hypot(tx - sx, ty - sy);
  if (legLen < POS_TOL_M) return NAV_OK;
  float ux = (tx - p.x) / legLen;
  float uy = (ty - p.y) / legLen;
  float legHeading = atan2(uy, ux);

  NavStatus s = turnTo(legHeading);
  if (s != NAV_OK) return s;

  unsigned long lastSeen = millis();
  unsigned long lastCtl  = millis();
  float headingI = 0;  // integral of heading error: learns left/right motor mismatch

  while (true) {
    if (millis() - t0 > DRIVE_TIMEOUT_MS) { drive::stop(); return NAV_TIMEOUT; }

    if (opt.checkObstacles && sonar::blocked(OBSTACLE_STOP_CM)) {
      drive::stop();
      return NAV_BLOCKED;
    }

    if (!vision::readPose(p)) {
      if (opt.allowBlind && millis() - lastSeen < BLIND_DRIVE_MAX_MS) {
        drive::set(DRIVE_PWM, DRIVE_PWM);  // e.g. marker hidden under the limbo bar
        continue;
      }
      if (!waitForPose(p, VISION_LOST_TIMEOUT_MS)) return NAV_LOST;
    }
    lastSeen = millis();

    float dx = tx - p.x, dy = ty - p.y;
    float along = dx * ux + dy * uy;  // distance still to go along the leg

    if (along < POS_TOL_M) { drive::stop(); return NAV_OK; }

    // Follow the LINE of the leg, not just the bearing to the end point:
    // cross > 0 means we are left of the line, so aim a bit to the right.
    // This keeps the OTV in the middle of its lane between obstacles.
    float cross = (p.y - sy) * ux - (p.x - sx) * uy;
    float want  = legHeading - atan(CROSS_TRACK_GAIN * cross);
    float err   = wrapAngle(want - p.theta);

    if (fabs(err) > REALIGN_RAD) {  // pushed far off course: stop and turn
      s = turnTo(want);
      if (s != NAV_OK) return s;
      lastCtl = millis();
      continue;
    }

    // PI heading control. The P part reacts to the current error; the
    // I part builds up until it cancels a motor that is simply weaker.
    unsigned long now = millis();
    float dt = (now - lastCtl) / 1000.0f;
    lastCtl = now;
    headingI = constrain(headingI + err * dt, -HEADING_I_MAX, HEADING_I_MAX);

    // Steer by slowing one side (err > 0 means we must turn LEFT).
    int corr  = (int)(HEADING_KP * err + HEADING_KI * headingI);
    int left  = constrain(DRIVE_PWM - corr, MIN_MOVE_PWM, 255);
    int right = constrain(DRIVE_PWM + corr, MIN_MOVE_PWM, 255);
    drive::set(left, right);
  }
}

NavStatus creepUntilClose(float stopCm, unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    float d = sonar::readCm();
    if (d != SONAR_NO_ECHO && d <= stopCm) { drive::stop(); return NAV_OK; }
    drive::set(CREEP_PWM, CREEP_PWM);
  }
  drive::stop();
  return NAV_TIMEOUT;
}

NavStatus reverse(float meters, unsigned long timeoutMs) {
  vision::Pose p;
  if (!waitForPose(p, VISION_LOST_TIMEOUT_MS)) return NAV_LOST;
  float sx = p.x, sy = p.y;
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (!vision::readPose(p)) {
      if (!waitForPose(p, VISION_LOST_TIMEOUT_MS)) return NAV_LOST;
    }
    if (hypot(p.x - sx, p.y - sy) >= meters) { drive::stop(); return NAV_OK; }
    drive::set(-DRIVE_PWM, -DRIVE_PWM);
  }
  drive::stop();
  return NAV_TIMEOUT;
}

void timedDrive(int left, int right, unsigned long ms) {
  drive::set(left, right);
  delay(ms);
  drive::stop();
}

const __FlashStringHelper* statusName(NavStatus s) {
  switch (s) {
    case NAV_OK:      return F("OK");
    case NAV_BLOCKED: return F("BLOCKED");
    case NAV_TIMEOUT: return F("TIMEOUT");
    default:          return F("LOST");
  }
}

}  // namespace nav
