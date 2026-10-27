// nav.h - movement primitives built on drive + sonar + vision.
// Every function BLOCKS until done and always has a timeout, so the OTV can
// never get stuck forever inside one of them.
#pragma once
#include <Arduino.h>
#include "vision.h"

enum NavStatus : uint8_t {
  NAV_OK,        // reached the goal
  NAV_BLOCKED,   // ultrasonic saw something closer than OBSTACLE_STOP_CM
  NAV_TIMEOUT,   // took too long
  NAV_LOST       // marker not visible for VISION_LOST_TIMEOUT_MS
};

struct DriveOptions {
  bool checkObstacles;  // stop with NAV_BLOCKED if the sonar sees something
  bool allowBlind;      // keep driving straight briefly if the marker disappears
};

namespace nav {

// Wrap any angle into -pi..pi.
float wrapAngle(float a);

// Wait until the marker is visible (motors stopped). false after timeoutMs.
bool waitForVision(unsigned long timeoutMs);
// Same, and also returns a valid pose in p.
bool waitForPose(vision::Pose& p, unsigned long timeoutMs);

// Turn in place until facing targetRad (Vision System frame).
NavStatus turnTo(float targetRad);

// Drive to the point (tx, ty) in meters, steering on the heading to it.
NavStatus driveTo(float tx, float ty, DriveOptions opt);

// Creep forward slowly until the sonar reads <= stopCm (approach the site).
NavStatus creepUntilClose(float stopCm, unsigned long timeoutMs);

// Reverse straight back until the camera says we moved `meters`.
NavStatus reverse(float meters, unsigned long timeoutMs);

// Drive at a fixed PWM for a fixed time.
void timedDrive(int left, int right, unsigned long ms);

const __FlashStringHelper* statusName(NavStatus s);

}  // namespace nav
