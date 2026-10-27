// vision.h - thin wrapper around the ENES100 library (Eagle Eye camera).
//
// WHY A WRAPPER: Enes100.h *defines* the global object `Enes100` inside the
// header, so if two .cpp files include it the linker fails with
// "multiple definition of Enes100". vision.cpp is the ONLY file that may
// include Enes100.h. Everyone else uses these functions.
#pragma once
#include <Arduino.h>

// Pattern of plantable substrate in the two far plots (SEED mission report).
enum PlotPattern : uint8_t {
  PLOTS_BOTH,
  PLOTS_NEITHER,
  PLOTS_ADJACENT,
  PLOTS_DIAGONAL,
  PLOTS_UNKNOWN   // not measured -> nothing is reported
};

namespace vision {

// Starts the WiFi module and waits (up to timeoutMs) for the Vision System.
bool begin(unsigned long timeoutMs);
bool connected();

bool  visible();
float x();       // meters, 0..4
float y();       // meters, 0..2
float theta();   // radians, -pi..pi, 0 = facing +x, CCW positive

// USE THIS for navigation instead of separate x()/y()/theta() calls.
// The library refreshes its data every 50 ms, so between a visible() check
// and a later x() call the marker can disappear and x() returns -1.
// readPose() reads everything together and re-checks visibility after.
struct Pose { float x, y, theta; };
bool readPose(Pose& p);  // false = marker not visible, p is not valid

// Report the far-plot pattern to the Vision System (mission scoring).
void reportPlots(PlotPattern p);

// Print to USB serial AND to the Vision System console (if connected).
// Keep these short: each message is sent over the slow WiFi link.
void say(const __FlashStringHelper* msg);
void say(const __FlashStringHelper* label, float value);

}  // namespace vision
