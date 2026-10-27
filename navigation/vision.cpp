// The ONLY file allowed to include Enes100.h (see vision.h).
#include "vision.h"
#include "config.h"
#include "Enes100.h"

namespace {
bool started = false;
}

namespace vision {

bool begin(unsigned long timeoutMs) {
  Enes100.begin(TEAM_NAME, SEED, ARUCO_ID, ROOM_NUMBER,
                WIFI_MODULE_TX_PIN, WIFI_MODULE_RX_PIN);
  started = true;

  unsigned long t0 = millis();
  while (!Enes100.isConnected()) {
    if (millis() - t0 > timeoutMs) {
      Serial.println(F("[vision] NOT connected - check WiFi module wiring/pins and room number"));
      return false;
    }
    delay(100);
  }
  say(F("[vision] connected"));
  return true;
}

bool connected() { return started && Enes100.isConnected(); }

bool  visible() { return started && Enes100.isVisible(); }
float x()       { return Enes100.getX(); }
float y()       { return Enes100.getY(); }
float theta()   { return VISION_THETA_SIGN * Enes100.getTheta(); }

bool readPose(Pose& p) {
  if (!visible()) return false;
  p.x = x();
  p.y = y();
  p.theta = theta();
  // x, y are -1 when hidden (theta = -1 is a real angle, so test x/y).
  return p.x >= 0.0f && p.y >= 0.0f && visible();
}

void reportPlots(PlotPattern p) {
  if (!started || p == PLOTS_UNKNOWN) return;
  int code;
  switch (p) {
    case PLOTS_BOTH:     code = BOTH;     break;
    case PLOTS_NEITHER:  code = NEITHER;  break;
    case PLOTS_ADJACENT: code = ADJACENT; break;
    default:             code = DIAGONAL; break;
  }
  Enes100.mission(LOCATION, code);
}

void say(const __FlashStringHelper* msg) {
  Serial.println(msg);
  if (started) Enes100.println(msg);
}

void say(const __FlashStringHelper* label, float value) {
  Serial.print(label);
  Serial.println(value, 3);
  if (started) {
    Enes100.print(label);
    Enes100.println(value);
  }
}

}  // namespace vision
