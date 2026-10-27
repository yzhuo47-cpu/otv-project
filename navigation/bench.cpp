#include "bench.h"
#include "config.h"
#include "drive.h"
#include "nav.h"
#include "sonar.h"
#include "vision.h"

namespace {

int pwm = DRIVE_PWM;
const unsigned long MOVE_MS = 1000;

void help() {
  Serial.println(F("\n=== OTV bench test ==="));
  Serial.println(F(" -- no vision needed --"));
  Serial.println(F(" w/s/a/d : forward / back / turn left / turn right for 1 s"));
  Serial.println(F(" x       : stop"));
  Serial.println(F(" + / -   : PWM up / down by 10 (find MIN_MOVE_PWM, TURN_PWM_MIN)"));
  Serial.println(F(" m       : each motor channel fwd 1 s then rev 1 s (check directions)"));
  Serial.println(F(" u       : 20 sonar readings, raw ping + median"));
  Serial.println(F(" o       : drive forward until sonar < OBSTACLE_STOP_CM (max 10 s)"));
  Serial.println(F(" -- needs Vision System (classroom) --"));
  Serial.println(F(" c       : connect to Vision System"));
  Serial.println(F(" v       : print x, y, theta for 10 s (turn OTV by hand: CCW = theta up?)"));
  Serial.println(F(" t       : turnTo 0, 90, 180, -90 deg"));
  Serial.println(F(" g       : driveTo(BENCH_GOTO_X, BENCH_GOTO_Y)"));
  Serial.println(F(" h       : this help"));
  Serial.print(F("PWM = ")); Serial.println(pwm);
}

void motorCheck() {
  const char sides[2] = { 'L', 'R' };
  for (uint8_t s = 0; s < 2; s++) {
    for (uint8_t i = 0; i < drive::channelCount(sides[s]); i++) {
      Serial.print(sides[s]); Serial.print(i); Serial.println(F(" FORWARD (wheel should push OTV forward)"));
      drive::testChannel(sides[s], i, pwm);  delay(MOVE_MS);
      Serial.print(sides[s]); Serial.print(i); Serial.println(F(" REVERSE"));
      drive::testChannel(sides[s], i, -pwm); delay(MOVE_MS);
      drive::stop(); delay(300);
    }
  }
  Serial.println(F("Wrong direction? set inverted=true for that channel in config.h"));
}

void sonarCheck() {
  for (uint8_t i = 0; i < 20; i++) {
    float raw = sonar::pingCm();
    delay(SONAR_PING_GAP_MS);
    float med = sonar::readCm();
    Serial.print(F("raw=")); Serial.print(raw, 1);
    Serial.print(F(" cm  median=")); Serial.print(med, 1); Serial.println(F(" cm"));
    delay(100);
  }
}

void obstacleCheck() {
  Serial.println(F("Driving forward - put a hand / box in front of the sensor"));
  unsigned long t0 = millis();
  while (millis() - t0 < 10000UL) {
    if (sonar::blocked(OBSTACLE_STOP_CM)) {
      drive::stop();
      Serial.print(F("STOP: obstacle at ")); Serial.print(sonar::readCm(), 1); Serial.println(F(" cm"));
      return;
    }
    drive::set(pwm, pwm);
  }
  drive::stop();
  Serial.println(F("No obstacle in 10 s"));
}

bool needVision() {
  if (vision::connected()) return true;
  Serial.println(F("Not connected - press c first"));
  return false;
}

void visionPrint() {
  if (!needVision()) return;
  unsigned long t0 = millis();
  while (millis() - t0 < 10000UL) {
    vision::Pose p;
    if (vision::readPose(p)) {
      Serial.print(F("x=")); Serial.print(p.x, 3);
      Serial.print(F(" y=")); Serial.print(p.y, 3);
      Serial.print(F(" theta=")); Serial.print(p.theta, 3);
      Serial.print(F(" (")); Serial.print(degrees(p.theta), 1); Serial.println(F(" deg)"));
    } else {
      Serial.println(F("marker not visible"));
    }
    delay(250);
  }
}

void turnCheck() {
  if (!needVision()) return;
  const float targets[] = { 0.0f, HALF_PI, PI, -HALF_PI };
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(F("turnTo ")); Serial.print(degrees(targets[i]), 0); Serial.print(F(" deg -> "));
    NavStatus s = nav::turnTo(targets[i]);
    Serial.print(nav::statusName(s));
    Serial.print(F(", now ")); Serial.print(degrees(vision::theta()), 1); Serial.println(F(" deg"));
    delay(1000);
  }
}

void gotoCheck() {
  if (!needVision()) return;
  Serial.print(F("driveTo ")); Serial.print(BENCH_GOTO_X); Serial.print(F(", ")); Serial.println(BENCH_GOTO_Y);
  DriveOptions opt = { true, false };
  NavStatus s = nav::driveTo(BENCH_GOTO_X, BENCH_GOTO_Y, opt);
  Serial.print(F("result: ")); Serial.println(nav::statusName(s));
}

}  // namespace

namespace bench {

void begin() {
  help();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'w': drive::set(pwm, pwm);   delay(MOVE_MS); drive::stop(); break;
    case 's': drive::set(-pwm, -pwm); delay(MOVE_MS); drive::stop(); break;
    case 'a': drive::set(-pwm, pwm);  delay(MOVE_MS); drive::stop(); break;
    case 'd': drive::set(pwm, -pwm);  delay(MOVE_MS); drive::stop(); break;
    case 'x': drive::stop(); break;
    case '+': pwm = min(pwm + 10, 255); Serial.print(F("PWM = ")); Serial.println(pwm); break;
    case '-': pwm = max(pwm - 10, 0);   Serial.print(F("PWM = ")); Serial.println(pwm); break;
    case 'm': motorCheck(); break;
    case 'u': sonarCheck(); break;
    case 'o': obstacleCheck(); break;
    case 'c': vision::begin(VISION_CONNECT_TIMEOUT_MS); break;
    case 'v': visionPrint(); break;
    case 't': turnCheck(); break;
    case 'g': gotoCheck(); break;
    case 'h': help(); break;
    default: break;  // ignore newlines etc.
  }
}

}  // namespace bench
