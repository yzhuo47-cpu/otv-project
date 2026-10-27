// =====================================================================
//  OTV Navigation - ENES100 Seed mission, Arduino UNO Rev3
//
//  File map (open config.h first):
//    config.h        pins, arena coordinates, tuning       <- edit here
//    drive.*         L298N skid-steer motors
//    sonar.*         HC-SR04 ultrasonic, median filtered
//    vision.*        Eagle Eye / ENES100 library wrapper
//    nav.*           turnTo / driveTo / creep primitives
//    mission.*       full run state machine (Fig T1)
//    site.*          arm + IR work at the mission site     <- teammates
//    bench.*         Serial Monitor test menu
//
//  RUN_MODE in config.h picks bench test or full mission.
// =====================================================================
#include "config.h"
#include "drive.h"
#include "sonar.h"
#include "bench.h"
#include "mission.h"

void setup() {
  drive::begin();  // motors OFF before anything else
  sonar::begin();
  Serial.begin(SERIAL_BAUD);

#if RUN_MODE == MODE_BENCH
  bench::begin();
#else
  Serial.println(F("OTV mission mode"));
  mission::begin();
#endif
}

void loop() {
#if RUN_MODE == MODE_BENCH
  bench::loop();
#else
  static bool running = true;
  if (running) running = mission::step();
  else drive::stop();
#endif
}
