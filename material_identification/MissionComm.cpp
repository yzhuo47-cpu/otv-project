#include "Arduino.h"
#include "Enes100.h"
#include "Config.h"
#include "MissionComm.h"

void commBegin() {
#if USE_VISION
  Enes100.begin(TEAM_NAME, SEED, MARKER_ID, ROOM_NUMBER);
#endif
}

void commSendPattern(FarPattern p) {
#if USE_VISION
  switch (p) {
    case PATTERN_BOTH:     Enes100.mission(LOCATION, BOTH);     break;
    case PATTERN_NEITHER:  Enes100.mission(LOCATION, NEITHER);  break;
    case PATTERN_ADJACENT: Enes100.mission(LOCATION, ADJACENT); break;
    case PATTERN_DIAGONAL: Enes100.mission(LOCATION, DIAGONAL); break;
  }
#endif
}