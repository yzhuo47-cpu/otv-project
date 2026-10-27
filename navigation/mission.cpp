#include "mission.h"
#include "config.h"
#include "drive.h"
#include "nav.h"
#include "site.h"
#include "sonar.h"
#include "vision.h"

namespace {

enum Phase : uint8_t {
  PH_CONNECT,       // join Vision System, find out where we started
  PH_TO_SITE,       // drive to the mission site and park in front of it
  PH_SITE_TASK,     // arm + IR (site.cpp), report plots, back away
  PH_TO_OBSTACLES,  // go to the first checkpoint in the nearest lane
  PH_CROSS,         // column by column: check lane with sonar, switch if blocked
  PH_TO_LIMBO,      // line up with the limbo opening
  PH_LIMBO,         // straight through the limbo to the goal
  PH_DONE,
  PH_FAILED
};

const uint8_t N_LANES = sizeof(LANE_Y) / sizeof(LANE_Y[0]);
const uint8_t N_COLS  = sizeof(OBS_COL_X) / sizeof(OBS_COL_X[0]);
const uint8_t MAX_RETRIES = 2;

const DriveOptions FREE     = { false, false };  // no sonar checks
const DriveOptions BLIND_OK = { false, true };   // marker may vanish (limbo)

Phase phase = PH_CONNECT;
uint8_t retries = 0;
bool siteOnTop = true;
uint8_t lane = 1;
uint8_t curCol = 0;  // next obstacle column to pass
unsigned long runStart = 0;

void logPhase(const __FlashStringHelper* name) {
  vision::say(name);
  vision::say(F("  t(s)="), (millis() - runStart) / 1000.0f);
}

uint8_t nearestLane(float y) {
  uint8_t best = 0;
  for (uint8_t i = 1; i < N_LANES; i++)
    if (fabs(LANE_Y[i] - y) < fabs(LANE_Y[best] - y)) best = i;
  return best;
}

// Closest free lane to `from`. Ties go to the lane nearer the limbo,
// because that is where we are heading next. -1 if every lane is blocked.
int8_t pickLane(uint8_t from, const bool* blocked) {
  int8_t best = -1;
  for (uint8_t i = 0; i < N_LANES; i++) {
    if (blocked[i]) continue;
    if (best < 0) { best = i; continue; }
    int dNew = abs((int)i - (int)from), dOld = abs(best - (int)from);
    if (dNew < dOld ||
        (dNew == dOld && fabs(LANE_Y[i] - LIMBO_Y) < fabs(LANE_Y[best] - LIMBO_Y)))
      best = i;
  }
  return best;
}

float colFront(uint8_t c) { return OBS_COL_X[c] - OBS_DEPTH_M / 2; }
float colBack(uint8_t c)  { return OBS_COL_X[c] + OBS_DEPTH_M / 2; }

// Where the OTV center parks (x) to look at column c.
// First column: CHECK_PREF_GAP_M in front of it.
// Later columns: the space between columns is tight (~0.36 m), so park in
// the middle of the gap - that leaves the most room to turn on the spot.
float checkpointX(uint8_t c) {
  float x = colFront(c) - OTV_HALF_LENGTH - CHECK_PREF_GAP_M;
  if (c > 0) x = max(x, (colBack(c - 1) + colFront(c)) / 2);
  return x;
}

// Handles a failed phase: retry it a couple of times, then take `fallback`.
Phase onFail(NavStatus s, Phase fallback) {
  vision::say(F("  nav failed:"));
  vision::say(nav::statusName(s));
  if (++retries <= MAX_RETRIES) return phase;  // try the same phase again
  return fallback;
}

Phase next(Phase p) { retries = 0; return p; }

// ---- phases ---------------------------------------------------------

Phase doConnect() {
  if (!vision::begin(VISION_CONNECT_TIMEOUT_MS)) return PH_FAILED;
  vision::Pose p;
  if (!nav::waitForPose(p, 10000UL)) {
    vision::say(F("marker not visible - check ARUCO_ID / marker on top"));
    return PH_FAILED;
  }
  runStart = millis();
  siteOnTop = p.y < ARENA_MID_Y;  // started low => site is on top
  vision::say(F("start x="), p.x);
  vision::say(F("start y="), p.y);
  vision::say(siteOnTop ? F("site: TOP") : F("site: BOTTOM"));

  // Warn early if the OTV is too big to turn in the gap between columns.
  for (uint8_t c = 1; c < N_COLS; c++) {
    if (OTV_TURN_RADIUS > (colFront(c) - colBack(c - 1)) / 2)
      vision::say(F("WARNING: OTV too big to turn between the columns!"));
  }
  return next(PH_TO_SITE);
}

// Heading while parked at the site: arm side must face the plots.
// Bottom site + arm on right -> face +x (right side = -y). Top site -> -x.
float siteHeading() {
  return (siteOnTop == (ARM_ON_RIGHT == 1)) ? PI : 0.0f;
}
float siteStopY() { return siteOnTop ? SITE_STOP_Y_TOP : SITE_STOP_Y_BOT; }

Phase doToSite() {
  float h = siteHeading();
  // Lead-in point SITE_LEADIN_M before the parking spot (along x), so the
  // last bit is a straight drive and the OTV never spins next to the plots.
  float leadX = SITE_X - SITE_LEADIN_M * cos(h);
  NavStatus s = nav::driveTo(leadX, siteStopY(), FREE);
  if (s == NAV_OK) s = nav::driveTo(SITE_X, siteStopY(), FREE);
  if (s == NAV_OK) s = nav::turnTo(h);  // tiny final squaring-up
  // Skipping the site still lets us score navigation, so fall back to that.
  if (s != NAV_OK) return onFail(s, PH_TO_OBSTACLES);

#if SITE_USE_SONAR_CREEP
  if (nav::creepUntilClose(SITE_STANDOFF_CM, SITE_CREEP_TIMEOUT_MS) != NAV_OK)
    vision::say(F("  creep timeout - doing site task from here"));
#endif
  return next(PH_SITE_TASK);
}

Phase doSiteTask() {
  PlotPattern p = site::runTask();
  vision::reportPlots(p);
  // Leave along x until clear of the grid, THEN turn (arm sweeps a circle).
  NavStatus s;
  if (cos(siteHeading()) > 0) s = nav::driveTo(SITE_EXIT_X, siteStopY(), FREE);  // facing +x: forward
  else                        s = nav::reverse(SITE_EXIT_X - SITE_X, 20000UL);   // facing -x: back up
  if (s != NAV_OK) vision::say(F("  site exit incomplete"));
  return next(PH_TO_OBSTACLES);
}

Phase doToObstacles() {
  vision::Pose p;
  if (!nav::waitForPose(p, VISION_LOST_TIMEOUT_MS)) return onFail(NAV_LOST, PH_FAILED);
  lane = nearestLane(p.y);
  curCol = 0;
  NavStatus s = nav::driveTo(checkpointX(0), LANE_Y[lane], FREE);
  if (s != NAV_OK) return onFail(s, PH_FAILED);
  return next(PH_CROSS);
}

// What the sonar sees from column c's checkpoint, facing +x, in the current
// lane. The map says how far each column face should be, so:
//   reading < column c face (+margin)        -> BLOCKED here
//   reading < column c+1 face (+margin)      -> free here, NEXT column blocked
//   farther / no echo                        -> THROUGH (free here and next)
enum LaneView : uint8_t { LV_BLOCKED, LV_NEXT_BLOCKED, LV_THROUGH };

LaneView lookDownLane(uint8_t c, const vision::Pose& p) {
  float d = sonar::readCm();
  vision::say(F("  sonar cm="), d);
  if (d == SONAR_NO_ECHO) return LV_THROUGH;
  float m = d / 100.0f;
  float sensorX = p.x + SONAR_OFFSET_M;
  if (m < colFront(c) - sensorX + BLOCKED_MARGIN_M) return LV_BLOCKED;
  if (c + 1 < N_COLS && m < colFront(c + 1) - sensorX + BLOCKED_MARGIN_M) return LV_NEXT_BLOCKED;
  return LV_THROUGH;
}

// Closest lane to `from` that is not in `skip`. Ties -> nearer the limbo.
int8_t closestLane(uint8_t from, const bool* skip) { return pickLane(from, skip); }

Phase doCross() {
  // curCol survives retries: a retry resumes at the column we were at,
  // it never drives back through columns already passed.
  while (curCol < N_COLS) {
    uint8_t c = curCol;
    float cx = checkpointX(c);
    bool tried[N_LANES]  = { false };  // looked down this lane at this column
    bool freeNow[N_LANES] = { false }; // free at this column, next one blocked
    uint8_t resets = 0;

    // Go to this column's checkpoint in the current lane (that lane was free
    // at the previous column, so no sonar needed on the way).
    NavStatus s = nav::driveTo(cx, LANE_Y[lane], FREE);
    if (s != NAV_OK) return onFail(s, PH_FAILED);

    while (true) {
      s = nav::turnTo(0);  // look down the lane
      if (s != NAV_OK) return onFail(s, PH_FAILED);
      vision::Pose p;
      if (!nav::waitForPose(p, VISION_LOST_TIMEOUT_MS)) return onFail(NAV_LOST, PH_FAILED);

      vision::say(F("  column "), c + 1);
      vision::say(F("  lane y="), LANE_Y[lane]);
      LaneView v = lookDownLane(c, p);
      tried[lane] = true;

      if (v == LV_THROUGH) break;  // free here (and at the next column)

      int8_t to;
      if (v == LV_NEXT_BLOCKED) {
        freeNow[lane] = true;
        // This lane works for now, but we would have to switch lanes in the
        // tight gap at the next column. If allowed, first look for a lane
        // that goes straight through both columns.
        to = AVOID_GAP_TURNS ? closestLane(lane, tried) : -1;
        if (to < 0) break;  // keep this lane; switch in the gap later
        vision::say(F("  next column blocked - looking for a straight lane"));
      } else {
        to = closestLane(lane, tried);
        if (to < 0) {
          // Every lane looked at. Use one that is at least free here.
          bool notFree[N_LANES];
          for (uint8_t i = 0; i < N_LANES; i++) notFree[i] = !freeNow[i];
          to = closestLane(lane, notFree);
          if (to < 0) {
            // Every lane "blocked" = sonar noise or wrong map numbers.
            if (++resets > 1) return PH_FAILED;
            vision::say(F("  all lanes blocked?! checking again"));
            for (uint8_t i = 0; i < N_LANES; i++) tried[i] = false;
            tried[lane] = true;
            to = closestLane(lane, tried);
          }
        }
      }
      // Slide sideways (in y) at the checkpoint into the new lane.
      s = nav::driveTo(cx, LANE_Y[to], FREE);
      if (s != NAV_OK) return onFail(s, PH_FAILED);
      lane = to;
      // A lane already known to be free here needs no second look.
      if (freeNow[lane] && tried[lane]) { s = nav::turnTo(0); if (s != NAV_OK) return onFail(s, PH_FAILED); break; }
    }
    curCol++;
  }

  // Past the last column: drive until the OTV's rear clears it.
  float exitX = colBack(N_COLS - 1) + OTV_HALF_LENGTH + OBS_EXIT_MARGIN_M;
  NavStatus s = nav::driveTo(exitX, LANE_Y[lane], FREE);
  if (s != NAV_OK) return onFail(s, PH_FAILED);
  return next(PH_TO_LIMBO);
}

Phase doToLimbo() {
  NavStatus s = nav::driveTo(LIMBO_APPROACH_X, LIMBO_Y, FREE);
  if (s != NAV_OK) return onFail(s, PH_FAILED);
  return next(PH_LIMBO);
}

Phase doLimbo() {
  NavStatus s = nav::driveTo(GOAL_X, LIMBO_Y, BLIND_OK);
  if (s == NAV_LOST) {
    // Probably parked right under the bar where the camera can't see the
    // marker - waiting there would never end. Push forward, then retry.
    nav::timedDrive(DRIVE_PWM, DRIVE_PWM, 3000);
  }
  if (s != NAV_OK) return onFail(s, PH_FAILED);
  return next(PH_DONE);
}

}  // namespace

namespace mission {

void begin() {
  phase = PH_CONNECT;
  retries = 0;
}

bool step() {
  switch (phase) {
    case PH_CONNECT:      logPhase(F("== CONNECT"));         phase = doConnect();     break;
    case PH_TO_SITE:      logPhase(F("== TO_SITE"));         phase = doToSite();      break;
    case PH_SITE_TASK:    logPhase(F("== SITE_TASK"));       phase = doSiteTask();    break;
    case PH_TO_OBSTACLES: logPhase(F("== TO_OBSTACLES"));    phase = doToObstacles(); break;
    case PH_CROSS:        logPhase(F("== CROSS_OBSTACLES")); phase = doCross();       break;
    case PH_TO_LIMBO:     logPhase(F("== TO_LIMBO"));        phase = doToLimbo();     break;
    case PH_LIMBO:        logPhase(F("== LIMBO"));           phase = doLimbo();       break;
    case PH_DONE:
      drive::stop();
      logPhase(F("== DONE"));
      return false;
    case PH_FAILED:
    default:
      drive::stop();
      logPhase(F("== FAILED (see messages above)"));
      return false;
  }
  return true;
}

}  // namespace mission
