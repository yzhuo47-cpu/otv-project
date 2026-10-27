// =====================================================================
//  config.h  -  ALL pins, arena coordinates and tuning numbers live here.
//  If something needs to change after testing, it should change HERE,
//  not inside the logic files.
//
//  Lines marked  TODO(verify)  are best guesses that must be checked
//  against Fig E6 (pin map), Fig T1 (mission flowchart) or the real arena.
// =====================================================================
#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------
//  Run mode
//    MODE_BENCH   : serial-monitor menu for testing motors / sonar / vision
//                   piece by piece (use this until the chassis is built)
//    MODE_MISSION : full autonomous run
// ---------------------------------------------------------------------
#define MODE_BENCH   0
#define MODE_MISSION 1
#define RUN_MODE     MODE_BENCH

#define SERIAL_BAUD  115200

// ---------------------------------------------------------------------
//  Vision System (Eagle Eye)
// ---------------------------------------------------------------------
#define TEAM_NAME    "OTV Team"     // TODO(verify): name shown on Vision System
#define ARUCO_ID     0              // TODO(verify): number printed on your marker
#define ROOM_NUMBER  1116           // TODO(verify): 1116, 1120 or 1201

// UNO Rev3 needs the external ESP8266 WiFi module (VCC=5V, GND, TX, RX).
// DO NOT use the library default (TX=3, RX=2) if D2/D3 are used by the
// sonar / motors (they are in the placeholder pins below). Avoid pins 0, 1,
// 13 (library rule) and 9, 10 (keep for the two SG90 servos: the Servo
// library uses Timer1 there).
#define WIFI_MODULE_TX_PIN  8       // TODO(verify): Arduino pin wired to module TX
#define WIFI_MODULE_RX_PIN  7       // TODO(verify): Arduino pin wired to module RX

#define VISION_CONNECT_TIMEOUT_MS  20000UL  // give up waiting for WiFi after this
#define VISION_LOST_TIMEOUT_MS      3000UL  // stop+wait this long if marker is hidden

// If the bench test shows theta DEcreasing when the OTV turns LEFT (CCW),
// change this to -1. Everything else (atan2 bearings) assumes CCW = positive.
#define VISION_THETA_SIGN  1

// ---------------------------------------------------------------------
//  Ultrasonic HC-SR04   TODO(verify): placeholder pins - the old pin map
//  (Fig E6) is obsolete since the board changed
// ---------------------------------------------------------------------
#define SONAR_TRIG_PIN   2
#define SONAR_ECHO_PIN   4
#define SONAR_TIMEOUT_US 20000UL    // ~3.4 m max range; no echo -> SONAR_NO_ECHO
#define SONAR_SAMPLES    3          // median of this many pings (odd number)
#define SONAR_PING_GAP_MS 30        // wait between pings so echoes don't mix
#define SONAR_NO_ECHO    -1.0f

#define OBSTACLE_STOP_CM   20.0f    // bench 'o' test + driveTo(checkObstacles) emergency stop.
                                    // (The mission decides lanes at checkpoints instead, see below.)

// ---------------------------------------------------------------------
//  Motors / L298N  (skid steer: left side + right side)
//
//  Each "channel" = one L298N output (one motor, or two motors in parallel).
//  Two wiring styles are supported:
//    a) ENA/ENB jumper ON, PWM goes into IN1/IN2      -> pinEn = -1
//         pinA, pinB must both be PWM pins (3,5,6,9,10,11)
//    b) PWM into ENA/ENB, direction on IN1/IN2        -> pinEn = PWM pin
//         pinA, pinB can be any digital pins
//  inverted = true flips that channel's direction (use this instead of
//  re-wiring a motor that spins backwards in the bench test).
//
//  TODO(verify): placeholder pins (old pin map is obsolete). Default
//  assumes style (a) with left side on D5/D6 and right side on D3/D11.
// ---------------------------------------------------------------------
struct MotorChannel {
  int8_t pinA;      // "forward" input
  int8_t pinB;      // "reverse" input
  int8_t pinEn;     // -1 = not used (style a)
  bool   inverted;
};

const MotorChannel LEFT_CHANNELS[]  = { {5, 6, -1, false} };
const MotorChannel RIGHT_CHANNELS[] = { {3, 11, -1, false} };
// Example if every motor has its own L298N channel (style b):
// const MotorChannel LEFT_CHANNELS[]  = { {7, 8, 5, false}, {12, 13, 6, false} };

#define DRIVE_PWM        255   // straight-line speed (worm gear motors: ~4.7 cm/s at full)
#define MIN_MOVE_PWM     110   // below this the motors stall - find it in bench test (p command)
#define TURN_PWM_MAX     230   // fastest in-place turn
#define TURN_PWM_MIN     140   // slowest in-place turn that still rotates the chassis
#define CREEP_PWM        160   // slow approach to the mission site

// ---------------------------------------------------------------------
//  Navigation tuning
// ---------------------------------------------------------------------
#define ANGLE_TOL_RAD      0.06f   // ~3.4 deg: "facing the right way"
#define TURN_KP            300.0f  // PWM per radian of heading error while turning
#define TURN_SETTLE_MS     250     // pause after a turn, then re-check heading
#define TURN_TIMEOUT_MS    15000UL

#define HEADING_KP         250.0f  // PWM correction per radian while driving
#define HEADING_KI         150.0f  // PWM per (radian*second): cancels weak-motor drift
#define HEADING_I_MAX      0.5f    // cap on the integral (KI*I_MAX = 75 PWM max)
#define CROSS_TRACK_GAIN   3.0f    // 10 cm off the lane line -> aim ~17 deg back
#define REALIGN_RAD        0.50f   // if heading error > this while driving, stop and turn
#define POS_TOL_M          0.015f  // "arrived" = less than this left to go ALONG the leg
                                   // (keep small: the gap between columns is tight)
#define DRIVE_TIMEOUT_MS   60000UL // one driveTo leg never takes longer than this

// Under the limbo bar the marker may be hidden from the camera; allow
// driving straight "blind" for at most this long on legs that permit it.
#define BLIND_DRIVE_MAX_MS 10000UL

// ---------------------------------------------------------------------
//  OTV footprint, from the MS3 CAD (Figs C2-C8, H1, H3):
//    box 20 cm long x 16 cm wide, 90 mm wheels on axles near the corners
//    -> wheels stick out ~2.8 cm front and back, ~3.3 cm each side;
//    arm + fixed IR arm (Fig M9) stick out of the RIGHT side, rear half,
//    ~7.3 cm past the box (C2).
//  The ArUco marker is assumed to sit at the box center (middle of the lid).
//  TODO(verify): measure the built OTV and update these.
// ---------------------------------------------------------------------
#define OTV_HALF_LENGTH    0.128f  // center -> front of the front wheels
#define OTV_TURN_RADIUS    0.175f  // center -> farthest point (IR-arm tip / wheel corner)
                                   //   = the circle swept when turning on the spot
#define SONAR_OFFSET_M     0.105f  // center -> face of the HC-SR04 (front panel)
#define ARM_ON_RIGHT       1       // arm reaches out of the OTV's right side (C2/H1)

// ---------------------------------------------------------------------
//  Arena layout (meters, Vision System frame: x 0..4, y 0..2,
//  theta 0 = +x, +pi/2 = +y).  Values read off Fig M1 (OTV Arena Map).
//  Fig M1 is not perfectly to scale (0.2 m obstacles are drawn 0.25 m
//  wide), so TODO(verify) with a tape measure in the real arena.
// ---------------------------------------------------------------------
#define ARENA_MID_Y          1.0f   // start y < this => started in the lower half

// Landing zone and mission site: squares centered at x~0.62, y~1.58 / ~0.42.
// The OTV starts in one; the mission site (2x2 plot grid, ~19 cm, drawn as
// the circle) is the other.
// Because the arm is on the SIDE, the OTV parks BROADSIDE next to the grid
// (like Fig H3), facing along x, so its arm side faces the plots:
//   site on bottom + arm on right -> face +x ;  site on top -> face -x.
// It drives in along x from SITE_LEADIN_M away so it never turns on the
// spot close to the plots (the arm would sweep into them).
#define SITE_X               0.62f  // OTV center x when parked (align arm with grid - arm team)
#define SITE_STOP_Y_TOP      1.35f  // OTV center y for a TOP site (grid near edge ~1.485)
#define SITE_STOP_Y_BOT      0.65f  // OTV center y for a BOTTOM site (grid near edge ~0.515)
                                    //   0.65 -> wheels ~2 cm from the grid; H3 shows almost touching
#define SITE_LEADIN_M        0.25f  // straight approach length along x
#define SITE_EXIT_X          0.92f  // after the task, move along x to here (clear of the grid) before turning

// Optional: creep until the FRONT sonar reads SITE_STANDOFF_CM. Off, because
// the sonar faces forward and the site is beside the OTV.
#define SITE_USE_SONAR_CREEP 0
#define SITE_STANDOFF_CM     15.0f
#define SITE_CREEP_TIMEOUT_MS 15000UL

// Obstacle zone: 2 columns of 0.2 x 0.5 m blocks, 3 possible lanes each.
const float LANE_Y[]    = { 0.50f, 1.00f, 1.50f };  // lane centers (block centers)
const float OBS_COL_X[] = { 1.535f, 2.10f };        // column centers (x)
#define OBS_DEPTH_M          0.20f  // block size along x
// At each column the OTV stops at a "checkpoint" in front of it, faces +x
// and uses the sonar to see if its lane is blocked.
#define CHECK_PREF_GAP_M     0.15f  // wanted room between OTV front and the column
                                    // (between the columns it parks in the middle of the gap instead)
#define BLOCKED_MARGIN_M     0.15f  // reading up to this much beyond the column face = blocked
// The gap between the columns (~0.36 m) is only just wider than the OTV's
// turning circle (2 x OTV_TURN_RADIUS). With 1, the OTV first looks for a lane
// that is free through BOTH columns (from column 1 the sonar can see column 2)
// and only turns inside the gap when no such lane exists.
#define AVOID_GAP_TURNS      0      // off: costs ~12 s avg (up to 40 s) and barely helped in the simulator
#define OBS_EXIT_MARGIN_M    0.10f  // after the last column, drive until the OTV rear is this far past it

// Bench test 'g' target
#define BENCH_GOTO_X         1.00f
#define BENCH_GOTO_Y         1.00f

// Limbo (blue part of the wall, y 1.0..2.0) / goal
#define LIMBO_APPROACH_X     3.00f  // line up here, then go straight through
#define LIMBO_Y              1.50f  // middle of the limbo opening
#define GOAL_X               3.75f  // wall is at x~3.43-3.51; OTV fully past it here
