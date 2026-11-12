# MS6 Navigation – The Harvesters (real OTV)

Three separate programs, one for each MS6 navigation objective. MS6 lets us demo each objective on its own, so upload only the one you are demoing.

| Folder | Objective | Start the OTV… | Ends when… |
|---|---|---|---|
| `Nav1_MissionSite` | Nav 1: within 150 mm of the mission site | anywhere in the landing zone | parked beside the site, arm side facing it |
| `Nav2_Obstacles` | Nav 2: completely past the 3 obstacles | anywhere before the obstacles (landing zone or mission site) | x ≥ 2.9 m |
| `Nav3_Destination` | Nav 3: completely into the destination zone | anywhere after the obstacles, before the limbo | x ≥ 3.8 m |

Each folder has one `.ino` file with the same name, so it opens directly in the Arduino IDE.

**The arm can't fold and sticks out on the right side.** The navigation is planned around that (see section 4).

## 1. Before uploading: fill in the settings

At the top of each `.ino`:

| Setting | Current value | Change to |
|---|---|---|
| `MARKER_ID` | 0 | our ArUco marker ID |
| `ROOM` | 1116 | the room we demo in (1116 / 1120 / 1201) |
| `WIFI_TX`, `WIFI_RX` | 3, 2 | pins the Wi-Fi module's TX and RX are wired to |

The ultrasonic sensor is wired TRIG → pin 12 and ECHO → pin 11, which is already set in Nav2.

Use ENES100 library **v6.2.4**, which has the 6-argument `Enes100.begin(name, SEED, markerId, room, tx, rx)`. Delete any older version first.

## 2. Pins

| Arduino pin | Connected to | Job |
|---|---|---|
| 5 | L298N ENA | left speed (PWM) |
| 9 / 8 | L298N IN1 / IN2 | left direction |
| 10 | L298N ENB | right speed (PWM) |
| 7 / 6 | L298N IN3 / IN4 | right direction |
| 12 / 11 | HC-SR04 TRIG / ECHO | ultrasonic (front center) |
| 3 / 2 | Wi-Fi module TX / RX | Vision System |

## 3. First checks on the arena

1. **Wheel direction.** Lift the OTV off the floor and run any program. When it drives forward, all 4 wheels should spin forward. If one side spins backward, swap that side's two IN pin numbers in the code, for example `L_IN1 = 8, L_IN2 = 9`.
2. **Marker direction.** Point the OTV toward the limbo (the +x direction). The Vision System should show theta ≈ 0, and driving forward should make x go up. The code assumes the marker is at the center of the OTV.
3. **Turning direction.** When theta needs to go up, the OTV should turn counterclockwise (left).

## 4. How each program works

All three use the same building blocks:
- `readPose()`: gets x, y and theta. If the marker isn't visible, the OTV **stops and waits**.
- `turnTo(angle)`: turns in place until it faces `angle`, within about 6°.
- `driveToward(angle)`: drives forward and steers toward `angle`. If the heading is off by more than about 30°, it stops and turns first.
- `moveToY(y, holdX)`: moves straight up or down and steers back to `holdX` if it drifts, so the arm doesn't drift into an obstacle.
- `angleDiff()`: keeps the angle error between −π and π, so the OTV never turns the long way around.

**Nav1:** The mission site is on the opposite side from where the OTV starts. The OTV drives to a parking spot `SIDE_DIST` from the site center, on its own side of the site. Then it turns so its **right side (the arm) faces the site**: it faces +x if the site is below it, and −x if the site is above it.

**Nav2:** Because of the arm, the OTV is about 39 cm wide, the lanes are only 50 cm wide, and turning in place swings the arm about 27 cm. So the OTV plans before it reaches the obstacles:
1. It drives to `SCAN_X` (before the first column) and stops in each of the 3 lanes, using the ultrasonic to see where each lane is blocked. It takes 9 readings per lane and uses the middle value.
2. If one lane is clear all the way, it takes that lane, trying the top lane first since the limbo is on top.
3. If no lane is clear, it drives in a lane that is open at column 1 and stops in the gap between the columns. The stop point is chosen so the arm has room on the side it points to. Then it moves sideways to a lane that is open at column 2 and continues.
4. In every lane it drives about 8 cm left of the lane center (`LANE_SHIFT`), so the arm and the left wheels have equal room.

**Nav3:** Moves sideways to line up with the limbo (y = 1.5), then drives straight under it until the whole OTV is in the destination zone.

## 5. Values to measure and tune

The OTV sizes come from the MS3 CAD, and the arena numbers are estimates from our MS3 arena map (Figure M1). **Measure the real ones.**

| Constant | File | What it is |
|---|---|---|
| `ARM_REACH` | Nav2 | OTV center to the arm tip (CAD: 0.27 m) |
| `HALF_WIDTH` | Nav2 | OTV center to the outside of the left wheels (CAD: 0.115 m) |
| `TURN_R` | Nav2 | OTV center to the farthest wheel corner (CAD: 0.17 m) |
| `FRONT` | Nav2 | OTV center to the ultrasonic sensor (CAD: 0.10 m) |
| `MISSION_X`, `MISSION_Y_TOP/BOTTOM` | Nav1 | center of the two possible mission site spots |
| `SIDE_DIST` | Nav1 | how far the OTV center parks from the site center. Lower it to get closer (goal: within 150 mm), raise it if the OTV or arm bumps the site. |
| `LANES` | Nav2 | center y of each obstacle row |
| `SCAN_X` | Nav2 | where the lane scan happens. There must be at least `ARM_REACH` + 5 cm of free space both in front (to the first column) and behind (to the mission site). |
| `OBS_DEPTH` | Nav2 | obstacle size along x (map: 0.2 m) |
| `PAST_OBSTACLES_X` | Nav2 | last obstacle's back edge + `ARM_REACH` (so Nav3 can move sideways safely) |
| `LIMBO_Y`, `DEST_X` | Nav3 | center of the limbo opening; destination line + half the OTV length |
| `DRIVE_SPEED`, `TURN_SPEED`, `K_HEADING` | all | lower the speeds if stops or turns are sloppy; raise `K_HEADING` if it drifts, lower it if it wiggles |

## 6. Limits and notes

- **Gap between obstacle columns.** A lane change between the two columns needs a gap (back of column 1 to front of column 2) of at least `ARM_REACH + TURN_R`, about **0.44 m**. In our simulation a 0.5 m gap worked but with only about 1 cm to spare, and a 0.8 m gap left about 4 cm. If the gap is smaller, the OTV prints `WARNING gap too small for the arm`. It only needs to change lanes between the columns when no lane is clear all the way.
- **Assumptions:** the obstacles sit in 2 columns and 3 rows, as on our arena map, and the arm is higher than the mission site (only the body was checked against the site).
- **Emergency stop:** if the ultrasonic reads less than 6 cm while driving, the OTV stops and waits. Don't raise `SAFETY_CM` above about 8 cm, because the lane-change stop point can be as close as about 10 cm to an obstacle.
- **Limbo height:** the CAD height is 17.84–17.96 cm against an 18.4 cm limbo. Keep the arm at its lowest before running Nav3.
- **Servo + pin 10:** On an Uno, `#include <Servo.h>` turns off PWM on pins 9 and 10. If the mission code uses a servo, move ENB (right speed) to pin 3 and move the Wi-Fi module to other pins.
- Progress messages such as "Nav2: lane 0 hit at x=1.40" print to the Vision System, which helps with tuning. They are not shown during the official product demo.

## 7. Testing done

All three programs were compiled and run in a simple simulation of our OTV: body plus right-side arm, 18 obstacle layouts, column gaps of 0.5 m and 0.8 m, several start positions, and added camera dropouts, position noise and bad ultrasonic readings. No collisions in 864 Nav2 runs. This is a simplified model, so test on the real arena.
