# MS6 Navigation – The Harvesters (real OTV)

Three separate programs, one for each MS6 navigation objective. MS6 lets us demo each objective on its own, so upload only the one you are demoing.

| Folder | Objective | Start the OTV… | Ends when… |
|---|---|---|---|
| `Nav1_MissionSite` | Nav 1: within 150 mm of the mission site | anywhere in the landing zone | the OTV is next to the mission site |
| `Nav2_Obstacles` | Nav 2: completely past the 3 obstacles | anywhere before the obstacles (landing zone or mission site) | x ≥ 2.9 m |
| `Nav3_Destination` | Nav 3: completely into the destination zone | anywhere after the obstacles, before the limbo | x ≥ 3.8 m |

Each folder has one `.ino` file with the same name, so it opens directly in the Arduino IDE.

## 1. Before uploading: fill in the settings

At the top of each `.ino`:

| Setting | Current value | Change to |
|---|---|---|
| `MARKER_ID` | 0 | our ArUco marker ID |
| `ROOM` | 1116 | our lab room (1116 / 1120 / 1201) |
| `WIFI_TX`, `WIFI_RX` | 3, 2 | pins the Wi-Fi module's TX and RX are wired to |
| `TRIG`, `ECHO` (Nav2 only) | 12, 11 | ultrasonic sensor pins |

This uses the ENES100 library's 6-argument `Enes100.begin(name, SEED, markerId, room, tx, rx)`.

## 2. Motor pins (L298N)

| Arduino pin | L298N | Job |
|---|---|---|
| 5 | ENA | left speed (PWM) |
| 9 | IN1 | left direction |
| 8 | IN2 | left direction |
| 10 | ENB | right speed (PWM) |
| 7 | IN3 | right direction |
| 6 | IN4 | right direction |

## 3. First checks on the arena

1. **Wheel direction.** Lift the OTV off the floor and run any program. When it drives forward, all 4 wheels should spin forward. If one side spins backward, swap that side's two IN pin numbers in the code, for example `L_IN1 = 8, L_IN2 = 9`.
2. **Marker direction.** Point the OTV toward the limbo (the +x direction). The Vision System should show theta ≈ 0. Driving forward should make x go up. If not, the marker is mounted rotated and needs to be turned.
3. **Turning direction.** When theta needs to go up, the OTV should turn counterclockwise (left). If it spins the wrong way forever, the left and right sides are swapped.

## 4. How each program works

All three use the same building blocks:
- `readPose()`: gets x, y and theta from the Vision System. If the marker isn't visible, the OTV **stops and waits** instead of using an old position.
- `turnTo(angle)`: turns in place until it faces `angle`, within about 6°.
- `driveToward(angle)`: drives forward and steers toward `angle`. The bigger the heading error, the stronger the correction. If the error is over about 30°, it stops and turns first.
- `angleDiff()`: keeps the angle error between −π and π, so the OTV never turns the long way around.

**Nav1:** The mission site is on the opposite side from where the OTV starts. If the OTV starts at the top it goes to the bottom site, and if it starts at the bottom it goes to the top site. `goToPoint()` keeps pointing at the site and stops `STOP_DIST` away from its center.

**Nav2:** The obstacles sit in 3 rows, at y = 0.5, 1.0 and 1.5. The OTV drives along the center of one row (a "lane"). If the ultrasonic sensor sees something closer than 15 cm, the OTV stops, moves sideways to another lane, and checks that the new lane is clear. It tries the top lane first, since the limbo is on top. The sensor takes 3 readings and uses the middle one, which ignores bad readings.

**Nav3:** Moves sideways to line up with the limbo (y = 1.5), then drives straight in +x under the limbo until the whole OTV is in the destination zone.

## 5. Values to tune on the real arena

The arena numbers are estimates from our MS3 arena map (Figure M1). Measure the real arena and adjust:

| Constant | File | What to do |
|---|---|---|
| `MISSION_X`, `MISSION_Y_TOP`, `MISSION_Y_BOTTOM` | Nav1 | center of the two possible mission site locations |
| `STOP_DIST` | Nav1 | lower it if the OTV stops too far from the site, raise it if it bumps the site. The goal is to stop within 150 mm. |
| `LANES` | Nav2 | y of the center of each obstacle row |
| `PAST_OBSTACLES_X` | Nav2 | last obstacle's right edge + half the OTV length + a few cm |
| `OBSTACLE_CM` | Nav2 | how close the OTV gets before switching lanes. It must be close enough that the OTV's sides clear the obstacle behind it when moving sideways. |
| `LIMBO_Y` | Nav3 | center of the limbo opening |
| `DEST_X` | Nav3 | destination zone line + half the OTV length + a few cm |
| `DRIVE_SPEED`, `TURN_SPEED` | all | lower them if the OTV is too fast to stop or turn accurately |
| `K_HEADING` | all | raise it if the OTV drifts off its lane, lower it if it wiggles |

## 6. Notes

- **Servo + pin 10:** On an Uno, `#include <Servo.h>` turns off PWM on pins 9 and 10. Right-side speed (ENB) is on pin 10. If the mission code uses a servo, move ENB to a free PWM pin (3 or 11) and change `R_EN` in the code.
- The H-bridge and Arduino need a common ground.
- Each program prints its progress, such as "Nav2: switched to lane 2", to the Vision System. These messages are not shown during the official product demo.
