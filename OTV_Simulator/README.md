# OTV Navigation — ENES100 Simulator Version

Navigation code for **The Harvesters** (ENES100, mission: Seed Planting), written for the
[ENES100 web simulator](https://enes100.umd.edu/tools/simulator).
It follows the same strategy as the real-robot code in `OTV_Navigation/`.

## What the robot does

1. **Landing zone → mission site.** The mission site is the landing spot the robot did *not* start on. The robot parks beside it with its arm side facing the site.
2. **Mission task.** Placeholder only — the simulator cannot simulate the arm or IR sensor.
3. **Obstacles.** At each obstacle column the robot stops at a checkpoint, faces forward and reads the ultrasonic sensor. If its lane is blocked, it slides sideways to another lane and checks again.
4. **Right before the limbo.** Drives to x = 3.0 m, lined up with the limbo opening.
5. **Through the limbo.** Drives on to x = 3.75 m.

Each stage, every sonar reading and the elapsed time are printed to the simulator console.

## How to run

1. Open the simulator and paste `OTV_Simulator.ino` into the editor.
2. Set the speed button to **instant** (three chevrons). Slower settings also work, just much slower.
3. Press **Run**, then **Continue** (the program pauses on its first line).
4. Press **Randomize** for a new obstacle layout and start position.

## How it works

- **Position:** `readPose()` gets x, y and heading from the overhead vision system before and after every move.
- **Moving in steps:** The robot does not steer continuously. It drives a short step (at most 12 cm), stops, reads its position and corrects. Turns work the same way.
- **Learning the stop delay:** The robot keeps moving a little after each stop command, and how much depends on the simulator speed setting. The code measures this after every move (`overheadMs`) and shortens the next move to match.
- **Obstacle check:** The column positions are known, so one sonar distance tells whether the obstacle is in this column, in the next column, or not in this lane.
- **Choosing a lane:** The closest lane not checked yet; on a tie, the lane closer to the limbo.

## Code layout

| Section | Contents |
|---|---|
| Settings | speeds, tolerances, robot size, arena positions |
| State | current pose, lane, learned stop delay |
| Helpers | read pose, wrap angles, set/stop motors |
| Movement | `turnTo()`, `driveTo()`, `reverseBy()` |
| Obstacles | checkpoints, lane choice, sonar check, `crossObstacles()` |
| Mission site | `goToSite()`, `doSiteTask()`, `leaveSite()` |
| Main program | `setup()` runs the whole mission once |

## Settings you are most likely to change

| Setting | Default | Meaning |
|---|---|---|
| `DRIVE_PWM` | 150 | driving speed (~0.29 m/s) |
| `TURN_PWM` | 80 | turning speed (~36 °/s) |
| `STEP_M` | 0.12 m | longest step before re-checking position |
| `ANGLE_TOL` | 0.035 rad | heading tolerance (~2°) |
| `ARM_ON_RIGHT` | `true` | side of the robot the arm sticks out of |
| `LIMBO_APPROACH_X` | 3.0 m | stop point before the limbo and log |

## Differences from the real robot (`OTV_Navigation/`)

- One file of plain C++ — the simulator only runs a subset of C++ (so settings are variables, not `#define`).
- Motors and sonar go through the simulator's `Tank` library.
- Arena and robot numbers are the simulator's: obstacle columns at x = 1.4–1.6 m and 2.2–2.4 m, landing spots at y = 0.55 / 1.45 m, robot 15 × 12 cm.
- `doSiteTask()` is only a placeholder for the arm and IR code.

## Milestone 5

This file can be used for the navigation sub-tasks, which MS5 allows to be shown in the simulator:

- **Sub-task 6** — navigation from a random start to the mission site and then to right before the limbo and log.
- **Sub-task 7** — getting past obstacles under a random layout.
