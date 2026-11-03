# OTV Navigation — ENES100 Simulator Version

Basic navigation code for **The Harvesters** (ENES100, mission: Seed Planting), written for the
[ENES100 web simulator](https://enes100.umd.edu/tools/simulator). It covers the two Milestone 5
navigation sub-tasks:

- **Sub-task 6:** random start in the landing zone → mission site → right before the limbo and log.
- **Sub-task 7:** get past the obstacles under a random layout.

## What the robot does

1. **Mission site.** The mission site is the landing spot the robot did *not* start on. The robot parks 20 cm from it and turns so its arm side (right side) faces the site.
2. **Obstacles.** At each of the two obstacle columns, the robot stops in front of the column, faces forward and reads the sonar. If something is closer than 35 cm, the lane is blocked, so it moves over to the next lane and checks again.
3. **Limbo.** The robot drives to x = 3.0 m, y = 1.5 m (in front of the limbo opening) and faces it.

The stages and sonar readings are printed to the simulator console.

## How to run

1. Open the simulator and paste `OTV_Simulator.ino` into the editor.
2. Set the speed button to **instant** (three chevrons).
3. Press **Run**, then **Continue** (the program pauses on its first line).
4. Press **Randomize** for a new obstacle layout and start position.

## How the code works

| Function | What it does |
|---|---|
| `readPose()` | gets x, y and heading from the vision system |
| `runFor()` | runs the wheels for a set time, then stops |
| `turnTo()` | turns in place until facing an angle (within ~2°) |
| `driveTo()` | drives straight to a point (within 3 cm) |
| `laneBlocked()` | reads the sonar: obstacle right in front? |
| `passObstacles()` | checks each column and changes lane if blocked |
| `setup()` | runs the whole route once |

**Moving in steps:** the robot does not steer continuously. It moves a short step (at most 15 cm), stops, reads its position from the vision system and corrects. Close to the target it switches to a slow speed so it does not overshoot.

## Settings you might change

| Setting | Default | Meaning |
|---|---|---|
| `DRIVE_PWM` | 150 | driving speed (~0.29 m/s) |
| `TURN_PWM` | 80 | turning speed (~36°/s) |
| `SLOW_PWM` | 40 | speed used close to the target |
| `BLOCKED_DIST` | 0.35 m | sonar reading below this = lane blocked |
| `LIMBO_X`, `LIMBO_Y` | 3.0, 1.5 m | stop point before the limbo and log |

## Notes

- The arena numbers are the simulator's (obstacle columns at x = 1.4–1.6 m and 2.2–2.4 m, lanes at y = 0.5 / 1.0 / 1.5 m). The real robot code lives in `OTV_Navigation/`.
- The mission task (arm, IR sensor) is not simulated; the robot just pauses 1 s at the site.
- To also drive through the limbo, add `driveTo(3.75, 1.5);` after the last `turnTo(0);` in `setup()`.
