# MS6 Navigation – The Harvesters

Navigation code for the real OTV. There is one program for each MS6 navigation objective. Upload only the one you are about to demo.

| Program | Objective | Place the OTV… | It stops when… |
|---|---|---|---|
| `Nav1_MissionSite` | Nav 1: within 150 mm of the mission site | anywhere in the landing zone | it is parked beside the site, arm side facing the site |
| `Nav2_Obstacles` | Nav 2: completely past the 3 obstacles | anywhere before the obstacles | x ≥ 2.9 m |
| `Nav3_Destination` | Nav 3: completely into the destination zone | after the obstacles, before the limbo | x ≥ 3.8 m |

Each program is in its own folder with the same name, as the Arduino IDE requires.

---

## Step 1 – Before uploading

### 1a. Fill in these settings (top of **all three** files)

| Setting | Default | Change to |
|---|---|---|
| `MARKER_ID` | 0 | our ArUco marker ID |
| `ROOM` | 1116 | the room we demo in: 1116, 1120 or 1201 |
| `WIFI_TX`, `WIFI_RX` | 3, 2 | the Arduino pins the Wi-Fi module's TX and RX are wired to |

### 1b. Check the computer

- The ENES100 library is **v6.2.4**. Delete any older version first.
- In the Arduino IDE, the board is set to **Arduino Uno** and the correct port is selected.
- Press **Verify** before uploading and make sure there are no errors.

### 1c. Check the wiring

| Arduino pin | Goes to | Job |
|---|---|---|
| 10 | L298N ENA | right motors speed (PWM) |
| 9, 8 | L298N IN1, IN2 | right motors direction |
| 5 | L298N ENB | left motors speed (PWM) |
| 7, 6 | L298N IN3, IN4 | left motors direction |
| 12, 11 | HC-SR04 TRIG, ECHO | ultrasonic sensor (front center, facing forward) |
| 3, 2 | Wi-Fi module TX, RX | Vision System |

All parts (Arduino, L298N, sensor, Wi-Fi module) must share a **common ground**.

### 1d. Make sure the code matches our MS5 motor fix

The left motors are mounted as a mirror image of the right motors. In MS5 we fixed this in **one** of two ways:

| MS5 version | How the left side was fixed | `LEFT_IN1` in our MS5 code |
|---|---|---|
| A | in the code | 6 |
| **B (final)** | **left motor wires swapped at L298N OUT3/OUT4** | **7** |

This code is written for **version B**.
- If our MS5 code has `LEFT_IN1 = 7`, nothing needs to change.
- If it has `LEFT_IN1 = 6` (version A), change `L_IN3 = 7, L_IN4 = 6` to `L_IN3 = 6, L_IN4 = 7` in all three files.

Never fix it in both the circuit and the code. The two fixes cancel out.

### 1e. Upload safely

**Turn the motor battery switch off** before uploading. The program starts as soon as the upload finishes, and the OTV could drive off with the USB cable attached.

---

## Step 2 – Running a program

1. Put the OTV at its start position (see the first table).
2. Turn the battery on. If the Arduino is powered by USB instead, press the **reset** button.
3. The program connects to the Vision System, waits until it sees the marker, and then starts driving.
4. When it's done, the OTV stays still. Press **reset** to run it again.

---

## Step 3 – First checks on the arena

1. **Wheels:** Lift the OTV and run any program. When it drives forward, all 4 wheels must spin forward. If one side spins backward, swap that side's two IN numbers in the code.
2. **Turning:** The OTV must turn **counterclockwise (left)** when theta needs to go up. If it turns the wrong way and keeps spinning, the left and right pin groups are swapped.
3. **Marker:** With the OTV facing the limbo (+x), the Vision System should show **theta ≈ 0**, and x should go up when it drives forward.

---

## Step 4 – Measure and tune

The OTV sizes come from the MS3 CAD, and the arena positions come from our MS3 arena map. **Measure the real values** and change them in the code.

### OTV size (in `Nav2_Obstacles`)

| Constant | Default | What to measure |
|---|---|---|
| `ARM_REACH` | 0.27 m | center of the OTV to the tip of the arm |
| `HALF_WIDTH` | 0.115 m | center of the OTV to the outside of the left wheels |
| `TURN_R` | 0.17 m | center of the OTV to the farthest wheel corner |
| `FRONT` | 0.10 m | center of the OTV to the ultrasonic sensor |

### Arena

| Constant | File | Default | What it is |
|---|---|---|---|
| `MISSION_X`, `MISSION_Y_TOP`, `MISSION_Y_BOTTOM` | Nav1 | 0.55, 1.45, 0.55 | center of the two possible mission site positions |
| `SIDE_DIST` | Nav1 | 0.35 m | how far the OTV center parks from the site center (must end within 150 mm) |
| `LANES` | Nav2 | 0.5, 1.0, 1.5 | center y of the three obstacle rows |
| `OBS_DEPTH` | Nav2 | 0.20 m | size of an obstacle along x |
| `SCAN_X` | Nav2 | 1.05 m | where the OTV scans the lanes. It needs `ARM_REACH` + 5 cm of free space in front and behind. |
| `PAST_OBSTACLES_X` | Nav2 | 2.9 m | back edge of the last obstacle + `ARM_REACH` |
| `LIMBO_Y` | Nav3 | 1.5 | center of the limbo opening |
| `DEST_X` | Nav3 | 3.8 m | destination zone line + half the OTV length |

### Driving (all three files)

| Constant | Default | When to change |
|---|---|---|
| `DRIVE_SPEED`, `TURN_SPEED` | 200, 170 | lower them if turns or stops are not accurate |
| `K_HEADING` | 100 | raise it if the OTV drifts off its line, lower it if it wiggles |

---

## How the code works

These notes are for explaining the code to the instructor.

### Shared building blocks (in every file)

| Function | What it does |
|---|---|
| `readPose()` | Reads x, y and theta from the Vision System. If the marker is not visible, the OTV stops and waits. |
| `turnTo(angle)` | Turns in place until the OTV faces `angle` (within about 6°). |
| `driveToward(angle)` | Drives forward while steering toward `angle`. If it is more than about 30° off, it stops and turns first. |
| `moveToY(y, holdX)` | Moves straight up or down to `y` and keeps correcting back to `holdX`, so it doesn't drift sideways. |
| `angleDiff()` | Keeps angle errors between −π and π, so the OTV always turns the short way. |
| `setMotors(left, right)` | Sets each side's speed from −255 to 255 (negative = backward). |

### Nav1 – Mission site

1. The mission site is on the opposite side from where the OTV starts.
2. The OTV drives to a parking spot `SIDE_DIST` away from the site center.
3. It turns so its **right side (the arm) faces the site**: facing +x if the site is below it, facing −x if the site is above it.

### Nav2 – Obstacles

The arm can't fold and sticks out about 27 cm on the right. With the arm, the OTV is about 39 cm wide, and the lanes are only 50 cm wide. So the OTV plans its path **before** it reaches the obstacles:

1. **Scan:** At `SCAN_X`, the OTV stops in each of the 3 lanes and uses the ultrasonic sensor to find where each lane is blocked. It takes 9 readings per lane and uses the middle value.
2. **Clear lane:** If one lane is open the whole way, the OTV drives straight through it. It tries the top lane first, because the limbo is on top.
3. **No clear lane:** The OTV drives in a lane that is open at the first column and stops in the gap between the two columns. The stop point leaves room on the side the arm points to. Then it moves sideways to a lane that is open at the second column and continues.
4. In every lane, the OTV drives about 8 cm left of center, so the arm and the left wheels have equal room.
5. **Emergency stop:** if anything is closer than 6 cm in front, the OTV stops and waits.

### Nav3 – Destination

The OTV moves sideways to line up with the limbo (y = 1.5), then drives straight under it into the destination zone.

---

## Limits and notes

- **Gap between obstacle columns:** A lane change between the two columns needs a gap of at least `ARM_REACH + TURN_R` ≈ **0.44 m**. A 0.5 m gap works but is very tight. If the gap is smaller, the OTV prints `WARNING gap too small for the arm`. It only changes lanes there when no lane is open the whole way.
- **Assumptions:** The obstacles sit in 2 columns and 3 rows. The arm is higher than the mission site. The marker is at the center of the OTV.
- **Limbo height:** The CAD height is about 17.9 cm, and the limbo is 18.4 cm. Keep the arm at its lowest before running Nav3.
- **Servo + pin 10:** On an Uno, `#include <Servo.h>` turns off PWM on pins 9 and 10, and the right motor speed is on pin 10. If the mission code uses a servo, move ENA to another PWM pin.
- **Messages:** Progress messages such as `Nav2: lane 0 hit at x=1.40` print to the Vision System, which helps with tuning. They are not shown during the official demo.

## Testing done

All three programs were compiled and run in a computer simulation of our OTV. The simulation used the body and the right-side arm, the real motor wiring, 18 obstacle layouts, column gaps of 0.5 m and 0.8 m, and several start positions. It also added camera dropouts, position noise and bad ultrasonic readings. Nav2 had **no collisions in 864 runs**, and Nav1 and Nav3 finished every run. The simulation is simplified, so always test on the real arena.
