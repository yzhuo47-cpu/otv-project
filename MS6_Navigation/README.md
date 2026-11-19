# MS6 Navigation – The Harvesters

Code for the three MS6 navigation objectives, on the real OTV (school **Arduino UNO R4 WiFi**).
Each objective is its own program. Upload the one you are about to demo.

| Program | Objective | Start the OTV | It stops when |
|---|---|---|---|
| `Nav1_MissionSite` | Within 150 mm of the mission site | Spot A or B in the landing zone | Parked beside the site, arm side facing it |
| `Nav2_Obstacles` | Completely past the 3 obstacles | Anywhere before x = 1.05 m (for example where Nav1 stopped) | x = 2.9 m |
| `Nav3_Destination` | Completely into the goal zone | After the obstacles (where Nav2 stopped) | x = 3.7 m |

The OTV does **not** need to go back to the start between programs. Each one starts where the last one stopped.

---

## Quick checklist

1. ENES100 library **v6.2.4** installed, board set to **Arduino UNO R4 WiFi**
2. Our ArUco marker shows up in the Vision System's **"Seen markers"**, and that number matches `MARKER_ID`
3. `ROOM = 1201` (change it if we demo in another room)
4. Motor battery **off** while uploading
5. The arm is **higher than the mission site** and **lower than 18.4 cm** (the limbo)

---

## 1. Computer setup (once)

1. **Board package:** Tools → Board → Boards Manager → search "UNO R4" → install **Arduino UNO R4 Boards**.
2. **Library:** download ENES100 **v6.2.4** from enes100.umd.edu → unzip into `Documents/Arduino/libraries`. Delete any older ENES100 folder first, then restart the IDE.
3. **Open the code:** keep the folder names as they are (the Arduino IDE needs the folder and the `.ino` to have the same name).
4. **Select the board:** Tools → Board → **Arduino UNO R4 WiFi**, then pick its port.
5. Press **Verify (✓)** on each program. You should see "Done compiling".

---

## 2. The ArUco marker

The camera must see our marker, or **no program will move**. Arena Details requires:

- **4x4 dictionary.** In the online generator (chev.me/arucogen), choose 4x4 (250) or 4x4 (1000). The default "Original ArUco" will not work.
- **50–100 mm** square, with a **15 mm white border** all around.
- Glued flat on something **rigid**, centered on top of the OTV, not covered by wires, tape or glare.
- IDs **0–5** are not allowed (those are the arena corners).

**Check:** put the OTV in the arena and open the Vision System page (room 1201: `lx-enes100-5.eng.umd.edu:8080`).
- Our ID must appear in **System Printouts → "Seen markers"**. If it isn't 205, change `MARKER_ID` in **all three** files.
- With the OTV facing the limbo (+x), **Theta ≈ 0**. If it reads about +1.57, turn the marker 90° clockwise; about −1.57, 90° counterclockwise; about ±3.14, turn it 180°.

---

## 3. Wiring

| Arduino pin | Goes to | Job |
|---|---|---|
| 10 | L298N ENA | Right motors speed (PWM) |
| 9, 8 | L298N IN1, IN2 | Right motors direction |
| 5 | L298N ENB | Left motors speed (PWM) |
| 7, 6 | L298N IN3, IN4 | Left motors direction |
| 12, 11 | HC-SR04 TRIG, ECHO | Ultrasonic (front center, facing straight ahead) |

All grounds (Arduino, L298N, sensor) must be connected together.

**Motor direction:** the code uses **IN1/IN3 HIGH = forward**. This matches MS5, where the left motor wires were swapped at the L298N OUT3/OUT4 terminal.
- If our MS5 code had `LEFT_IN1 = 6` (the left side was fixed in code, not in the wiring), change `L_IN3 = 7, L_IN4 = 6` to `L_IN3 = 6, L_IN4 = 7` in all three files.
- Fix the left side in **one** place only: the wiring **or** the code. Fixing both cancels out.

---

## 4. Uploading and running

1. **Turn the motor battery off.** The program starts the moment the upload finishes.
2. Plug in USB and press **Upload (→)**. Wait for "Done uploading".
3. Put the OTV at its start position.
4. Turn the battery on. If the Arduino is powered by USB only, press the board's **reset** button instead.
5. The program connects to the Vision System and waits until it sees the marker, then it starts. **A few seconds of no movement at the start is normal.**
6. When it finishes, the OTV stays still. Press **reset** to run it again.

---

## 5. What each program does

### Nav1 – Mission site
1. Works out where the site is: on the spot we didn't start on (A at y = 1.5, or B at y = 0.5).
2. Drives to a parking spot on **the side of the site that faces us**, `SIDE_DIST` from the site center.
3. Turns 90° so the **right side (arm) faces the site**: facing +x if the site is below, facing −x if it is above.

### Nav2 – Obstacles
The arm can't fold and sticks out 27 cm on the right. With the arm, the OTV is about 39 cm wide, and a lane is 50 cm wide. So the OTV plans before it reaches the obstacles:

1. Drives to x = 1.05 m (in front of column 1).
2. **Stops in each of the 3 lanes** and uses the ultrasonic to see where each lane is blocked. This is the "looking" step, not the OTV getting stuck.
3. **If a lane is clear the whole way, it takes it.** By the arena rules there is one obstacle per row, and the rumbles are low. So the clear lane is usually the rumbles lane, and the OTV drives straight over the rumbles.
4. **If no lane is clear** (the sensor saw the rumbles): it drives a lane that is open at column 1, stops in the 0.6 m gap between the columns, moves sideways to a lane that is open at column 2, then continues.
5. In every lane it drives about 8 cm left of center, so the arm and the left wheels have equal room.
6. Stops at x = 2.9 m.

### Nav3 – Goal zone
1. Moves sideways to the **limbo line** (y ≈ 1.58: about 8 cm left of the opening's center, so the arm clears the right side wall).
2. Drives straight under the limbo.
3. **If the limbo rod hides the marker**, the OTV keeps driving straight and estimates its x from the speed it measured before. It stops waiting if the marker stays hidden for more than 40 cm.
4. Stops at x = 3.7 m (middle of the goal zone).

---

## 6. First test run: what to watch

Lower the speeds a bit for the first runs (`DRIVE_SPEED` 170, `TURN_SPEED` 140 in all three files). If the motors buzz but don't move, raise them back.

| Program | Watch for | If it goes wrong |
|---|---|---|
| All | First turn turns once, then stops | Spins forever → left and right pin groups are swapped. Swings back and forth → lower `TURN_SPEED`. |
| Nav1 | Gap between the OTV and the site | More than 150 mm → lower `SIDE_DIST`. Bumps the site → raise it. The OTV stops about 4 cm before its parking spot. |
| Nav2 | Scan position, x ≈ 1.05 | Clearly past 1.10 → set `SCAN_X = 1.0`. |
| Nav2 | Printed `lane _ hit at x=...` | Column 1 should be about 1.40, column 2 about 2.20. Always off by the same amount → change `FRONT` (prints 3 cm too high → lower `FRONT` by 0.03). |
| Nav2 | Driving over the rumbles | Stops in front of them and waits → set `SAFETY_CM = 0`. |
| Nav3 | Push the OTV under the limbo rod by hand first | If "ArUco Visible" turns NO, that's expected: the code handles it. |
| Nav3 | Distance to the limbo side walls | Should be about 9 cm on each side. Closer → re-measure `ARM_REACH` and `HALF_WIDTH`. |

---

## 7. Troubleshooting

| What you see | Likely cause | Fix |
|---|---|---|
| Doesn't move at all | Not connected, or camera can't see the marker | System Printouts should say "The Harvesters has connected", and ArUco Visible should be YES. Is the battery on? |
| Drives the wrong way, gets more and more lost | Marker rotated on the OTV | Redo the Theta check in section 2 |
| Stops in the middle of a run | Marker hidden (glare, arm, cable), or something within 6 cm ahead | Watch ArUco Visible; for the rumbles see `SAFETY_CM` above |
| Prints "no path found" | Ultrasonic not reading | Check pins 12 and 11 and the sensor's 5 V/GND |
| Prints "WARNING gap too small for the arm" | `ARM_REACH` too large | Re-measure the arm |
| Stuck under the limbo | Marker hidden for more than 40 cm | Start Nav3 farther back, or raise `BLIND_MAX` |

---

## 8. All settings

When you change a value, **change it in every file that has it**, then upload again.

| Setting | Value | In | What it is |
|---|---|---|---|
| `MARKER_ID` | 205 | all | our ArUco ID (**confirm it**: 205 is also the library's example number) |
| `ROOM` | 1201 | all | Vision System room |
| `DRIVE_SPEED`, `TURN_SPEED` | 200, 170 | all | motor PWM (0–255) |
| `TURN_TOL`, `BIG_ERR` | 0.10, 0.50 rad | all | turn accuracy (~6°); above ~30° off, stop and turn first |
| `K_HEADING` | 100 | all | steering strength. Raise it if the OTV drifts, lower it if it wiggles |
| `MISSION_X`, `MISSION_Y_TOP`, `MISSION_Y_BOTTOM` | 0.5, 1.5, 0.5 | Nav1 | spots A and B (Arena Details) |
| `SIDE_DIST` | 0.35 m | Nav1 | parking distance from the site center (**tune**) |
| `ARM_REACH` | 0.27 m | Nav2, Nav3 | OTV center to arm tip (**measure**) |
| `HALF_WIDTH` | 0.115 m | Nav2, Nav3 | OTV center to outside of the left wheels (**measure**) |
| `TURN_R` | 0.17 m | Nav2 | OTV center to farthest wheel corner (**measure**) |
| `FRONT` | 0.10 m | Nav2 | OTV center to the ultrasonic sensor (**measure**) |
| `LANES` | 0.5, 1.0, 1.5 | Nav2 | obstacle rows (Arena Details) |
| `SCAN_X` | 1.05 m | Nav2 | where the lanes are checked |
| `PAST_OBSTACLES_X` | 2.9 m | Nav2 | Nav2 finish line |
| `SAFETY_CM` | 6 cm | Nav2 | emergency stop distance (0 = off) |
| `LIMBO_Y` | 1.5 | Nav3 | center of the limbo opening |
| `DEST_X` | 3.7 m | Nav3 | Nav3 stop line |
| `BLIND_MAX` | 0.4 m | Nav3 | farthest to drive while the marker is hidden |

---

## 9. Limits and assumptions

- **Arm height:** the arm passes **over** the mission site (end of Nav1, start of Nav2) and **under** the limbo rod (18.4 cm, CAD height about 17.9 cm). Check both on the real OTV.
- **Arm shape:** the code assumes a thin arm sticking straight out from the middle of the right side. If it is longer, thicker or angled, tell the programmer.
- **Clearance is small:** the closest the arm gets to a solid obstacle in Nav2 is about 5 cm, so watch the first runs.
- **Rumbles:** assumed low enough to drive over.
- **Messages** like `Nav2: lane 0 hit at x=1.40` print on the Vision System page and help with tuning. They are hidden during the official demo, which is normal.

## 10. Testing done

All three programs compile. They were run in a computer simulation of our OTV (CAD body plus the right-side arm, MS5 motor wiring) on the official arena. The simulation covered the official start spots and headings and every legal obstacle layout, with the rumbles both invisible to the sensor and seen as solid. It also included the limbo side walls and rod and the log, plus camera dropouts, position noise and bad ultrasonic readings. There were no collisions. In Nav2 the arm stayed at least about 5 cm from solid obstacles; in Nav3 it stayed about 9 cm from the limbo walls. The simulation is simplified, so always test on the real arena.
