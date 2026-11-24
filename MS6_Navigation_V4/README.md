# MS6 Navigation V4 – The Harvesters

Code for the three MS6 navigation objectives, on the real OTV (school **Arduino UNO R4 WiFi**).
Each objective is its own program. Upload the one you are about to demo.

| Program | Objective | Start the OTV | It stops when |
|---|---|---|---|
| `Nav1_MissionSite_V4` | Within 150 mm of the mission site | Spot A or B in the landing zone | Parked beside the site, arm side facing it |
| `Nav2_Obstacles_V4` | Completely past the 3 obstacles (2 boxes + rumbles, all avoided) | Anywhere before x = 0.9 m (for example where Nav1 stopped) | x = 2.9 m |
| `Nav3_Destination_V4` | Completely into the goal zone | After the obstacles (where Nav2 stopped) | x = 3.7 m |
| `Ultrasonic_Test_V4` | Not an objective: checks the ultrasonic sensor by itself | Anywhere (on the table is fine) | Never; it keeps printing distances |

The OTV does **not** need to go back to the start between programs. Each one starts where the last one stopped.

**New in V4:** more room at the scan point. If a box is closer than 32 cm (was 25), the OTV backs up before turning, but never back into the mission site (`SCAN_MIN_X`). It scans the other lanes 50 cm from the box it measured (was 45). Everywhere else the OTV is already centered between obstacles (lanes 50 cm wide, the gap between the columns 60 cm, the limbo opening 56 cm), so the room there is already the most the arena allows. Delete the old versions from the Arduino folder so you don't upload them by mistake.

**From V3** (from the V2 test: the IR sensor hit a column 1 box while turning at the scan point, and the OTV drifted on straight lines):
- **Nav2 keeps its distance by measuring, not only by the official numbers.** The obstacles may not sit exactly at the official x, and the camera position of a raised marker is off by a few cm. So the scan point is farther back (x = 0.9). If a box is closer than 32 cm there, the OTV backs up before turning, and it scans the other lanes 50 cm from the box it really measured. It also works out how far the columns are from the official x, and at the lane change it stops by the column 2 obstacle it sees ahead (about 20 cm), so it ends up in the middle of the real gap.
- **All three drive straighter.** Turns stop within 3° (was 6°), and the OTV steers back onto its line faster (`LOOKAHEAD` 0.3 m, was 0.5).
- **Turns can't swing back and forth forever.** Bursts can be as short as 30 ms, and a turn stops after 12 bursts.

**From V2:**
- **All three:** they handle the camera position updating only every 250 ms. Turns happen in bursts, and the OTV stops near each stop point, waits for a fresh position, then finishes with short nudges (section 5). Short pauses are normal.
- **Nav2, ultrasonic:** it no longer misses echoes when WiFi is on, and leaves at least 60 ms between readings (section 7). It prints the reading on the Vision System page about twice a second.
- **Nav2, gap check:** after moving into the second lane, it checks column 2 again from about 20 cm away before going on (section 5).
- **Nav2, prints:** each message goes out in one print call, so messages aren't dropped.
- **Ultrasonic_Test_V4:** a new test for the sensor alone, using the Serial Monitor.

---

## Quick checklist

1. ENES100 library **v6.2.4** installed, board set to **Arduino UNO R4 WiFi**
2. Our ArUco marker shows up in the Vision System's **"Seen markers"**, and that number matches `MARKER_ID`
3. `ROOM = 1201` (change it if we demo in another room)
4. Motor battery **off** while uploading
5. The arm is **higher than the mission site** and **lower than 18.4 cm** (the limbo)
6. `Ultrasonic_Test_V4` shows the right distances (section 7) before running Nav2

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
- The code expects the marker center **6 cm behind the OTV center** (`MARKER_BACK = 0.06`). If you move the marker, measure again and update `MARKER_BACK` in all three files.

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
| 5V, GND | HC-SR04 VCC, GND | Ultrasonic power (**5 V, not 3.3 V**) |

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

**How all three move.** On the R4, the ENES100 library gets a new position from the Vision System only every **250 ms**, and it also arrives a bit late. So the code never turns or drives "until the camera says we're there", because that overshoots. Instead:
- **Turns** happen in bursts: turn for about 80% of the remaining angle, stop, wait for a fresh position (`SETTLE_MS`), and repeat.
- **Stops at a point** (a lane line, the scan point, the lane-change point, the parking spot): drive normally until `APPROACH` (15 cm) away, stop, wait for a fresh position, then finish with short nudges.
- The code **measures the real turning and driving speed** during the run, so the bursts and nudges get the right length on their own. There is nothing to calibrate.

So **short pauses before every turn and every stop are normal.** That is the OTV waiting for a fresh position.

### Nav1 – Mission site
1. Works out where the site is: on the spot we didn't start on (A at y = 1.5, or B at y = 0.5).
2. Drives to a parking spot on **the side of the site that faces us**, `SIDE_DIST` from the site center.
3. Turns 90° so the **right side (arm) faces the site**: facing +x if the site is below, facing −x if it is above.

### Nav2 – Obstacles
The rumbles can't be driven over, so all three obstacles are avoided. By the arena rules there is **one obstacle in every row** (two boxes and the rumbles), so **every lane is blocked once** and the OTV always switches lanes once, in the 0.6 m gap between the two columns.

1. Drives to x = 0.9 m (about 50 cm in front of column 1).
2. **Stops in each of the 3 lanes** and uses the ultrasonic to see where each lane is blocked. This is the "looking" step, not the OTV getting stuck. If a box is closer than `MIN_SCAN_CM` (32 cm), it backs up before turning (never behind `SCAN_MIN_X`, so it can't back into the mission site). After the first box it measures, it scans the remaining lanes `SCAN_ROOM` (50 cm) from that box.
3. Sorts each hit into column 1 (x ≈ 1.4) or column 2 (x ≈ 2.2). It also works out how far the real columns are from the official x (printed `columns are __ cm from the official x`) and moves the lane-change point by that much.
4. **Finds the rumbles even if the sensor can't see them** (they are low):
   - The lane with no hit is the rumbles lane.
   - If both boxes are in the same column, the rumbles must be in the other column (rule: two in one column, one in the other).
   - Otherwise the rumbles' column is unknown, so the OTV never uses that lane.
5. Drives a lane that is **blocked at column 2** (so open at column 1), stops in the gap, and moves sideways to a lane **blocked at column 1** (so open at column 2). This path always exists. In the gap it stops **by the ultrasonic**: when the column 2 obstacle ahead is about 20 cm away, which is the middle of the real gap. If the sensor can't see it (the rumbles), it uses the camera. While moving sideways it holds the x where it stopped.
6. **Checks again in the gap.** Column 2 is only about 20 cm ahead there, so the ultrasonic sees it clearly. If something is within `CLEAR_CM` (60 cm), the plan was wrong (a bad reading during the scan). It then moves to the third lane, but only if the scan saw that lane blocked at column 1, and checks again. If it can't confirm a lane is open, it stops in the gap instead of guessing.
7. Stops at x = 2.9 m.

### Nav3 – Goal zone
1. Moves sideways to the **limbo line**: the opening's center (y = 1.5), shifted left by half of (arm − wheels), so the arm and the left wheels have the same room to the side walls.
2. Drives straight under the limbo.
3. **If the limbo rod hides the marker**, the OTV keeps driving straight and estimates its x from the speed it measured before. It stops waiting if the marker stays hidden for more than 40 cm.
4. Stops at x = 3.7 m (middle of the goal zone).

---

## 6. First test run: what to watch

Lower the speeds a bit for the first runs (`DRIVE_SPEED` 170, `TURN_SPEED` 140 in all three files). If the motors buzz but don't move, raise them back.

| Program | Watch for | If it goes wrong |
|---|---|---|
| All | Turns in 1–3 bursts with short pauses, then stops | Spins forever → left and right pin groups are swapped. Swings back and forth many times → lower `TURN_SPEED`. |
| Nav1 | Gap between the OTV and the site | More than 150 mm → lower `SIDE_DIST`. Bumps the site → raise it. The OTV stops about 4 cm before its parking spot. |
| Nav2 | `ultrasonic __ cm`, printed right after `start obstacles` (before it moves) and then about twice a second while it drives straight or scans | A number → the sensor works. `ultrasonic no echo` over and over → stop and see section 7. Nothing prints while it turns or moves sideways (the sensor isn't used then). |
| Nav2 | Scan position, about 45–50 cm in front of column 1 | Closer than 35 cm → set `SCAN_X` 5 cm lower. |
| Nav2 | Printed `columns are __ cm from the official x` | Within ±10 cm is normal (placement and camera error). If it's always about the same, the obstacles really sit there. |
| Nav2 | Printed `lane _ dist __ cm, hit at x = ...` | A lane blocked at column 1 reads about 40 cm (hit x ≈ 1.40), at column 2 about 120 cm (hit x ≈ 2.20). Always off by the same amount → change `FRONT` (prints 3 cm too high → lower `FRONT` by 0.03). `no echo` on every lane → section 7. |
| Nav2 | The planned lanes (printed `lane _ then _`) | The first lane should be blocked at column 2, the second at column 1. If it chose a lane with the rumbles, check the `hit at` numbers. |
| Nav2 | Gap check (printed `lane _ is clear, going on`) | Normal. `lane _ is blocked at column 2` means the scan was wrong. It tries the third lane or stops; then check the `hit at` numbers. |
| Nav3 | Push the OTV under the limbo rod by hand first | If "ArUco Visible" turns NO, that's expected: the code handles it. |
| Nav3 | Distance to the limbo side walls | Should be about 15 cm on each side. Much closer → check `ARM_REACH`, `HALF_WIDTH` and `MARKER_BACK`. |

---

## 7. Troubleshooting

| What you see | Likely cause | Fix |
|---|---|---|
| Doesn't move at all | Not connected, or camera can't see the marker | System Printouts should say "The Harvesters has connected", and ArUco Visible should be YES. Is the battery on? |
| Drives the wrong way, gets more and more lost | Marker rotated on the OTV | Redo the Theta check in section 2 |
| Stops in the middle of a run | Short pauses (about 0.5 s) before turns and stops are normal. A long stop: marker hidden (glare, arm, cable), or something within 12 cm ahead | Watch ArUco Visible |
| Prints "saw fewer than 2 obstacles" or "no path found" | Ultrasonic not reading the boxes | The OTV stops safely at the scan point. Run `Ultrasonic_Test_V4` (below) |
| Prints "no safe lane left, stopping" or "blocked too, stopping" | The scan misread a lane; the gap check caught it | The OTV stops safely in the gap. Check the `lane _ dist` numbers from the scan |
| Positions all seem a few cm off | Marker offset wrong | Re-measure `MARKER_BACK` |
| Prints "WARNING gap too small for the arm" | `ARM_REACH` too large | Re-measure the arm |
| Stuck under the limbo | Marker hidden for more than 40 cm | Start Nav3 farther back, or raise `BLIND_MAX` |

### Ultrasonic not seeing the obstacles

The boxes are tall and flat, so a working sensor sees them easily. If it doesn't, the problem is almost always wiring, power or which way the sensor points. `Ultrasonic_Test_V4` tests the sensor alone:

1. Upload `Ultrasonic_Test_V4`, then **turn the battery on** (lift the wheels off the table first). If the sensor's 5 V comes from the CPT converter or the breadboard rail, it has no power with USB alone.
2. Open **Tools → Serial Monitor**, set **9600 baud**. It prints about 5 readings per second.
3. Hold your hand (or a book) **10 cm, 30 cm, 100 cm** in front of the sensor.

| What it prints | Meaning | Fix |
|---|---|---|
| Changes correctly with your hand | Sensor and wiring are fine | Check the mounting on the OTV (next table) |
| Always `no echo` | No signal back | In order: (1) battery on; (2) in the test, change to `TRIG = 11`, `ECHO = 12` and upload: if numbers appear, the two wires are swapped; (3) multimeter on the sensor's VCC–GND: about 5 V; (4) push every jumper in; (5) try another HC-SR04 |
| The same small number (2–10 cm) even with nothing in front | The sensor sees part of the OTV | Something in front of it: the body wall, the lid, the arm, a wire. Move the sensor to the very front edge |
| Jumps around randomly | Loose wire or bad sensor | Re-seat the wires, then try another sensor |

**The test works but Nav2 didn't.** There were two reasons, and both are fixed in this version:
1. **The WiFi task interrupted the timing.** On the R4, the ENES100 library runs WiFi as a background FreeRTOS task with a higher priority than our code. Every 10 ms it can pause our code, so `pulseIn` missed the echo and reported nothing. The test program doesn't connect to WiFi, so it never had this problem. Nav2 now pauses task switching (`vTaskSuspendAll`) only while it times the echo, at most 30 ms. Interrupts keep running, so no WiFi data is lost.
2. **Readings were too fast.** Old Nav2 fired the sensor every 10 ms, but the HC-SR04 needs at least 60 ms between readings. Nav2 now waits `PING_GAP_MS` (60 ms).

If Nav2 still prints `no echo` with this version, check how it is mounted:

| Check | Why |
|---|---|
| It is on the side that leads when the OTV drives **forward** | Nav2 faces +x and looks straight ahead |
| Both round "eyes" point **straight ahead, level** | Tilted down → sees the floor; tilted up or sideways → misses the box |
| Nothing within about 15° of its beam (arm, scoop, wheels, marker paper) | The beam is a cone, not a thin line |
| Mounted lower than the box tops | The boxes are tall, so any normal height works |

---

## 8. All settings

When you change a value, **change it in every file that has it**, then upload again.

| Setting | Value | In | What it is |
|---|---|---|---|
| `MARKER_ID` | 205 | all | our ArUco ID (**confirm it**: 205 is also the library's example number) |
| `ROOM` | 1201 | all | Vision System room |
| `MARKER_BACK` | 0.06 m | all | marker center is this far behind the OTV center (**measured**) |
| `DRIVE_SPEED`, `TURN_SPEED` | 200, 170 | all | motor PWM (0–255) |
| `TURN_TOL`, `BIG_ERR` | 0.05, 0.50 rad | all | turn accuracy (~3°); above ~30° off, stop and turn first |
| `K_HEADING` | 100 | all | steering strength. Raise it if the OTV drifts, lower it if it wiggles |
| `LOOKAHEAD` | 0.3 m | Nav2, Nav3 | steer toward a point this far ahead on the line; smaller = gets back onto the line faster |
| `SETTLE_MS` | 500 ms | all | after stopping, wait this long for a fresh camera position (position updates every 250 ms, plus delay) |
| `APPROACH` | 0.15 m | all | drive normally until this close to a stop point, then finish with nudges |
| `MISSION_X`, `MISSION_Y_TOP`, `MISSION_Y_BOTTOM` | 0.5, 1.5, 0.5 | Nav1 | spots A and B (Arena Details) |
| `SIDE_DIST` | 0.35 m | Nav1 | parking distance from the site center (**tune**) |
| `ARM_REACH` | 0.14 m | Nav2, Nav3 | OTV center to the farthest point of the arm (the IR sensor, right-rear; **measured**) |
| `HALF_WIDTH` | 0.115 m | Nav2, Nav3 | OTV center to outside of the left wheels (**measure**) |
| `TURN_R` | 0.17 m | Nav2 | OTV center to farthest wheel corner (**measure**) |
| `FRONT` | 0.10 m | Nav2 | OTV center to the ultrasonic sensor (**measure**) |
| `LANES` | 0.5, 1.0, 1.5 | Nav2 | obstacle rows (Arena Details) |
| `COL1_X`, `COL2_X` | 1.5, 2.3 | Nav2 | obstacle columns (Arena Details); used to sort hits and to place the lane change |
| `SCAN_X` | 0.9 m | Nav2 | where the lanes are checked |
| `SCAN_ROOM` | 0.50 m | Nav2 | after measuring a column 1 box, scan the other lanes this far (OTV center) from it |
| `SCAN_MIN_X` | 0.85 m | Nav2 | never scan or back up behind this x (the mission site ends at about 0.65–0.70) |
| `MIN_SCAN_CM` | 32 cm | Nav2 | at the scan point, a box closer than this makes the OTV back up before turning |
| `PAST_OBSTACLES_X` | 2.9 m | Nav2 | Nav2 finish line |
| `REPORT_MS` | 500 ms | Nav2 | how often the ultrasonic reading is printed on the Vision System page |
| `PING_GAP_MS` | 60 ms | Nav2 | minimum time between two ultrasonic readings (HC-SR04 datasheet). Don't lower it |
| `SAFETY_CM` | 12 cm | Nav2 | backup emergency stop distance (0 = off). Keep it at 15 or less: at the lane-change point column 2 is about 20 cm ahead, and at 20 the OTV stops there and waits forever |
| `CLEAR_CM` | 60 cm | Nav2 | gap check: if blocked, column 2 is about 20 cm ahead; nothing within 60 cm means the lane is open |
| `LIMBO_Y` | 1.5 | Nav3 | center of the limbo opening |
| `DEST_X` | 3.7 m | Nav3 | Nav3 stop line |
| `BLIND_MAX` | 0.4 m | Nav3 | farthest to drive while the marker is hidden |

---

## 9. Limits and assumptions

- **Arm height:** the arm passes **over** the mission site (end of Nav1, start of Nav2) and **under** the limbo rod (18.4 cm, CAD height about 17.9 cm). Check both on the real OTV.
- **Arm shape:** the farthest point of the arm is the IR sensor, 14 cm from the OTV center toward the right-rear (measured). If it changes, update `ARM_REACH`.
- **Clearance:** in the simulation the OTV stays about 10 cm from every obstacle (including the rumbles) in Nav2. Still, watch the first runs.
- **Rumbles:** always avoided. The plan relies on the arena rules (one obstacle per row; two in one column, one in the other).
- **Messages** like `Nav2: lane 0 dist 25.00 cm, hit at x = 1.40` print on the Vision System page and help with tuning. They are hidden during the official demo, which is normal.

## 10. Testing done

All three programs compile. They were run in a computer simulation of our OTV (CAD body plus the right-side arm, MS5 motor wiring) on the official arena. The simulation covered the official start spots and headings and every legal obstacle layout, with the rumbles both invisible to the sensor and seen as solid. It also included the limbo side walls and rod and the log, plus camera dropouts, position noise and bad ultrasonic readings. The OTV model used the measured arm (14 cm) and the marker 6 cm behind the center, and touching the rumbles counted as a collision. There were no collisions in 576 Nav2 runs, with every part of the OTV at least about 10 cm from the obstacles. Nav1 and Nav3 finished every run, and Nav3 stayed about 16 cm from the limbo walls. With an arm 4 cm longer than measured, Nav2 still had no collisions, with about 7 cm of clearance. With a dead ultrasonic, Nav2 stopped at the scan point and printed "saw fewer than 2 obstacles" without hitting anything.

**Camera timing.** The ENES100 R4 library gets a new position only every 250 ms. With that, plus 100 ms of delay, the previous version hit an obstacle in 46 of 48 Nav2 runs. With 200 ms of delay, every run of all three programs failed (it got stuck turning back and forth). The current version was tested with that timing and 0, 100 or 200 ms of delay, at car speeds from 0.08 to 0.3 m/s, with noise and the limbo rod hiding the marker. There were no collisions in about 1,000 Nav2 runs, and every part of the OTV stayed at least about 10 cm from the obstacles. Nav1 parked within about 5 cm of its spot, and Nav3 reached the goal zone every time. Nav2 is slower because of the pauses: about 65 s in the simulation (the scan of the 3 lanes is about 25 s of that), instead of 40 s. The gap check was also tested with an extra box placed in column 2 of the planned lane (36 runs): the OTV switched to the third lane and finished, or stopped safely, with no collisions. With normal layouts the gap check never fired.

**V3: real arena errors.** The simulation was given the errors that probably caused the V2 crash: obstacles placed 10 or 20 cm closer (or 10 cm farther) than the official x, the camera error of a raised marker (10–15 cm high), and one motor 10% weaker. V2 hit obstacles in up to every run (18 of 48 with the obstacles 10 cm closer and a 15 cm marker). V3 had no collisions in any of these cases (about 1,000 runs, including noise, 200 ms camera delay and car speeds from 0.08 to 0.3 m/s), with 2–11 cm to spare depending on how large the errors were. The IR sensor was modeled at the right-rear, 14 cm from the center. Nav2 takes about 60–70 s in the simulation. The simulation is simplified, so always test on the real arena.
