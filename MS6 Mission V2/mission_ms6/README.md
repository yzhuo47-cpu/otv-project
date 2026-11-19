# MS6 Mission Code V2 — The Harvesters (Seed Planting)

This code does the **mission part only** (no navigation). For MS6 each objective can be shown on its own, so we put the OTV next to the site by hand and let it do the mission.

**V2:** no calibration step. The IR values from our earlier calibration are already in the code. Marker ID is **147**.

**Board:** Arduino UNO R4 WiFi (school-issued, Wi-Fi built in)

---

## 1. Overview

Choose what the code does with `MODE` at the top of `mission_ms6.ino`:

| `MODE` | Where the OTV starts | What it does | MS6 objective |
|---|---|---|---|
| 1 | Front of the site | Plants the seed → backs up one plot → scoops rocks | Mission II + III |
| 2 | Back of the site | Reads the two far plots → sends the result | Mission I |

**Order for today:**
1. Dry run (MODE 1, OTV held in the air): checks the servos turn the right way
2. MODE 2: Mission I
3. MODE 1: Mission II + III

**Who moves the OTV?** You place it at the start by hand. Between two plots (8.4 cm), the OTV moves by itself.

The connection test `vision_test` is in a separate zip, **Vision System Test.zip**. You only need it if the OTV stops after its first step (see section 7).

---

## 2. Before you start

**Bring:** charged battery, lima beans, ruler, thin stick (to check the seed depth)

**Check the OTV:**
- Marker **147** is flat on top, facing up, not covered by the arm or wires, white border visible, no shiny tape on it
- Wiring:

| Part | Pin |
|---|---|
| Upper servo (lifts the arm, closer to the OTV) | A1 |
| Lower servo (turns the bucket) | A2 |
| IR sensor (TCRT5000, AO) | A0 |
| Right motors: ENA / IN1 / IN2 | 10 / 9 / 8 |
| Left motors: ENB / IN3 / IN4 | 5 / 7 / 6 |

- If the IR sensor's VCC was moved to the Arduino 5V pin for testing, put it back where it was

**Check the laptop:**
- ENES100 library and ArduinoHttpClient library installed ✓
- Tools → Board → **Arduino UNO R4 WiFi**, and the right Port

**Words used below:**
- **Right side** = stand behind the OTV with its front (ultrasonic sensor) pointing away from you. The arm is on your right
- **Near / far, left / right plots** = as seen from the **front** of the site (from the middle of the arena). Near-left is always orzo, near-right is always rocks

---

## 3. How to upload (same every time)

**Never have USB plugged in and the power switch ON at the same time.**

1. OTV power switch **OFF**
2. Change `MODE` at the top of `mission_ms6.ino` (if needed)
3. Plug in USB → click **Upload**
4. **Unplug USB**
5. Place the OTV at the starting spot
6. Power switch **ON** → the program starts after 3 s

To run the same program again: press **RESET** on the Arduino. No need to upload again.

---

## 4. Step 1 — Dry run (checks the servo directions)

The servo angles are a best guess. Check them once, away from the site, so a wrong direction can't hit the plots.

1. Set `MODE = 1`, upload, unplug USB
2. Hold the OTV in the air (or put it on a box) **away from the site**, wheels off the ground
3. Power ON and watch:
   - Right away: the arm goes **up**
   - After 3 s: the arm goes **down** → the bucket turns one way (scoop) → turns the other way (pour) → the arm goes up → the bucket goes back to level
4. Power OFF. If something was wrong, change it at the top of the code:

| What you saw | Change |
|---|---|
| The arm went **down** right away | `ARM_UP = 105;` and `ARM_DOWN = 78;` |
| The bucket turned the wrong way first (it should scoop first, toward the substrate) | swap the numbers of `BUCKET_SCOOP` and `BUCKET_DUMP` |
| A servo buzzes or shakes and doesn't move | It is pushing on something: power OFF right away, check that the arm can move freely |

If the vision system isn't connected, the dry run stops after the seed step (it waits to back up). That is normal here: just turn the power off.

---

## 5. Step 2 — Mission I (MODE 2)

1. Set `MODE = 2`, upload, unplug USB, power OFF
2. Open **enes100.umd.edu → Student Tools → Vision System → 1201**, choose **The Harvesters** in the Team list
3. Place the OTV at the **back** of the site (between the site and the wall):
   - right side facing the site, parallel to the edge
   - chassis side **9.3 cm** from the site edge (measure at both ends)
   - **IR above the middle of the far plot toward the OTV's front**
   - drive or carry it in already facing the right way: don't spin in the gap, the arm would hit the wall
4. Power ON, take your hands away. Check the page: **"The Harvesters has connected"** at the bottom, and **Visible = YES** in the table
5. After 3 s it:
   reads the first plot (from the back this is **far-right**) → backs up 8.4 cm → reads **far-left** → sends the result
6. The print box on the page shows:
   ```
   MISSION MESSAGE: plot locations = DIAGONAL
   Seen from the front of the site:
   far-left: rocks (IR 24.1)
   far-right: orzo (IR 20.0)
   ```
   Check that `far-left` and `far-right` match the real plots

```
BACK SIDE (top view)

   back <-[============ OTV ============]-> front
                (right side faces the site)
                             IR starts here v
        [ far-left  ] [ far-right  ]
        [ near-left ] [ near-right ]
```

**How it decides orzo or rocks:** our calibration gave orzo about **19–21** and rocks about **23–25**. Below **21.5** = orzo, above = rocks. Each reading is the average of 50 samples. If a reading is very close to 21.5 (between 21.0 and 22.0), it scans again, up to 4 times, and uses the average.

---

## 6. Step 3 — Mission II + III (MODE 1)

1. Set `MODE = 1`, upload, unplug USB, power OFF
2. Open the vision system page (1201), choose **The Harvesters**
3. Put the **lima bean** in the bucket
4. Place the OTV at the **front** of the site:
   - right side facing the site, **front of the OTV toward the orzo side**
   - chassis side **10.5 cm** from the site edge (measure at both ends)
   - **bucket above the middle of the near-left plot (orzo)**
5. Power ON, take your hands away. Check the page: **"The Harvesters has connected"** at the bottom, and **Visible = YES** in the table
6. After 3 s it:
   1. **plants the seed:** arm down → scoops a hole, the bean drops in → pours the orzo back → arm up
   2. **backs up 8.4 cm** → bucket above the near-right plot
   3. **scoops rocks:** arm down → scoop → arm up (bucket stays tilted so the rocks stay in)
7. Check:
   - **Seed:** push a thin stick down until it touches the bean. **The top of the bean must be at least 1 cm deep.** (2 cm of digging gives about 1.3 cm, so check every run)
   - **Rocks:** take them out and weigh them: **at least 10 g**

```
FRONT SIDE (top view)

        [ far-left  ] [ far-right  ]
        [ near-left ] [ near-right ]
        [   ORZO    ] [   ROCKS    ]
             ^ bucket starts here
   front <-[============ OTV ============]-> back
                (right side faces the site)
```

---

## 7. If something goes wrong

**Something goes wrong mid-run: turn the power OFF right away.**

| Problem | Fix (top of the code) |
|---|---|
| Bucket stops above the orzo, or a servo buzzes against the plot | The 90° guess is off: change `ARM_DOWN` 3–5° further down, and `ARM_UP` by the same amount |
| Bean not deep enough | Move `ARM_DOWN` 1–2° further down. The wrist servo must not go into the orzo |
| Bean not covered | Change `BUCKET_DUMP` by 5–10° |
| Rocks fall out when lifting | Turn `BUCKET_SCOOP` 5–10° further in the scooping direction |
| Stops after the first step and never moves | Not connected, or the camera can't see marker 147. Run `vision_test` (**Vision System Test.zip**) to find out which |
| OTV goes **forward** instead of back | In `driveBackward()`, swap `HIGH` and `LOW` on IN1–IN4 |
| OTV **spins** instead of backing up | One side is wired the other way: swap `HIGH`/`LOW` for that side only (IN1/IN2 = right, IN3/IN4 = left) |
| Keeps backing up for 15 s | The camera lost the marker while moving |
| Mission I: one plot is wrong | Compare that plot's IR number on the page with 21.5. If our two numbers are far from 19–25 (for example in the hundreds), the sensor height or position is different from the calibration: tell Jonathan |
| Mission I: left and right are swapped | The OTV faced the wrong way at the back. The IR must start above the far plot toward the OTV's **front** |

---

## 8. How the code works (for instructor questions)

- **Mission II / III need no sensor:** the near-left plot is always orzo and the near-right is always rocks (official rule).
- **IR:** orzo and rocks reflect IR differently. From our calibration, orzo reads about 19–21 and rocks about 23–25, so the cutoff is 21.5. The difference is small, so each reading averages 50 samples, and a reading close to the cutoff is scanned again.
- **Why read from the back:** in our CAD, the IR arm only reaches the row closest to the OTV. From the back, the far row is the closest.
- **Left / right flip:** from the back, the far plot toward the OTV's front is far-RIGHT. The code reads it first.
- **Moving one plot:** the vision system only gives position (x, y), so the code saves the start position and works out the distance after each short full-power pulse. It stops at 8.4 cm. Position updates every 250 ms, so it waits 350 ms between pulses. A position of −1 (marker not seen) is ignored.
- **Arm angles:** from a simulation of the CAD front view. Digging 12° past "just touching" goes about 2 cm deep; more would push the wrist servo into the orzo. Lifting 15° keeps the bucket about 3 cm above the plots and under the 18.4 cm limbo.
- **Servos move 1° every 15 ms** so the bucket doesn't throw the bean or the substrate.

---

## 9. Later: full run with navigation

Navigate to the front of the site → `frontSide()` → drive around to the back → `missionOne()` → continue to the obstacles (keep the rocks in the bucket until the end).

- Parking accuracy needed (from the CAD): about ±2.4 cm in position and ±3° in heading.
- When merging, test the navigation speed control (`analogWrite`) with the servos attached. If one side stops responding, move that speed pin to another PWM pin (timer conflict).
