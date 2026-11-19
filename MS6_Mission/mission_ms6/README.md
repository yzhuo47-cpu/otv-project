# MS6 Mission Code — The Harvesters (Seed Planting)

This code does the **mission part only** (no navigation). For MS6 each objective can be shown on its own, so we put the OTV next to the mission site by hand and let it do the mission.

**Board:** Arduino UNO R4 WiFi (school-issued, Wi-Fi built in)

> Ask the instructor first: is it OK to place the OTV next to the site by hand for the MS6 demo?

---

## 1. Overview

`mission_ms6` is the mission code. Choose what it does with `MODE` at the top of the file.

| `MODE` | Where the OTV starts | What it does | MS6 objective |
|---|---|---|---|
| 0 | Front of the site | Calibration: finds the servo angles and the IR threshold | (setup) |
| 1 | Front of the site | Plants the seed → backs up one plot → scoops rocks | Mission II + III |
| 2 | Back of the site | Reads the two far plots → sends the result | Mission I |

**Order for today:** MODE 0 → MODE 2 (Mission I) → MODE 1 (Mission II + III)

The connection test `vision_test` is in a separate zip, **Vision System Test.zip**. You only need it if the OTV stops after its first step (see section 7).

**Who moves the OTV?** You place it at the start by hand. Between two plots (8.4 cm), the OTV moves by itself.

---

## 2. Before you start

**Bring:** charged battery, lima beans, ruler, thin stick (to check the seed depth), black tape

**Check the OTV:**
- Marker 205 is flat on top, facing up, not covered by the arm or wires, white border visible, no shiny tape on it
- Wiring:

| Part | Pin |
|---|---|
| Upper servo (lifts the arm, closer to the OTV) | A1 |
| Lower servo (turns the bucket) | A2 |
| IR sensor (TCRT5000, AO) | A0 |
| Right motors: ENA / IN1 / IN2 | 10 / 9 / 8 |
| Left motors: ENB / IN3 / IN4 | 5 / 7 / 6 |

- Servo power comes from the 5 V buck converter (not the Arduino 5 V pin). All GND connected together

**Check the laptop (Deem's):**
- ENES100 library installed ✓
- Tools → Board → Boards Manager → **Arduino UNO R4 Boards** installed
- Tools → Board → **Arduino UNO R4 WiFi**, and the right Port

**Words used below:**
- **Right side** = stand behind the OTV with its front (ultrasonic sensor) pointing away from you. The arm is on your right
- **Near / far, left / right plots** = as seen from the **front** of the site (from the middle of the arena). Near-left is always orzo, near-right is always rocks

---

## 3. How to upload (same every time)

1. OTV power switch **OFF**
2. Change `MODE` at the top of `mission_ms6.ino` (if needed)
3. Plug in USB → click **Upload**
4. Unplug USB. (For MODE 0, keep it plugged in to use the Serial Monitor)
5. Place the OTV at the starting spot
6. Power switch **ON** → the program starts

To run the same program again: press **RESET** on the Arduino. No need to upload again.

---

## 4. Step 1 — Calibrate (MODE 0)

Set `MODE = 0`, upload. **Keep USB in, power ON.** (MODE 0 doesn't use the vision system.) Serial Monitor: **9600** and **Newline**.
At power-on both servos go to 90 (**hold the arm**, this first move jumps). Every second it prints:
`IR: 612   arm: 90   bucket: 90`

Type a command in the box at the top and press Enter. After every command it prints the setting lines to copy.

| Command | What it does |
|---|---|
| `a 85` | move the arm servo to 85° |
| `b 90` | move the bucket servo to 90° |
| `t` | save: the bucket **just touches** the orzo now |
| `d+` / `d-` | bigger (`d+`) or smaller (`d-`) numbers move the arm **down** |
| `h` | save: the bucket is **level** now |
| `s` | save: the bucket is in the **scoop** position now |
| `o` / `r` | the IR is above **orzo** / **rocks**: record one reading |
| `c` | clear the IR readings |

### Quick version (no servo tuning, about 3 minutes)
1. **Arm direction check:** type `a 75`. Does the arm go **up**?
   - Yes → keep the servo angles in the code
   - No (it goes down) → change the code to `ARM_UP = 105;` and `ARM_DOWN = 78;`

   Then type `a 90`
2. **IR:** do part C below

### Full version (servo angles measured, about 10 minutes)
**A. Arm** — OTV at the front, chassis side **10.5 cm** from the site edge, bucket above the near-left plot
1. Lower the arm step by step (`a 85`, `a 80`...). Near the surface use 1–2° steps
2. When the bucket **just touches** the orzo, type `t`
3. Type `d+` or `d-`
4. The code works out `ARM_DOWN` (12° lower, digs about 2 cm) and `ARM_UP` (15° higher)

**B. Bucket** — keep the arm at the touch angle
1. Put a bean in. Turn the bucket (`b 85`, `b 95`...) until it is level and the bean stays in. Type `h`
2. Try about 60° each way from there. Where it **scoops and the bean falls out**, type `s`
3. The code works out `BUCKET_DUMP`

### C. IR (needed for Mission I)
1. Push the OTV to the **front** of the site, chassis side **9.3 cm** from the site edge. **Push from the back or sides**
2. IR above the **near-left plot (orzo)**: hold still, type `o`. Do this at **3 spots**
3. IR above the **near-right plot (rocks)**: type `r` at **5 or more spots**, especially light-colored rocks
4. It always prints a threshold. If it adds a **Note** (gap under 50, or overlap), the result is still usable. If you have time, black tape around the sensor makes it more reliable: then type `c` and measure again

### D. Copy the result
Copy the lines from the **last** printout and **replace** the same lines at the top of the code (don't add them a second time). Example:
```
const int IR_THRESHOLD = 515;
const bool ORZO_IS_HIGHER = true;
```
Lines it can't work out yet show a hint in brackets instead. For those, keep what is in the code.

**Safety:** if a servo **buzzes, shakes, or gets warm**, it is pushing on something. Move it back at once.

---

## 5. Step 2 — Mission I (MODE 2)

1. Set `MODE = 2`, upload, unplug USB, power OFF
2. Open the vision system page (1201), choose **The Harvesters**
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
   far-left: rocks (IR 310)
   far-right: orzo (IR 612)
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
| The arm goes **up** when it should dig | `ARM_UP = 105;` and `ARM_DOWN = 78;` |
| The bucket turns the **wrong way** | swap the numbers of `BUCKET_SCOOP` and `BUCKET_DUMP` |
| Bucket stops above the orzo, or a servo buzzes against the plot | The 90° guess is wrong: do the full version of MODE 0 |
| Bean not deep enough | Move `ARM_DOWN` 1–2° further down. The wrist servo must not go into the orzo |
| Bean not covered | Change `BUCKET_DUMP` by 5–10° |
| Rocks fall out when lifting | Turn `BUCKET_SCOOP` 5–10° further in the scooping direction |
| Stops after the first step and never moves | Not connected, or the camera can't see marker 205. Run `vision_test` (**Vision System Test.zip**) to find out which |
| OTV goes **forward** instead of back | In `driveBackward()`, swap `HIGH` and `LOW` on IN1–IN4 |
| OTV **spins** instead of backing up | One side is wired the other way: swap `HIGH`/`LOW` for that side only (IN1/IN2 = right, IN3/IN4 = left) |
| Keeps backing up for 15 s | The camera lost the marker while moving |
| Mission I: one plot is wrong | The page shows that plot's IR number: compare it with `IR_THRESHOLD`, redo part C |
| Mission I: left and right are swapped | The OTV faced the wrong way at the back. The IR must start above the far plot toward the OTV's **front** |

---

## 8. How the code works (for instructor questions)

- **Mission II / III need no sensor:** the near-left plot is always orzo and the near-right is always rocks (official rule).
- **IR:** orzo and rocks reflect IR differently. One reading = the average of 10 quick samples at one spot. A threshold from calibration (halfway between the closest orzo and rock readings) decides orzo or rocks.
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
