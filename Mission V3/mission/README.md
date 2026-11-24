# Mission Code V3 — The Harvesters (Seed Planting)

Mission code only (no navigation). It is used to set up and test the three mission objectives before MS7.

**What's new in V3**
- **MODE 0 (calibration) is back.** After the power wiring change (section 2), the USB and the battery can be on at the same time, so the servo angles can be set by typing commands.
- Marker **147**, room **1201**
- IR cutoff **21.5** from our calibration (orzo about 19–21, rocks about 23–25). MODE 0 can re-check it.
- With USB in, MODE 1 and 2 also print their progress in the Serial Monitor.

**Board:** Arduino UNO R4 WiFi

---

## 1. Overview

Choose what the code does with `MODE` at the top of `mission.ino`:

| `MODE` | Where the OTV starts | What it does |
|---|---|---|
| 0 | Front of the site | Calibration: servo angles, and an optional IR re-check |
| 1 | Front of the site | Plants the seed → backs up one plot → scoops rocks (Mission II + III) |
| 2 | Back of the site | Reads the two far plots → sends the result (Mission I) |

**Order:**
1. Change the power wiring (section 2), once
2. MODE 0: set the servo angles and measure the arm (section 5)
3. MODE 1 in the air: dry run (section 6)
4. MODE 2: Mission I (section 7)
5. MODE 1: Mission II + III (section 8)

---

## 2. Power wiring change (do this first, once)

**Goal:** the USB and the battery can both be on.

| Part | Gets power from |
|---|---|
| Arduino | **Battery 12 V → VIN pin** (after the power switch). The R4 WiFi takes 6–24 V on VIN |
| Servos, IR sensor | **Buck converter 5 V** |
| All GND | Connected together (battery, buck converter, Arduino, L298N, servos, sensors) |

**The rule:** nothing from the buck converter's 5 V may connect to the **Arduino 5V pin**. Also check the L298N's own **5V terminal** is not wired to the Arduino 5V pin.

**Check before turning on:**
- Arduino **5V pin**: nothing from the buck converter or the L298N on it
- Arduino **VIN pin**: the battery's 12 V, after the switch
- GND: everything connected

**Until this change is done, never have the USB in and the power switch on at the same time.**

---

## 3. Before you start

**Bring:** charged battery, lima beans, ruler, thin stick (to check the seed depth)

**Wiring:**

| Part | Pin |
|---|---|
| Upper servo (lifts the arm, closer to the OTV) | A1 |
| Lower servo (turns the bucket) | A2 |
| IR sensor (TCRT5000, AO) | A0 |
| Right motors: ENA / IN1 / IN2 | 10 / 9 / 8 |
| Left motors: ENB / IN3 / IN4 | 5 / 7 / 6 |

**Laptop:** ENES100 library and ArduinoHttpClient library installed. Tools → Board → **Arduino UNO R4 WiFi**, and the right Port.

**Words used below:**
- **Right side** = stand behind the OTV with its front (ultrasonic sensor) pointing away from you. The arm is on your right
- **Near / far, left / right plots** = as seen from the **front** of the site (from the middle of the arena). Near-left is always orzo, near-right is always rocks

---

## 4. How to upload

1. Power switch **OFF**
2. Change `MODE` at the top of `mission.ino`
3. Plug in USB → click **Upload**
4. Place the OTV → power switch **ON** → the program starts

After the wiring change, the USB can stay in: open the Serial Monitor (**9600**) to see what the OTV is doing. Unplug it for real runs, so the cable doesn't pull the OTV.

To run the same program again: press **RESET**. No need to upload again.

---

## 5. Step 1 — Calibrate (MODE 0)

`MODE = 0`, upload. **USB in, power ON.** Serial Monitor: **9600** and **Newline**.
At power-on both servos go to 90 (**hold the arm**, this first move jumps). Every second it prints:
`IR: 20.4   arm: 90   bucket: 90`

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

**If a servo buzzes, shakes or gets warm, it is pushing on something: move it back at once.**

### A. Arm
OTV at the **front** of the site, right side facing it, bucket above the **near-left plot (orzo)**.
1. Lower the arm step by step (`a 85`, `a 80`...). Near the surface use 1–2° steps
2. When the bucket **just touches** the orzo, type `t`
3. Type `d+` or `d-`
4. The code works out `ARM_DOWN` (12° lower, digs about 2 cm) and `ARM_UP` (15° higher)

### B. Bucket
Keep the arm at the touch angle.
1. Put a bean in. Turn the bucket (`b 85`, `b 95`...) until it is level and the bean stays in. Type `h`
2. Try about 60° each way from there. Where it **scoops and the bean falls out**, type `s`
3. The code works out `BUCKET_DUMP`

### C. IR re-check (optional)
The code already has our cutoff (21.5). Re-check only if you want to confirm it at the real height:
1. IR above the near-left plot (orzo): type `o` at 3 spots
2. IR above the near-right plot (rocks): type `r` at 5 or more spots
3. It prints `IR_THRESHOLD` and `ORZO_IS_HIGHER`

### D. Measure the arm and the IR (for the parking distance)
With the arm at the touch angle, measure from the **right side of the chassis**:
- to the **center of the bucket**: front parking distance = this − **4.2 cm**
- to the **center of the IR sensor**: back parking distance = this − **4.2 cm**

(4.2 cm = half a plot.) Our CAD gives 10.5 cm (front) and 9.3 cm (back). Use your measured numbers in sections 7 and 8 if they differ.

### E. Copy the result
Copy the lines from the **last** printout and **replace** the same lines at the top of the code. Lines it can't work out yet show a hint in brackets: for those, keep what is in the code.

---

## 6. Step 2 — Dry run (MODE 1, in the air)

1. `MODE = 1`, upload
2. Hold the OTV in the air **away from the site**, wheels off the ground. Power ON
3. Watch: right away the arm goes **up** → after 3 s: arm **down** → bucket scoops → bucket pours → arm up → bucket level
4. If the vision system isn't connected, it stops after the seed step (waits to back up). That's normal here: power OFF

---

## 7. Step 3 — Mission I (MODE 2)

1. `MODE = 2`, upload
2. Open **enes100.umd.edu → Student Tools → Vision System → 1201**, choose **The Harvesters**
3. Place the OTV at the **back** of the site (between the site and the wall):
   - right side facing the site, parallel to the edge
   - chassis side **9.3 cm** (or your measured number) from the site edge, at both ends
   - **IR above the middle of the far plot toward the OTV's front**
4. Power ON. Check the page: **"The Harvesters has connected"** and **Visible = YES**
5. After 3 s it reads the first plot (**far-right**) → backs up 8.4 cm (**toward the OTV's rear**) → reads **far-left** → sends the result
6. The page shows:
   ```
   MISSION MESSAGE: plot locations = DIAGONAL
   Seen from the front of the site:
   far-left: rocks (IR 24.1)
   far-right: orzo (IR 20.0)
   ```
   Check that far-left and far-right match the real plots

```
BACK SIDE (top view)

   back <-[============ OTV ============]-> front
                (right side faces the site)
                             IR starts here v
        [ far-left  ] [ far-right  ]
        [ near-left ] [ near-right ]
```

**How it decides:** below 21.5 = orzo, above = rocks. Each reading averages 50 samples. If a reading is between 21.0 and 22.0, it scans again (up to 4 times) and uses the average.

**The four results:** BOTH = both far plots orzo · NEITHER = both rocks · ADJACENT = only far-left orzo · DIAGONAL = only far-right orzo

---

## 8. Step 4 — Mission II + III (MODE 1)

1. `MODE = 1`, upload
2. Open the vision system page (1201), choose **The Harvesters**
3. Put the **lima bean** in the bucket
4. Place the OTV at the **front** of the site:
   - right side facing the site, **front of the OTV toward the orzo side**
   - chassis side **10.5 cm** (or your measured number) from the site edge, at both ends
   - **bucket above the middle of the near-left plot (orzo)**
5. Power ON. Check the page: **connected** and **Visible = YES**
6. After 3 s it plants the seed → backs up 8.4 cm → scoops rocks
7. Check: **top of the bean at least 1 cm deep** (thin stick), **rocks at least 10 g**

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

## 9. If something goes wrong

**Something goes wrong mid-run: power OFF right away.**

| Problem | Fix |
|---|---|
| Bean not deep enough | Move `ARM_DOWN` 1–2° further down. The wrist servo must not go into the orzo |
| Bean not covered | Change `BUCKET_DUMP` by 5–10° |
| Rocks fall out when lifting | Turn `BUCKET_SCOOP` 5–10° further in the scooping direction |
| Serial Monitor stops at "Waiting for position..." | Not connected, or the camera can't see marker 147. Run `vision_test` (Vision System Test V2.zip) |
| OTV goes **forward** instead of back | In `driveBackward()`, swap `HIGH` and `LOW` on IN1–IN4 |
| OTV **spins** instead of backing up | One side is wired the other way: swap `HIGH`/`LOW` for that side only (IN1/IN2 = right, IN3/IN4 = left) |
| "Moved" is much less than 8.4 cm | The camera lost the marker while moving (15 s limit) |
| Mission I: one plot is wrong | Compare its IR number with 21.5. Readings far from 19–25: redo MODE 0 part C |
| Mission I: left and right swapped | The OTV faced the wrong way. The IR must start above the far plot toward the OTV's **front** |

---

## 10. How the code works (for instructor questions)

- **Mission II / III need no sensor:** near-left is always orzo and near-right is always rocks (official rule).
- **IR:** orzo and rocks reflect IR differently. Orzo reads about 19–21 and rocks about 23–25, so the cutoff is 21.5. The difference is small, so each reading averages 50 samples, and a reading close to the cutoff is scanned again.
- **Why read from the back:** the IR only reaches the row closest to the OTV. From the back, the far row is the closest.
- **Left / right flip:** from the back, the far plot toward the OTV's front is far-RIGHT. The code reads it first.
- **Moving one plot:** the vision system only gives position (x, y), so the code saves the start position and works out the distance after each short full-power pulse. It stops at 8.4 cm. Position updates every 250 ms, so it waits 350 ms between pulses. A position of −1 (marker not seen) is ignored.
- **Arm angles:** digging 12° past "just touching" goes about 2 cm deep (CAD simulation); more would push the wrist servo into the orzo. Lifting 15° keeps the bucket about 3 cm above the plots and under the 18.4 cm limbo.
- **Servos move 1° every 15 ms** so the bucket doesn't throw the bean or the substrate.
