# Mission Code for MS6 — Seed Planting (The Harvesters)

This code runs the **mission part only** (no navigation).
MS6 lets us demonstrate each objective separately, so we place the OTV next to the mission site by hand and let it do the mission.

> Ask the instructor first: is it OK to place the OTV next to the site by hand for the MS6 demo?

---

## 1. What the code does

You choose what to run with `MODE` at the top of `mission_ms6.ino`.

| MODE | Where | What happens | MS6 objective |
|---|---|---|---|
| 0 | Anywhere | Prints IR numbers to the Serial Monitor. Motors don't move. | IR calibration |
| 1 | Front of the site | Plants the seed → backs up 8.4 cm → scoops rocks | Mission II + III |
| 2 | Back of the site | Reads far plot → backs up 8.4 cm → reads the other far plot → sends the result | Mission I |

**Who moves the OTV?**
- **To the starting spot: you, by hand.** This code has no navigation.
- **To the next plot (8.4 cm): the OTV, by itself.** The overhead camera measures the distance.
- **From the front to the back: you, by hand.** MODE 1 and MODE 2 are separate runs.

---

## 2. Wiring

| Part | Arduino pin |
|---|---|
| Upper servo (lifts the arm, closer to the OTV) | A1 |
| Lower servo (turns the bucket) | A2 |
| IR sensor TCRT5000 (AO) | A0 |
| Wi-Fi module TX / RX | 3 / 2 |
| Right motors: ENA / IN1 / IN2 | 10 / 9 / 8 |
| Left motors: ENB / IN3 / IN4 | 5 / 7 / 6 |

Servo power: red wire to 5 V from the buck converter (not the Arduino 5 V pin). All GND wires connected together.

---

## 3. The upload routine (every time you change MODE)

1. Turn the OTV power switch **OFF**
2. Change `MODE` in the code
3. Plug in the USB cable → click **Upload**
4. Unplug the USB cable (for MODE 0, keep it plugged in to see the Serial Monitor)
5. Place the OTV at the starting spot
6. Turn the power switch **ON** → the code starts

To run the same MODE again: press the **reset button** on the Arduino (or turn the power off and on).

---

## 4. Step by step

### Step 1 — Set up the laptop (once)
1. ENES100 library installed (Deem's laptop: done ✓)
2. Put `mission_ms6.ino` and this README in a folder named **`mission_ms6`** (the Arduino IDE needs this name)
3. Open `mission_ms6.ino` in the Arduino IDE
4. Tools → Board → **Arduino Uno**, and choose the right Port
5. Click **Verify (✓)**. It should compile with no errors

### Step 2 — Fill in the settings (once)
At the top of the code, change everything marked `TODO`:
- `MARKER_ID`: the number of our ArUco marker
- `ROOM`: the room of the demo (1116, 1120 or 1201)
- Check that the Wi-Fi module TX wire is on pin 3 and RX on pin 2

### Step 3 — Check the servo start position (MODE 0)
1. Set `MODE = 0`, upload, keep USB plugged in, power switch ON
2. The servos move to `ARM_UP` and `BUCKET_HOLD`
3. Check: arm is up, bucket is level, nothing hits the OTV
4. If not, change those two numbers and upload again

> First time: hold the arm with your hand, because the angles are only guesses.

### Step 4 — Calibrate the IR (MODE 0, at the mission site)
1. Still MODE 0. Keep USB plugged in. Open **Tools → Serial Monitor**, set **9600 baud**
2. Push the OTV by hand to the **front** of the site, right side facing the site. From the front, the IR is above the near plots
3. IR above the **near-left plot (always orzo)** → write down the numbers
4. IR above the **near-right plot (always rocks)** → write down the numbers. Try a few spots
5. Fill in the code:
   - `IR_THRESHOLD` = halfway between the orzo number and the rock number
   - `ORZO_IS_HIGHER` = `true` if orzo gives the bigger number, otherwise `false`

> The two numbers need a clear gap. If they are close, block outside light with black tape or lower the sensor a little.

### Step 5 — Demo Mission II + III (MODE 1)
1. Set `MODE = 1`, upload, unplug USB
2. Put the **lima bean** in the bucket
3. Place the OTV at the **front** of the site:
   - right side of the chassis about **10.5 cm** from the site edge
   - OTV parallel to the site edge
   - **bucket above the near-left plot**
4. Power ON. The OTV connects to the vision system, then:
   plants the seed → backs up 8.4 cm → scoops rocks → lifts the arm
5. Check:
   - **Seed:** push a thin stick down until it touches the bean. The **top of the bean must be at least 1 cm below the surface**
   - **Rocks:** at least **10 g** (aim for about 20 g)

```
FRONT SIDE (top view)

        [ far-left  ] [ far-right  ]
        [ near-left ] [ near-right ]
        [   ORZO    ] [   ROCKS    ]
             ^ bucket starts here
   front <-[============ OTV ============]-> back
                (right side faces the site)
```

### Step 6 — Demo Mission I (MODE 2)
1. Set `MODE = 2`, upload, unplug USB
2. Place the OTV at the **back** of the site (between the site and the wall):
   - right side of the chassis about **9.3 cm** from the site edge
   - OTV parallel to the site edge
   - **IR above the far plot toward the OTV's front**
3. Power ON. The OTV reads the first plot → backs up 8.4 cm → reads the second plot → sends the result
4. Check that the vision system screen shows the right answer: **BOTH, NEITHER, ADJACENT or DIAGONAL**

```
BACK SIDE (top view)

   back <-[============ OTV ============]-> front
                (right side faces the site)
                             IR starts here v
        [ far-left  ] [ far-right  ]
        [ near-left ] [ near-right ]
```

What the result means (seen from the front, the seed plot is near-left):

| Result | Far-left | Far-right |
|---|---|---|
| BOTH | orzo | orzo |
| NEITHER | rocks | rocks |
| ADJACENT | orzo | rocks |
| DIAGONAL | rocks | orzo |

---

## 5. Tuning the servo angles

The angles in the code are **guesses**. Watch each run and change them:

| Setting | What it controls | Change it if... |
|---|---|---|
| `ARM_UP` | Arm position when driving | arm hits the OTV, or the OTV is taller than 18.4 cm (limbo) |
| `ARM_DOWN` | How deep the bucket goes | bean is not 1 cm deep, or bucket doesn't reach the rocks |
| `BUCKET_HOLD` | Bucket at the start | bean falls out before planting |
| `BUCKET_SCOOP` | Scooping | bucket doesn't scoop, or rocks fall out when lifting |
| `BUCKET_DUMP` | Pouring orzo back | bean is not covered |

---

## 6. Problems and fixes

| Problem | Fix |
|---|---|
| MODE 1/2: nothing happens after power ON | The code waits until it connects to the vision system. Check Wi-Fi wiring (TX→3, RX→2), `ROOM`, and that you are in the arena room. |
| OTV goes forward instead of backward | In `driveBackward()`, swap `HIGH` and `LOW` on IN1–IN4 |
| OTV keeps backing up for 15 s | Camera can't see the marker, or `MARKER_ID` is wrong |
| Mission I gives ADJACENT/DIAGONAL swapped | OTV was facing the wrong way at the back. IR must start above the far plot toward the OTV's **front** |
| Mission I result is wrong in other ways | Redo the IR calibration (Step 4) |
| Servos twitch a little during Wi-Fi | Normal. The Wi-Fi library briefly pauses the servo timing |

---

## 7. How the code works (for instructor questions)

- **IR identification:** orzo and rocks reflect IR differently. We average 10 readings and compare with one threshold from calibration.
- **Only the far plots need the IR:** the near-left plot is always orzo and the near-right is always rocks (official rule).
- **Why read from the back:** in our CAD, the IR arm only reaches the row closest to the OTV. From the back, the far row is the closest row.
- **Left/right flip:** from the back, the plot toward the OTV's front is far-RIGHT. The code reads it first.
- **Moving between plots:** the OTV backs up in short pulses and checks the camera position after each one, so it stops close to 8.4 cm.
- **Why `digitalWrite` for the motors:** the Servo library turns off PWM on pin 10 (our right motor speed), so `analogWrite(10)` would not work.

---

## 8. Later: full run with navigation

1. Navigation drives to the front of the site, bucket above the near-left plot → `frontSide()`
2. Drive around to the back of the site → `missionOne()`
3. Continue to the obstacles. **Keep the rocks in the bucket until the end.**

Notes:
- Turn to the right heading **before** entering the gap between the site and the wall. Don't spin there: the bucket reaches about 27 cm from the OTV's center and would hit the wall.
- Parking accuracy needed (from the CAD): about ±2.4 cm in position and ±3° in heading.
- **Pin 10 problem:** once the servos are attached, `analogWrite(10, ...)` in the navigation code stops working. Fix: move ENA to pin 11 and move the ultrasonic ECHO wire from pin 11 to a free pin.
