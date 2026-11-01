# HC-SR04 Ultrasonic Sensor Calibration

Arduino sketch that calibrates the HC-SR04 ultrasonic sensor our OTV uses for obstacle avoidance.

Written for **ENES100 Milestone 5, sub-task 8 (Obstacle sensing):** *"Your team must demonstrate that the sensors used for obstacle avoidance have been calibrated."*

## Why calibrate

The HC-SR04 doesn't measure distance directly. It measures **time**: how long a sound pulse takes to reach an obstacle and echo back. The code converts that time to centimeters using the speed of sound, and the conversion has two small errors:

| Error | Effect | Main cause |
|---|---|---|
| **SCALE** | Off by the same **percentage** at every distance | The speed of sound changes with air temperature (about 0.6 m/s per °C) |
| **OFFSET** | Off by the same **amount** at every distance | The sound doesn't start exactly at the front of the sensor, and the electronics add a small fixed delay |

Together they form a straight line:

```
true distance = SCALE × sensor reading + OFFSET
```

The sketch compares the sensor with a ruler at 5 known distances, finds the best-fit SCALE and OFFSET (least squares), and then lets you check the result at new distances.

Our navigation code treats an obstacle closer than **15 cm** as blocked (`BLOCKED_MARGIN`), so the goal is to show that the sensor's error is much smaller than that.

## What you need

- Arduino UNO Rev3 and the HC-SR04, ideally mounted on the OTV at its real height
- A ruler (30 cm is enough) and a flat box or obstacle block

| HC-SR04 | Arduino |
|---|---|
| VCC | 5V |
| GND | GND |
| TRIG | D2 (`TRIG_PIN`) |
| ECHO | D4 (`ECHO_PIN`) |

> D2 and D4 are placeholders. Change `TRIG_PIN` and `ECHO_PIN` at the top of the sketch to match the OTV's pin map.

## How to use

1. Upload `Ultrasonic_Calibration.ino` and open the Serial Monitor at **9600 baud**. Any line ending works ("No line ending" just adds about a 1 s wait).
2. **Calibrate (5 points).** For each prompt, put the box square to the sensor, measure from the front of the sensor (the two silver cans) to the box, type the distance in cm (e.g. `15` or `15.5`), and press Enter. Each point takes about a second.
   Use 5 distances with at least **20 cm** between the nearest and the farthest, e.g. **10, 15, 20, 25, 30 cm**. Order and spacing don't matter.
3. **Read the results.** The sketch prints a table (ruler / sensor / calibrated), the calibration line, and the largest error before and after calibration.
4. **Check at new distances.** Put the box at distances you did *not* use, inside the calibrated range (e.g. 12, 18, 27 cm). Type each one and press Enter. Each check prints the error before and after calibration. Repeat as often as you like.
5. Press **RESET** on the Arduino to start over.

> Closing and reopening the Serial Monitor resets the Uno and clears the results, so copy or screenshot them first.

## Example output

From a simulated sensor. A real HC-SR04 usually keeps a few tenths of a cm of error after calibration.

```
tape    sensor  calibrated   (cm)
10.0    10.9    10.0
15.0    15.9    15.0
20.0    20.7    19.9
25.0    25.8    25.0
30.0    30.8    30.0
Calibration: true = 1.0070 x sensor - 0.98 cm
Largest error: before 0.9 cm, after 0.1 cm

Now check it at distances you did NOT use above.
Check: put the box at a new distance, type it in cm, press Enter:
   tape 12.0 cm:  sensor 12.8 (error 0.8)  ->  calibrated 11.9 (error 0.1)
Check: put the box at a new distance, type it in cm, press Enter:
   tape 27.0 cm:  sensor 27.7 (error 0.7)  ->  calibrated 26.9 (error 0.1)
```

## How the code works

- **`pingCm()`**: one measurement. A 10 µs pulse on TRIG makes the sensor send a burst of 40 kHz sound. ECHO stays HIGH until the echo returns, and `pulseIn()` measures that time. Distance = `µs × 0.0343 / 2`: 343 m/s is 0.0343 cm/µs, and ÷2 because the sound travels there and back. It gives up after 30 ms (about 5 m) so it never hangs.
- **`readCm(20)`**: takes 20 measurements 60 ms apart and returns the **median**. A stray echo can make one reading far off. That would pull an average away, but the median ignores it. The 60 ms gap lets the previous echo die out.
- **`setup()`**, steps 1–3: collects the 5 points, checks that they span at least 20 cm, fits the line with least squares (x = sensor, y = ruler, `mx` and `my` = their averages), and prints the results:
  - `SCALE = sxy / sxx`, where `sxy = Σ (x − mx)(y − my)` and `sxx = Σ (x − mx)²`
  - `OFFSET = my − SCALE × mx`
- **`loop()`**, step 4: checks the calibration at new distances. The table's errors come from the same points used for the fit, so they always look good. A distance that wasn't used is a fair test.

## Messages

| Message | Meaning | What to do |
|---|---|---|
| `Type a number between 2 and 400.` | Not a number, or outside the sensor's 2–400 cm range | Type the distance again |
| `No echo - is the box square to the sensor?` | Most of the 20 pings got no echo | Square the box to the sensor; check the wiring and pin numbers |
| `Distances too close together ...` | Nearest and farthest points are less than 20 cm apart | Press RESET and spread the points out |
| `A point is off by more than 2 cm ...` | One point doesn't fit the line (box moved, or a typo) | Press RESET and redo |

## MS5 demo (sub-task 8)

1. Calibrate live at 10–30 cm (about 2 minutes), explaining that the ruler is the reference.
2. Point out the **sensor** and **calibrated** columns and the `Largest error: before … after …` line.
3. Have the instructor pick new distances inside the calibrated range. Each check line shows the error before and after calibration.
4. Conclusion: the remaining error is far smaller than the 15 cm at which our navigation treats an obstacle as blocked, so the sensor is accurate enough for obstacle avoidance.

## Notes

- The line is only reliable inside the range you calibrated, so do the checks inside that range.
- Results are kept in RAM only and are not sent to the navigation code. To use them on the OTV, apply `distance = SCALE × reading + OFFSET` in `sonar.cpp`.
- Calibrate with the sensor at its real position on the OTV (MS5 requires the tested sub-system to match the design). Recalibrate if the sensor is replaced or moved.
