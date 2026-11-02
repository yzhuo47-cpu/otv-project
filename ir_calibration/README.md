# IR Calibration (MS5 sub-task 9: mission sensing)

Calibrates the TCRT5000 IR sensor that tells **orzo (plantable)** from **rocks (non-plantable)**.

## How it works
- The sensor shines IR light down and measures how much reflects back. A0 reads it as 0-1023.
- Orzo and rocks reflect different amounts, but the exact numbers depend on the sensor, its height
  and the room light, so we measure them. That is the calibration.
- Each material is measured at 8 different spots. Each reading is the average of 25 reads.
- The average of each material is its **reference value**. A new reading is classified as whichever
  reference it is closer to, so the dividing value is halfway between the two averages.

## Setup
- Upload `ir_calibration.ino` (UNO Rev3 or UNO R4 WiFi).
- Open the Serial Monitor at **115200** baud (any line ending).
- Fix the sensor at the **IR-arm height**, pointing straight down. Height matters most: the sensor
  works best within about 1.5 cm.

## Steps
1. *(Optional)* Type `v` to see raw readings. Move between orzo and rocks and check the numbers
   change clearly. If not, lower the sensor. Any key + Enter stops.
2. Put **orzo** under the sensor and type `p`. It takes 8 readings with a countdown before each:

   | Countdown | What to do |
   |---|---|
   | `5.. 4.. 3.. 2..` | Slide the container to a new spot |
   | `1..` | Hands off, hold still |
   | number appears | Reading done |

3. Put **rocks** under the sensor and type `n`. Same countdown.
4. The result prints by itself.

Typing `p` or `n` again re-measures only that material; the other one is kept.

## Commands
| Type | What happens |
|---|---|
| `p` | Measure orzo (8 spots) |
| `n` | Measure rocks (8 spots) |
| `t` | Test once: one reading, one verdict, numbered (Test 1, Test 2, ...). Place the container first. |
| `v` | Live readings, 2-3 per second. Shows the verdict once calibrated. Any key + Enter stops. |

## Reading the result
Example (your numbers will differ; orzo may read lower than rocks, which is fine):
```
=== RESULT (0-1023) ===
	average	min	max
orzo	623	613	633
rocks	447	421	476
Difference of averages: 176   (need at least 40)
Readings classified correctly: 16/16
Dividing value: 535.0  (above = orzo, below = rocks)

-> CALIBRATED. Type t to test once, v for live readings.
```

| Line | Meaning |
|---|---|
| average | The reference value of each material |
| min / max | Range of the 8 readings |
| Difference of averages | Safety margin. Must be at least 40 so new readings are not misclassified. |
| Readings classified correctly | All 16 calibration readings must be correct with the rule |
| Dividing value | Halfway between the two averages |

**CALIBRATED** needs both 16/16 and a difference of at least 40.
Until then, `t` refuses to test and `v` shows raw numbers only.

## Demo
1. `v` before calibrating: show that the raw numbers change between orzo and rocks.
2. `p` then `n`: calibrate live, then explain the result table.
3. `t` blind test: the instructor places either container, then type `t`. Repeat 5-6 times.
4. Optional: shine a phone flashlight near the sensor and test again.

Before the demo, use `t` on different spots of the rocks (especially light-colored ones) to make
sure they all classify correctly.

## Troubleshooting
| Problem | Fix |
|---|---|
| NOT RELIABLE | Lower the sensor, shade it from room light, check it points straight down, then `p` and `n` again |
| One reading looks wrong | Usually the container moved during the reading. Re-measure that material (`p` or `n`). |
| Nothing in the Serial Monitor | Check 115200 baud, then press the board's reset button |
| Countdown too short | Increase `MOVE_SECONDS` at the top of the code |
