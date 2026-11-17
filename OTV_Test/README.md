# OTV_Test

Quick hardware test for the OTV. After power-on or reset it runs once:

1. Waits 3 s (time to put the OTV on the floor)
2. **Forward** → **Backward** → **Turn left** → **Turn right** (1.5 s each, 1 s pause between)
3. **Arm servo** (A1): 90° → 60° → 120° → 90°
4. **Bucket servo** (A2): 90° → 60° → 120° → 90°

Press the Arduino **RESET** button to run it again. No ENES100 library or Eagle Eye needed.

## Pins

| Part | Pin |
|---|---|
| Right side (L298N ENA / IN1 / IN2) | 10 / 9 / 8 |
| Left side (L298N ENB / IN3 / IN4) | 5 / 7 / 6 |
| Arm servo (upper) | A1 |
| Bucket servo (lower) | A2 |

## How to run

1. Open `OTV_Test.ino` in the Arduino IDE (the folder must stay named `OTV_Test`).
2. Upload it.
3. Unplug USB, power the OTV from the battery, put it on the floor. It starts after 3 s.
4. For servo-only testing on the table, set `TEST_DRIVE = false` (wheels won't spin).

## Settings (top of the file)

| Setting | What it does |
|---|---|
| `TEST_DRIVE`, `TEST_SERVOS` | Turn each part of the test on/off |
| `SPEED` | Motor speed, 0–255 |
| `MOVE_TIME` | How long each drive step lasts (ms) |
| `ARM_MIN/MAX`, `BUCKET_MIN/MAX` | Servo test range in degrees. Start small, widen once you know the real limits |

## What to check

- **Forward goes backward, or the OTV spins during "Forward"**: one side is wired the other way. Swap that side's two IN pin numbers in the code (e.g. `IN1 = 8; IN2 = 9;`), or swap its motor wires at the L298N.
- **One side doesn't move at all**: check that the ENA/ENB jumper caps on the L298N are removed and the 12V supply is on.
- **Servo shakes or Arduino resets when servos move**: servos are drawing too much current; power them from the 5V buck converter, not the Arduino 5V pin.

## Note: Servo library and pin 10

The Servo library takes over Timer1, which breaks `analogWrite` speed control on pins 9 and 10. ENA is on pin 10, so in this test the servos are attached only after driving is done. In the main code, if servos and motors are used together, the right side only works correctly at speed 0 or 255 (or move ENA to a free PWM pin, like pin 3).
