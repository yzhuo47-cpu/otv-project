# Sub-task 2: Forward Locomotion

The OTV waits 3 seconds, drives straight forward for a fixed time (`DRIVE_TIME`), then stops. No Wi-Fi module, ArUco marker, or ENES100 library is needed.

**Requirement:** from the landing zone, reach a point right before the limbo and log (3.4 m away) in under 3 minutes. Obstacles may be removed, and the OTV may be caught at the end.

**Expected time:** at the calculated 4.71 cm/s, 340 cm ÷ 4.71 cm/s ≈ 72 s. The L298N drops about 2 V, so the real speed will be a bit lower, but it should still be well under 180 s.

## Wiring

Same wiring as Sub-task 3. One L298N drives all four motors, with the two motors on each side wired in parallel.

| Ribbon wire | L298N pin | Arduino pin | What it does |
|---|---|---|---|
| Yellow | ENA | D10 (PWM) | Right side speed |
| Green | IN1 | D9 | Right side direction |
| Blue | IN2 | D8 | Right side direction |
| Purple | IN3 | D7 | Left side direction |
| Gray | IN4 | D6 | Left side direction |
| White | ENB | D5 (PWM) | Left side speed |

## Settings

| Line | Parameter | Meaning |
|---|---|---|
| 6–11 | Pin numbers | Already set to the current wiring. Only change if the wiring changes |
| 13 | `LEFT_SPEED` | Left side PWM (0–255) |
| 14 | `RIGHT_SPEED` | Right side PWM (0–255) |
| 15 | `DRIVE_TIME` | Milliseconds to drive. 75000 is only a starting guess |

## Before the demo

**Step 1 — both sides drive forward.**
Upload and watch the wheels.
- Left side goes backward → swap the numbers on lines 9 and 10.
- Right side goes backward → swap the numbers on lines 7 and 8.

**Step 2 — drive straight.**
Skid steer vehicles often drift because the two sides are never exactly equal.
- Curves **right** → the left side is faster. Lower `LEFT_SPEED` a little (e.g. 255 → 240).
- Curves **left** → the right side is faster. Lower `RIGHT_SPEED` a little.
- Change in small steps. Keep the slower side at 255 so it keeps full torque.

**Step 3 — stop right before the limbo.**
Run once and measure how far the OTV went (in cm). Then update:
**new DRIVE_TIME = old DRIVE_TIME × 340 ÷ measured distance**.
For example, if 75000 ms covers 300 cm: 75000 × 340 ÷ 300 = 85000 ms.
Keep `DRIVE_TIME` under 180000 (3 minutes).

## Tips

- Use a fully charged battery. A lower battery makes the motors slower.
- Install the arm and all parts. The demo OTV must have about the same mass as the final design.
- Aim the OTV straight down the arena by hand before turning it on.
- Test on the arena floor if possible. Carpet and tile have different friction.

## Possible instructor questions

- **How does it know when to stop?** It drives for a fixed time that was calibrated by testing. This sub-task doesn't require programming or sensing.
- **Will it make it in 3 minutes?** Calculated speed is 4.71 cm/s, so 3.4 m takes about 72 s, well under 180 s even with some speed loss.
- **Why is `DRIVE_TIME` an `unsigned long`?** On the Arduino UNO, `int` only goes up to 32767, so it can't hold times like 75000 ms.
- **How do you keep it straight?** Each side has its own PWM value. If one side is faster, its PWM is lowered slightly to match the other side.
- **What do ENA/ENB and IN1–IN4 do?** IN pins set the direction (one HIGH, one LOW). ENA/ENB receive a PWM signal that sets the speed.
