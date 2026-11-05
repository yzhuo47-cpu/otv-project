# Sub-task 3: Turning (timed version, no Vision System)

The OTV turns 90° left in place by running the left side backward and the right side forward for a fixed time (`TURN_TIME`). It does this 3 times in a row. No Wi-Fi module, ArUco marker, or ENES100 library is needed, so it can be tested anywhere.

## Change before uploading

| Line | Parameter | Meaning |
|---|---|---|
| 2–5 | `LEFT_A` `LEFT_B` `RIGHT_A` `RIGHT_B` | L298N input pins. Must be PWM pins (UNO: 3, 5, 6, 9, 10, 11) |
| 8 | `TURN_TIME` | Milliseconds for a 90° turn. 5000 is only a starting guess |

**Wiring assumption:** on each L298N, IN1 and IN3 share one Arduino pin, IN2 and IN4 share another pin, and the ENA/ENB jumpers stay on.

## How to calibrate `TURN_TIME`

1. Put a strip of tape on the floor along the front edge of the OTV.
2. Upload, set the OTV down within 3 seconds, and watch the first turn.
3. Estimate the angle it actually turned, then update: **new TURN_TIME = old TURN_TIME × 90 ÷ measured angle**.
   For example, if 5000 ms gives 75°: 5000 × 90 ÷ 75 = 6000 ms.
4. Repeat until all 3 turns are close to 90°.

**If it turns right (clockwise) instead of left:** swap the numbers on lines 2 and 3, and on lines 4 and 5.

## Things that change the turn angle

- **Battery charge:** a lower battery makes the motors slower. Calibrate and demo with a fully charged battery.
- **Floor:** carpet, tile, and the arena floor have different friction. Calibrate on the arena floor if possible, or on a similar hard floor.
- **Weight:** calibrate with the arm and all parts installed, since the demo OTV must match the final design.

## Possible instructor questions

- **How does it know it turned 90°?** It doesn't measure the angle. The turn time was calibrated by testing, so running the motors for that time gives about 90°.
- **How does it turn without a steering mechanism?** Skid steer: the left wheels go backward and the right wheels go forward, so the OTV spins in place.
- **Why full speed?** The 10 RPM worm gear motors are already slow, and skid steer turning needs more torque than driving straight. A lower PWM would reduce torque.
- **What are the limits?** There is no feedback, so the angle can drift with battery charge and floor friction. The final navigation code will use the Vision System's heading instead.
