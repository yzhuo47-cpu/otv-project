# Sub-task 3: Turning (timed version, no Vision System)

The OTV turns 90° left in place by running the left side backward and the right side forward for a fixed time (`TURN_TIME`). It does this 3 times in a row. No Wi-Fi module, ArUco marker, or ENES100 library is needed, so it can be tested anywhere.

## Wiring

One L298N drives all four motors. The two motors on each side are wired in parallel to the same output:

| Side | L298N output | Control pins |
|---|---|---|
| Right (front + rear) | OUT1 / OUT2 | ENA, IN1, IN2 |
| Left (front + rear) | OUT3 / OUT4 | ENB, IN3, IN4 |

The ENA/ENB jumpers are removed, and the 6 header pins go to the Arduino:

| Ribbon wire | L298N pin | Arduino pin | What it does |
|---|---|---|---|
| Yellow | ENA | D10 (PWM) | Right side speed |
| Green | IN1 | D9 | Right side direction |
| Blue | IN2 | D8 | Right side direction |
| Purple | IN3 | D7 | Left side direction |
| Gray | IN4 | D6 | Left side direction |
| White | ENB | D5 (PWM) | Left side speed |

D2, D3, D4, D11, D12, D13, and A0–A5 are still free for the Wi-Fi module, servos, and sensors.

**Note for later (combining with the servos):** on the UNO, the Servo library turns off PWM on pins 9 and 10. Right side speed (ENA) is on D10, so once servos are added it will stop working. Before combining this code with Sub-task 10, move the yellow ENA wire to D3 or D11 and change line 6 to match.

## Change before uploading

| Line | Parameter | Meaning |
|---|---|---|
| 6–8 | `RIGHT_EN` `RIGHT_IN1` `RIGHT_IN2` | Arduino pins wired to ENA, IN1, IN2 (already set to the current wiring) |
| 9–11 | `LEFT_IN1` `LEFT_IN2` `LEFT_EN` | Arduino pins wired to IN3, IN4, ENB (already set to the current wiring) |
| 14 | `TURN_TIME` | Milliseconds for a 90° turn. 5000 is only a starting guess |

Only change lines 6–11 if the wiring changes. `RIGHT_EN` and `LEFT_EN` must be PWM pins (UNO: 3, 5, 6, 9, 10, 11).

## Check directions first

**Step 0 — both wheels on one side spin the same way.**
The front and rear motors on each side are mounted flipped, so the "+" tab is on the opposite end of each motor. Touch a 9V battery to each side's pair of wires: both wheels on that side must spin the same way. If not, swap the two wires of one motor.

**Step 1 — both sides drive forward.**
Temporarily change line 40 from `drive(-SPEED, SPEED);` to `drive(SPEED, SPEED);` and upload. The OTV should drive straight forward.
- Left side goes backward → swap the numbers on lines 9 and 10.
- Right side goes backward → swap the numbers on lines 7 and 8.

**Step 2 — turn direction.**
Change line 40 back to `drive(-SPEED, SPEED);`. The OTV should turn **counterclockwise (left)**.
- If it turns right instead, the left and right channels are swapped. Swap the numbers of the two groups: lines 6↔11, 7↔9, 8↔10.

## How to calibrate `TURN_TIME`

1. Put a strip of tape on the floor along the front edge of the OTV.
2. Upload, set the OTV down within 3 seconds, and watch the first turn.
3. Estimate the angle it actually turned, then update: **new TURN_TIME = old TURN_TIME × 90 ÷ measured angle**.
   For example, if 5000 ms gives 75°: 5000 × 90 ÷ 75 = 6000 ms.
4. Repeat until all 3 turns are close to 90°.

## Things that change the turn angle

- **Battery charge:** a lower battery makes the motors slower. Calibrate and demo with a fully charged battery.
- **Floor:** carpet, tile, and the arena floor have different friction. Calibrate on the arena floor if possible, or on a similar hard floor.
- **Weight:** calibrate with the arm and all parts installed, since the demo OTV must match the final design.

## Current limit

Each L298N channel is rated about 2 A, and each channel here powers two motors. Normal driving is well under that, but if both motors on one side stall (about 1.7 A each), the channel is overloaded. If the L298N gets very hot or the OTV stops partway through a turn, this is the likely cause. Don't let the OTV push against a wall or obstacle for long.

## Possible instructor questions

- **How does it know it turned 90°?** It doesn't measure the angle. The turn time was calibrated by testing, so running the motors for that time gives about 90°.
- **How does it turn without a steering mechanism?** Skid steer: the left wheels go backward and the right wheels go forward, so the OTV spins in place.
- **How are four motors driven by one L298N?** The two motors on each side are wired in parallel, so each L298N channel drives one side.
- **What do ENA/ENB and IN1–IN4 do?** IN pins set the direction (one HIGH, one LOW). ENA/ENB receive a PWM signal that sets the speed.
- **Why full speed?** The 10 RPM worm gear motors are already slow, and skid steer turning needs more torque than driving straight. A lower PWM would reduce torque.
- **What are the limits?** There is no feedback, so the angle can drift with battery charge and floor friction. The final navigation code will use the Vision System's heading instead.
