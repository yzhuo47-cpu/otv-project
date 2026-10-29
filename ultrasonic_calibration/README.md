# Ultrasonic Calibration

Calibration of the HC-SR04 ultrasonic sensor used for obstacle avoidance (MS5 item 8: obstacle sensing).

How to use:
1. Wire VCC to 5V, GND to GND, TRIG and ECHO to the pins at the top of `ultrasonic_calibration.ino` (same as `navigation/config.h`).
2. Upload it, open Serial Monitor at 115200 baud with line ending "Newline".
3. Put a flat target at a measured distance (from the front of the sensor), type the distance in cm and press Enter.
   Repeat for about 6 distances between 10 and 100 cm.
4. Type `c`: calibration table (before/after errors) and two lines to paste into `navigation/config.h`.
5. Type `v`: live calibrated readings for the demo (Enter stops).
Then upload the navigation code again so the OTV uses the calibrated distances.
