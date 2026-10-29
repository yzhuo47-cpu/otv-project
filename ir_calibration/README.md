# IR Calibration

Calibration of the TCRT5000 IR sensor used to tell orzo (plantable) from rocks (non-plantable)
(MS5 item 9: mission sensing). Readings and classification work exactly like `material_identification/`.

How to use:
1. Upload `ir_calibration.ino`, open Serial Monitor at 115200 baud with "New Line".
2. Hold the sensor at the IR-arm height above the substrate (use a fixed stand).
3. Over orzo: type `p` + Enter, move to a new spot, repeat about 8 times.
4. Over rocks: type `n` + Enter, move to a new spot, repeat about 8 times.
5. Type `c`: table (average, spread, min, max for each material), dividing value, and whether every
   sample is classified correctly. Copy the two `IR_..._DEFAULT` lines into `material_identification/Config.h`.
6. Demo: `tp` (sensor over orzo) / `tn` (over rocks) test new readings and keep a score; `v` = live readings.
   Repeat the tests under bright light (phone flashlight) to show it still works.
