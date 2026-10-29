# Navigation Simulator

Navigation code for the ENES100 web simulator (enes100.umd.edu/tools/simulator), for the MS5 navigation demo.
Same strategy as the real-robot code in `navigation/`, rewritten for the simulator (one file, uses the Tank library).

How to run:
1. Copy everything in `OTV_Simulator.ino` and paste it into the simulator's code editor (delete the example code first).
2. Click the speed button until it shows three arrows ("instant").
3. Click Randomize, or Remove Obstacles for the no-obstacle test.
4. Click Run, then Continue (the program pauses on its first line).
5. Watch the console: "== 1. Landing zone -> mission site" ... "== DONE". Keep the tab visible while it runs.
