#include "Enes100.h"

// ===== Vision system test =====
// 1. Serial Monitor (115200 baud) shows each connection step.
// 2. The vision system page shows our position once per second.
// USB power is enough: no battery, nothing moves.

const int MARKER_ID = 205;  // our ArUco marker
const int ROOM = 1201;      // open lab (1116 / 1120 for the classrooms)

void setup() {
  Serial.begin(115200);
  Enes100.setDebug(true);   // print each connection step (must be before begin)
  Enes100.begin("The Harvesters", SEED, MARKER_ID, ROOM);
}

void loop() {
  if (!Enes100.isConnected()) {
    Serial.println("not connected yet...");
  } else if (Enes100.isVisible()) {
    // Connected and the camera sees our marker
    Enes100.print("x = ");
    Enes100.print(Enes100.getX());
    Enes100.print("  y = ");
    Enes100.print(Enes100.getY());
    Enes100.print("  theta = ");
    Enes100.println(Enes100.getTheta());
    Serial.println("connected, marker visible");
  } else {
    // Connected, but the camera can't see marker 205
    Enes100.println("connected, marker NOT visible");
    Serial.println("connected, marker NOT visible");
  }
  delay(1000);
}
