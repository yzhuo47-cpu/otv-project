#include "Arduino.h"
#include "Enes100.h"
#include "Config.h"
#include "IRSensor.h"
#include "Classifier.h"
#include "MissionComm.h"

/*
  Serial Monitor: 115200 baud, "Newline" line ending
    r  live-read the sensor once
    p  sample PLANTABLE reference   (sensor over near-left plot, orzo)
    n  sample NON-PLANTABLE ref.    (sensor over near-right plot, gravel)
    c  classify whatever the sensor is over right now
    l  sample FAR-LEFT plot
    t  sample FAR-RIGHT plot (prints and sends the far-plot pattern)
    s  print stored references
*/

int farLeftReading  = -1;
int farRightReading = -1;

void printRefs() {
  Serial.print("plantableRef = ");    Serial.println(irPlantableRef());
  Serial.print("nonPlantableRef = "); Serial.println(irNonPlantableRef());
  Serial.print("separation = ");
  if (irPlantableRef() >= 0 && irNonPlantableRef() >= 0)
    Serial.println(abs(irPlantableRef() - irNonPlantableRef()));
  else
    Serial.println("n/a");
}

void finishFarScan() {
  if (farLeftReading < 0 || farRightReading < 0) return;
  FarPattern p = classifyFar(irIsPlantable(farLeftReading),
                             irIsPlantable(farRightReading));
  Serial.print("Result: ");
  Serial.println(patternName(p));
  commSendPattern(p);
  farLeftReading  = -1;
  farRightReading = -1;
}

bool refsOk() {
  if (!irReferencesReady()) {
    Serial.println("Set both references first (p, n), or separation is too small.");
    return false;
  }
  return true;
}

void handleCommand(char cmd) {
  switch (cmd) {
    case 'r':
      Serial.print("IR reading: ");
      Serial.println(irRead());
      break;
    case 'p':
      Serial.print("Plantable ref set: ");
      Serial.println(irCalibratePlantable());
      break;
    case 'n':
      Serial.print("Non-plantable ref set: ");
      Serial.println(irCalibrateNonPlantable());
      break;
    case 'c':
      if (refsOk()) {
        int v = irRead();
        Serial.print("Reading ");
        Serial.print(v);
        Serial.println(irIsPlantable(v) ? " -> PLANTABLE" : " -> NON-PLANTABLE");
      }
      break;
    case 'l':
      if (refsOk()) {
        farLeftReading = irRead();
        Serial.print("Far-left: ");
        Serial.println(farLeftReading);
        finishFarScan();
      }
      break;
    case 't':
      if (refsOk()) {
        farRightReading = irRead();
        Serial.print("Far-right: ");
        Serial.println(farRightReading);
        finishFarScan();
      }
      break;
    case 's':
      printRefs();
      break;
  }
}

void setup() {
  Serial.begin(115200);
  irBegin();
  commBegin();
  Serial.println("Material ID ready. Commands: r p n c l t s");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') handleCommand(c);
  }
}