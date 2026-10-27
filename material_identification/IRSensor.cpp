#include "IRSensor.h"
#include "Config.h"

static int plantableRef    = -1;
static int nonPlantableRef = -1;

void irBegin() {
  pinMode(IR_PIN, INPUT);
}

int irRead() {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(IR_PIN);
    delay(SAMPLE_DELAY_MS);
  }
  return (int)(sum / NUM_SAMPLES);
}

int irCalibratePlantable() {
  plantableRef = irRead();
  return plantableRef;
}

int irCalibrateNonPlantable() {
  nonPlantableRef = irRead();
  return nonPlantableRef;
}

bool irReferencesReady() {
  return plantableRef >= 0 && nonPlantableRef >= 0 &&
         abs(plantableRef - nonPlantableRef) >= MIN_SEPARATION;
}

int irPlantableRef()    { return plantableRef; }
int irNonPlantableRef() { return nonPlantableRef; }

// Whichever reference the reading is closer to wins. This works whether
// the module reads higher or lower when more IR is reflected.
bool irIsPlantable(int reading) {
  return abs(reading - plantableRef) < abs(reading - nonPlantableRef);
}