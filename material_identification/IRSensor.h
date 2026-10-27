#pragma once

// Sensor reading and reference calibration.
// The plots nearest the OTV are always known (left = orzo, right = gravel),
// so they are used as live references for every later reading.

void irBegin();
int  irRead();                    // averaged analog reading (0-1023)

int  irCalibratePlantable();      // sensor over near-left plot
int  irCalibrateNonPlantable();   // sensor over near-right plot
bool irReferencesReady();

int  irPlantableRef();
int  irNonPlantableRef();

// true = plantable (orzo), false = non-plantable (gravel)
bool irIsPlantable(int reading);