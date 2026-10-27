#pragma once
#include "Classifier.h"

void commBegin();                      // connect to the vision system
void commSendPattern(FarPattern p);    // send the far-plot result