// site.h - what the OTV does once it is parked at the mission site.
// NAVIGATION ONLY CALLS THIS. The arm / IR teammates fill in site.cpp.
#pragma once
#include "vision.h"

namespace site {

// Called with the OTV stopped SITE_STANDOFF_CM in front of the mission site,
// facing it. Do the arm + IR work here, then return the far-plot pattern
// (or PLOTS_UNKNOWN if it could not be measured - nothing is reported then).
// Must return; navigation continues to the obstacle zone afterwards.
PlotPattern runTask();

}  // namespace site
