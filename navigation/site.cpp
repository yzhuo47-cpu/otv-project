#include "site.h"
#include "config.h"

namespace site {

PlotPattern runTask() {
  // When this runs, the OTV is parked BROADSIDE next to the 2x2 plot grid,
  // arm side (right, ARM_ON_RIGHT) facing the plots, wheels ~2 cm from the
  // grid (like Fig H3). Tune SITE_X / SITE_STOP_Y_* in config.h together
  // with the arm team so the arm lines up with the plots.
  //
  // TODO (arm: Jason/Jesse, IR: Subhaan/Mai/Jonathan - Week 2), Fig M3 order:
  //   1. near-left plot (orzo): lower arm, plant seed 10 mm deep, bury
  //   2. near-right plot (rocks): collect >= 10 g, close lid, retract arm
  //   3. read TCRT5000 over the two far plots and classify
  //      (threshold from the Week 1 IR test) -> return the pattern
  //   Note: Fig T1 classifies the far plots FIRST - pick one order as a team.
  //   Time budget: navigation takes up to ~210 s of the 300 s, so keep this
  //   under ~80 s. Leave the arm RETRACTED: navigation assumes the OTV fits
  //   in a 0.175 m turning circle (OTV_TURN_RADIUS).
  // Keep using vision::say(F("...")) so progress shows on the Vision console.
  vision::say(F("[site] stub: no arm/IR code yet"));
  delay(2000);
  return PLOTS_UNKNOWN;
}

}  // namespace site
