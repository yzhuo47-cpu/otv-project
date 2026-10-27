// mission.h - the full autonomous run as a state machine (follow Fig T1).
#pragma once

namespace mission {

void begin();

// Runs the current phase (blocking) and moves to the next one.
// Returns false once the run is finished (DONE or FAILED).
bool step();

}  // namespace mission
