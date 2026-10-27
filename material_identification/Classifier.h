#pragma once

// Far plots, as seen from the OTV: top-left and top-right.
//   plantable-adjacent: far-left plantable,     far-right NON-plantable
//   plantable-diagonal: far-left NON-plantable, far-right plantable
enum FarPattern {
  PATTERN_BOTH,
  PATTERN_NEITHER,
  PATTERN_ADJACENT,
  PATTERN_DIAGONAL
};

FarPattern  classifyFar(bool farLeftPlantable, bool farRightPlantable);
const char* patternName(FarPattern p);