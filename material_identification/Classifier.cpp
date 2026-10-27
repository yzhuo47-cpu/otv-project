#include "Classifier.h"

FarPattern classifyFar(bool farLeftPlantable, bool farRightPlantable) {
  if (farLeftPlantable && farRightPlantable)   return PATTERN_BOTH;
  if (!farLeftPlantable && !farRightPlantable) return PATTERN_NEITHER;
  if (farLeftPlantable && !farRightPlantable)  return PATTERN_ADJACENT;
  return PATTERN_DIAGONAL;
}

const char* patternName(FarPattern p) {
  switch (p) {
    case PATTERN_BOTH:    return "BOTH PLANTABLE";
    case PATTERN_NEITHER: return "NEITHER PLANTABLE";
    case PATTERN_ADJACENT:return "PLANTABLE-ADJACENT";
    default:              return "PLANTABLE-DIAGONAL";
  }
}