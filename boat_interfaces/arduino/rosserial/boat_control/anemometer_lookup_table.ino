const int LookupTableLength = 5;
float lookupTable[LookupTableLength][2] = {
  // each entry has format: {apparent direction, true direction}
  // have at least 2 entries here!
  // max of 180 degrees between the apparent directions of one entry and the next.
  // ensure that the LookupTableLength constants above matches that amount of entries here.
  {0, 0},
  {60, 60},
  {120, 120},
  {180, 180},
  {240, 240}
};

//call this in setup if you don't want to manually sort the lookup table
void bubbleSortlookupTable() {
  for (int i = 1; i < LookupTableLength; i++) {
    for (int j = i - 1; j >= 0; j--) {
      if (lookupTable[j][0] > lookupTable[j + 1][0]) {
        float t0 = lookupTable[j][0];
        float t1 = lookupTable[j][1];
        lookupTable[j][0] = lookupTable[j + 1][0];
        lookupTable[j][1] = lookupTable[j + 1][1];
        lookupTable[j + 1][0] = t0;
        lookupTable[j + 1][1] = t1;
      }
    }
  }
}

//expects 0-360, returns 0-360
float lookupTrueWindDirection(float apparentDirection) {
  // assume table is sorted
  // it should have been sorted in setup()
  int lowerIndex;
  int upperIndex;
  if (lookupTable[0][0] > apparentDirection || lookupTable[LookupTableLength - 1][0] <= apparentDirection) {
    lowerIndex = LookupTableLength - 1;
    upperIndex = 0;
  } else {
    for (int i = 1; i < LookupTableLength; i++) {
      if (lookupTable[i][0] > apparentDirection) {
        lowerIndex = i - 1;
        upperIndex = i;
        break;
      }
    }
  }
  float frac = (angleDifference(apparentDirection, lookupTable[lowerIndex][0]))
                / angleDifference(lookupTable[upperIndex][0], lookupTable[lowerIndex][0]);
  float trueValue = lookupTable[lowerIndex][1] + frac * angleDifference (lookupTable[lowerIndex][1], lookupTable[upperIndex][1]);
  if (trueValue < 0) {
    trueValue += 360;
  }
  return trueValue;
}

float angleDifference(float a1, float a2) {
  float diff = fmod(a2 - a1, 360);
  return fmod(2 * diff, 360) - diff;
}


