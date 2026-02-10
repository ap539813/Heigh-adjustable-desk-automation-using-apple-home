/*
 * Utility Functions Implementation
 */

#include "utils.h"

float pulsesToHeight(int pulses) {
  return MIN_HEIGHT_CM + (pulses * CM_PER_PULSE);
}

int heightToPulses(float height) {
  return (int)((height - MIN_HEIGHT_CM) / CM_PER_PULSE);
}

float pulsesToPercentage(int pulses) {
  return ((float)(pulses - PULSES_AT_MIN_HEIGHT) / (PULSES_AT_MAX_HEIGHT - PULSES_AT_MIN_HEIGHT)) * 100.0;
}

int percentageToPulses(float percentage) {
  return PULSES_AT_MIN_HEIGHT + (int)((percentage / 100.0) * (PULSES_AT_MAX_HEIGHT - PULSES_AT_MIN_HEIGHT));
}
