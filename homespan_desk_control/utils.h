/*
 * Utility Functions
 * Height/Pulse/Percentage conversions
 */

#ifndef UTILS_H
#define UTILS_H

#include "config.h"

// Convert pulses to height in centimeters
float pulsesToHeight(int pulses);

// Convert height in centimeters to pulses
int heightToPulses(float height);

// Convert pulses to percentage (0-100)
float pulsesToPercentage(int pulses);

// Convert percentage (0-100) to pulses
int percentageToPulses(float percentage);

#endif
