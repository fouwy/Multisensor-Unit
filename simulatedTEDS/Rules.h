#ifndef RULES_H
#define RULES_H
#include <Arduino.h>

boolean isAboveThreshold(float sensorValue, float thresh);

boolean isBelowThreshold(float sensorValue, float thresh);

boolean isBetweenThresholds(float sensorValue, float thresh_low, float thresh_high);

boolean averageIsBelowThreshold(float *values, int buffer_size, float thresh);

boolean averageIsAboveThreshold(float *values, int buffer_size, float thresh);

boolean useRule(char *ruleID, float *readings, int buffer_size, float thresh, float thresh_high, int isBufferFull);

float getAverage(float *values, int buffer_size);

#endif
