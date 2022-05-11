#ifndef RULES_H
#define RULES_H
#include <Arduino.h>

boolean isAboveThreshold(float sensorValue, float thresh);

boolean isBelowThreshold(float sensorValue, float thresh);

boolean isBetweenThresholds(float sensorValue, float thresh_low, float thresh_high);

boolean useRule(char *ruleID, float sensorValue, float thresh, float thresh_high);

#endif
