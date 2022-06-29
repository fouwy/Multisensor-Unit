#ifndef RULES_H
#define RULES_H
#include <Arduino.h>
#include "TEDSDevices.h"

boolean isAboveThreshold(float sensorValue, float thresh);

boolean isBelowThreshold(float sensorValue, float thresh);

boolean isBetweenThresholds(float sensorValue, float thresh_low, float thresh_high);

boolean isOutsideThresholds(float sensorValue, float thresh_low, float thresh_high);

boolean averageIsBelowThreshold(float *values, int buffer_size, float thresh);

boolean averageIsAboveThreshold(float *values, int buffer_size, float thresh);

boolean averageIsBetweenThresholds(float *values, int buffer_size, float thresh_low, float thresh_high);

boolean averageIsOutsideThresholds(float *values, int buffer_size, float thresh_low, float thresh_high);

boolean useRule(Sensor sensor);

boolean useComplexRule(Sensor sensor, Sensor secondSensor); 

boolean evaluateOperation(bool firstCondition, bool secondCondition, char *logicOperator);

float getAverage(float *values, int buffer_size);

#endif
