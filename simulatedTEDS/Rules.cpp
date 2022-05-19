#include "Rules.h"


boolean isAboveThreshold(float sensorValue, float thresh)
{
  return (sensorValue > thresh);
}

boolean isBelowThreshold(float sensorValue, float thresh)
{
  return (sensorValue < thresh);
}

boolean isBetweenThresholds(float sensorValue, float thresh_low, float thresh_high)
{
  return ( (sensorValue > thresh_low) && (sensorValue < thresh_high) );
}

boolean isOutsideThresholds(float sensorValue, float thresh_low, float thresh_high)
{
  return !( (sensorValue > thresh_low) && (sensorValue < thresh_high) );
}

boolean averageIsAboveThreshold(float *values, int buffer_size, float thresh) {
  
  return isAboveThreshold( getAverage(values, buffer_size), thresh );
}

boolean averageIsBelowThreshold(float *values, int buffer_size, float thresh) {
  
  return isBelowThreshold( getAverage(values, buffer_size), thresh );
}

boolean useRule(char *ruleID, float *readings, int buffer_size, float thresh, float thresh_high, int isBufferFull) {

  float sensorValue = readings[0];

  if ( strcmp(ruleID, "isAboveThreshold") == 0 ) {
      return isAboveThreshold(sensorValue, thresh);
  }
  else if ( strcmp(ruleID, "isBelowThreshold") == 0 )
      return isBelowThreshold(sensorValue, thresh);
      
  else if ( strcmp(ruleID, "isBetweenThresholds") == 0 )
      return isBetweenThresholds(sensorValue, thresh, thresh_high);
      
  else if ( strcmp(ruleID, "isOutsideThresholds") == 0 )
      return isOutsideThresholds(sensorValue, thresh, thresh_high);

  else if ( strncmp("average", ruleID, strlen("average")) == 0 ) {
    if (isBufferFull) {

      if ( strcmp(ruleID, "averageIsAboveThreshold") == 0 ) {
        return averageIsAboveThreshold(readings, buffer_size, thresh);
      }

      if ( strcmp(ruleID, "averageIsBelowThreshold") == 0 ) {
        return averageIsBelowThreshold(readings, buffer_size, thresh);
      }
      
    } else {
      return false;
    }
  }
      
  else
    return false;
}

boolean useComplexRule(Sensor sensor) {
  //TODO: -in the arguments we need the second sensor for its threshold and rule values
  //      -do both rule checks
  //      -use the operator to return true or false
}


float getAverage(float *values, int buffer_size) {
  float average = values[0];
  for (int i=1; i<buffer_size; i++) {
      average += values[i];
  }
  average /= buffer_size;

  return average;
}
