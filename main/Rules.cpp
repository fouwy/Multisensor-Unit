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

boolean averageIsBetweenThresholds(float *values, int buffer_size, float thresh_low, float thresh_high) {
  
  return isBetweenThresholds( getAverage(values, buffer_size), thresh_low, thresh_high );
}


boolean averageIsOutsideThresholds(float *values, int buffer_size, float thresh_low, float thresh_high) {
  return isOutsideThresholds( getAverage(values, buffer_size), thresh_low, thresh_high );
}

boolean useRule(Sensor sensor) {

  char *ruleID = sensor.ruleID;
  float sensorValue = sensor.readings[0];
  int buffer_size = READINGS_BUFF_SIZE;
  float thresh = sensor.threshold;
  float thresh_high = sensor.high_threshold;
  boolean isBufferFull = sensor.BUFFER_FULL_FLAG;
  
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

      float *readings = sensor.readings;
      
      if ( strcmp(ruleID, "averageIsAboveThreshold") == 0 ) {
        return averageIsAboveThreshold(readings, buffer_size, thresh);
      }

      if ( strcmp(ruleID, "averageIsBelowThreshold") == 0 ) {
        return averageIsBelowThreshold(readings, buffer_size, thresh);
      }

      if ( strcmp(ruleID, "averageIsBetweenThresholds") == 0 ) {
        return averageIsBetweenThresholds(readings, buffer_size, thresh, thresh_high);
      }

      if ( strcmp(ruleID, "averageIsOutsideThresholds") == 0 ) {
        return averageIsOutsideThresholds(readings, buffer_size, thresh, thresh_high);
      }
      
    } else {
      return false;
    }
  }
      
  else
    return false;
}

boolean useComplexRule(Sensor sensor, Sensor secondSensor) {
  
  boolean firstCondition = useRule(sensor);
  boolean secondCondition = useRule(secondSensor);

  Serial.print("second sensor: ");
  Serial.println(sensor.op);
  Serial.print("first-");
  Serial.println(firstCondition);
  Serial.print("second-");
  Serial.println(secondCondition);
  return evaluateOperation(firstCondition, secondCondition, sensor.op);
}

boolean evaluateOperation(bool firstCondition, bool secondCondition, char *logicOperator) {

  if      ( strcmp(logicOperator, "AND") == 0 ) {
    return firstCondition && secondCondition; 
  } 
  else if ( strcmp(logicOperator, "OR") == 0 ) {
    return firstCondition || secondCondition;  
  }
  else if ( strcmp(logicOperator, "XOR") == 0 ) {
    return firstCondition ^ secondCondition;
  }
  else {
    Serial.println("Operator not recognized");
    return false;
  }
}

float getAverage(float *values, int buffer_size) {
  float average = values[0];
  for (int i=1; i<buffer_size; i++) {
      average += values[i];
  }
  average /= buffer_size;

  return average;
}
