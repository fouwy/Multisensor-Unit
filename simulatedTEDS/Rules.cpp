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


boolean useRule(char *ruleID, float sensorValue, float thresh, float thresh_high) {

  if ( strcmp(ruleID, "isAboveThreshold") == 0 ) {
      return isAboveThreshold(sensorValue, thresh);
  }
  else if ( strcmp(ruleID, "isBelowThreshold") == 0 )
      return isBelowThreshold(sensorValue, thresh);
      
  else if ( strcmp(ruleID, "isBetweenThresholds") == 0 )
      return isBetweenThresholds(sensorValue, thresh, thresh_high);
      
  else if ( strcmp(ruleID, "isOutsideThresholds") == 0 )
      return isOutsideThresholds(sensorValue, thresh, thresh_high);
      
  else
    return false;
}
