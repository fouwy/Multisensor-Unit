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

  if (ruleID == "isAboveThreshold")
      return isAboveThreshold(sensorValue, thresh);
      
  else if (ruleID == "isBelowThreshold")
      return isBelowThreshold(sensorValue, thresh);
      
  else if (ruleID == "isBetweenThresholds")
      return isBetweenThresholds(sensorValue, thresh, thresh_high);
      
  else if (ruleID == "isOutsideThresholds")
      return isOutsideThresholds(sensorValue, thresh, thresh_high);
      
  else
    return false;
}
