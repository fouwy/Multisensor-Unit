#ifndef TEDSDevices_H
#define TEDSDevices_H

#include <Arduino.h>
#include <TaskManagerIO.h>

#define READINGS_BUFF_SIZE 5 //Buffer size to store readings of each sensor

typedef struct {
  
  boolean valid;
  char ID[20];
  taskid_t task_id;
  int pin;
  
  float threshold;                      //threshold for sending LoRa message
  float high_threshold;                 //To be used as high threshold in case of rule with two thresholds
  char ruleID[50];                      //Saves the name of the rule used with the thresholds
  char op[4] = "";                      //Operator used for complex rules(AND, OR, ...)
  char second_sensor[20];               //ID of the other sensor for the complex rule  
  boolean isSecondSensor = false;       //True when this sensor is part of a complex rule which another sensor is using
  
  float readings[READINGS_BUFF_SIZE] = {0};   //Stores the last "READINGS_BUFF_SIZE" readings of the sensor
  int buffer_length = 0;
  boolean BUFFER_FULL_FLAG = false;
  
  int aqui_rate;                        //aquisition rate in millis
  float calib_multiplier;               //multiplier for calibration of analog sensor
  
} Sensor;

//Analog sensors
#define WATER_LVL_SENSOR  0
#define TEMP_HUM_SENSOR   1
#define CO2_SENSOR        2
#define SOUND_SENSOR      3
#define AMMONIA_SENSOR    4

//Digital sensors
#define DHT11_SENSOR      0

//DHT11 Sensor defaults (no calibration required)
#define DHT11_ID            "dht11"
#define DHT11_THRESHOLD     40
#define DHT11_THRESH_HIGH   60
#define DHT11_AQUI_RATE     8000
#define DHT11_RULE_ID       "isAboveThreshold"


//WATER LEVEL SENSOR Defaults
#define WATER_LVL_ID          "water"
#define WATER_LVL_THRESHOLD   20
#define WATER_LVL_THRESH_HIGH   60
#define WATER_LVL_AQUI_RATE   8000
#define WATER_LVL_CALIB_MULT  1.2
#define WATER_LVL_RULE_ID     "isAboveThreshold"

//TEMP_HUM SENSOR Defaults
#define TEMP_HUM_ID           "heat"
#define TEMP_HUM_THRESHOLD    20
#define TEMP_HUM_THRESH_HIGH   60
#define TEMP_HUM_AQUI_RATE    10000
#define TEMP_HUM_CALIB_MULT   1.0
#define TEMP_HUM_RULE_ID      "isBetweenThresholds"

//CO2 SENSOR Defaults
#define CO2_ID                "co2"
#define CO2_THRESHOLD         20
#define CO2_THRESH_HIGH        60
#define CO2_AQUI_RATE         5000
#define CO2_CALIB_MULT        1.0
#define CO2_RULE_ID           "isAboveThreshold"

//SOUND SENSOR Defaults
#define SOUND_ID              "sound"
#define SOUND_THRESHOLD       20
#define SOUND_THRESH_HIGH     60
#define SOUND_AQUI_RATE       5000
#define SOUND_CALIB_MULT      1.0
#define SOUND_RULE_ID         "isBelowThreshold"

//AMMONIA SENSOR Defaults
#define AMMONIA_ID            "ammonia"
#define AMMONIA_THRESHOLD     20
#define AMMONIA_THRESH_HIGH   60
#define AMMONIA_AQUI_RATE     5000
#define AMMONIA_CALIB_MULT    1.0
#define AMMONIA_RULE_ID       "isAboveThreshold"

#endif
