#ifndef TEDSDevices_H
#define TEDSDevices_H

#include <Arduino.h>

#define WATER_LVL_SENSOR  0
#define TEMP_HUM_SENSOR   1
#define CO2_SENSOR        2
#define SOUND_SENSOR      3
#define AMMONIA_SENSOR    4

//WATER LEVEL SENSOR Defaults
#define WATER_LVL_ID          "water"
#define WATER_LVL_THRESHOLD   20
#define WATER_LVL_AQUI_RATE   5000
#define WATER_LVL_CALIB_MULT  1.2

//TEMP_HUM SENSOR Defaults
#define TEMP_HUM_ID          "dht11"
#define TEMP_HUM_THRESHOLD   20
#define TEMP_HUM_AQUI_RATE   7000
#define TEMP_HUM_CALIB_MULT  1.0

//CO2 SENSOR Defaults
#define CO2_ID          "co2"
#define CO2_THRESHOLD   20
#define CO2_AQUI_RATE   5000
#define CO2_CALIB_MULT  1.0

//SOUND SENSOR Defaults
#define SOUND_ID          "sound"
#define SOUND_THRESHOLD   20
#define SOUND_AQUI_RATE   5000
#define SOUND_CALIB_MULT  1.0

//AMMONIA SENSOR Defaults
#define AMMONIA_ID          "ammonia"
#define AMMONIA_THRESHOLD   20
#define AMMONIA_AQUI_RATE   5000
#define AMMONIA_CALIB_MULT  1.0

#endif
