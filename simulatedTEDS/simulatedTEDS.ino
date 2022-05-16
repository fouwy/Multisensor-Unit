#include <FlashStorage.h>
#include <OneWire.h>
#include "TEDSDevices.h"
#include <TaskManagerIO.h>
#include <ExecWithParameter.h>
#include <ArduinoLowPower.h>
#include "Rules.h"
#include <ArduinoJson.h>


#define MAX_SENSORS 10
#define READINGS_BUFF_SIZE 5 //Buffer size to store readings of each sensor

OneWire  ds(8);    // 1-wire on pin 8
byte     addr[8];  // Contains the eeprom unique ID
byte memory[128];

typedef struct {
  
  boolean valid;
  char ID[20];
  taskid_t task_id;
  
  float threshold;                      //threshold for sending LoRa message
  float high_threshold;                 //To be used as high threshold in case of rule with two thresholds
  char ruleID[30];                      //Saves the name of the rule used with the thresholds
  
  float readings[READINGS_BUFF_SIZE] = {0};   //Stores the last "READINGS_BUFF_SIZE" readings of the sensor
  int buffer_length = 0;
  int write_index = 0;
  
  int aqui_rate;                        //aquisition rate in millis
  float calib_multiplier;               //multiplier for calibration of analog sensor
  
} Sensor;

//Flash memory space for sensors
FlashStorage(waterSensor, Sensor);
FlashStorage(heatSensor, Sensor);   //This is temp+hum sensor
FlashStorage(co2Sensor, Sensor);
FlashStorage(soundSensor, Sensor);
FlashStorage(ammoniaSensor, Sensor);


typedef struct {
  
  int man_ID;
  int model;
  int ver_letter;
  int version;
  int serial;
    
} BasicTEDS;


Sensor sensors[MAX_SENSORS];
int deviceAmount = 0; //REVERT THIS VALUE TO 0 AFTER TESTING

StaticJsonDocument<96> doc;

void setup() {
  Serial.begin(9600);
  while (!Serial) { }

  
  while (SearchAddress(addr)) { //This will reset the search after no more devices are found.
    deviceAmount++;
  }
  
  if (deviceAmount == 0) {
    Serial.println("No TEDS device connected. Stopping...");
    while (1) {}
  }

  Serial.print("Device amount: ");
  Serial.println(deviceAmount);
  
  BasicTEDS teds[deviceAmount];
  
  for (int i=0; i<deviceAmount; i++) {
    SearchAddress(addr);
    ReadAndSave();
    getBasicTEDS(teds[i].man_ID, teds[i].model, teds[i].ver_letter, teds[i].version, teds[i].serial);

    Serial.print("Models: ");
    Serial.println(teds[i].model);
  }

  //Setup each sensor
  for (int i=0; i<deviceAmount; i++) {

    if (teds[i].man_ID == 0) {
      switch (teds[i].model) {
        case WATER_LVL_SENSOR:
          Serial.println("In water level sensor code");
          sensors[i] = waterSensor.read();

          if (sensors[i].valid == false) {
            Serial.println("No data in memory. Filling with defaults.");

            strcpy(sensors[i].ID, WATER_LVL_ID);
            strcpy(sensors[i].ruleID, WATER_LVL_RULE_ID);
            sensors[i].threshold = WATER_LVL_THRESHOLD;
            sensors[i].high_threshold = WATER_LVL_THRESH_HIGH;
            sensors[i].aqui_rate = WATER_LVL_AQUI_RATE;
            sensors[i].calib_multiplier = WATER_LVL_CALIB_MULT;
            sensors[i].valid = true;
      
            waterSensor.write(sensors[i]);
          } else {
            Serial.println("Data found in memory.");
          }
          break;

        case TEMP_HUM_SENSOR:
          Serial.println("In heat sensor code");
          sensors[i] = heatSensor.read();

          if (sensors[i].valid == false) {
            Serial.println("No data in memory. Filling with defaults.");

            strcpy(sensors[i].ID, TEMP_HUM_ID);
            strcpy(sensors[i].ruleID, TEMP_HUM_RULE_ID);
            sensors[i].threshold = TEMP_HUM_THRESHOLD;
            sensors[i].high_threshold = TEMP_HUM_THRESH_HIGH;
            sensors[i].aqui_rate = TEMP_HUM_AQUI_RATE;
            sensors[i].calib_multiplier = TEMP_HUM_CALIB_MULT;
            sensors[i].valid = true;
      
            heatSensor.write(sensors[i]);
          } else {
            Serial.println("Data found in memory.");
          }
          break;

        case CO2_SENSOR:
          sensors[i] = co2Sensor.read();
        
          if (sensors[i].valid == false) {
            Serial.println("No data in memory. Filling with defaults.");

            strcpy(sensors[i].ID, CO2_ID);
            strcpy(sensors[i].ruleID, CO2_RULE_ID);
            sensors[i].threshold = CO2_THRESHOLD;
            sensors[i].high_threshold = CO2_THRESH_HIGH;
            sensors[i].aqui_rate = CO2_AQUI_RATE;
            sensors[i].calib_multiplier = CO2_CALIB_MULT;
            sensors[i].valid = true;
      
            co2Sensor.write(sensors[i]);
          } else {
            Serial.println("Data found in memory.");
          }
          break;

        case SOUND_SENSOR:
          sensors[i] = soundSensor.read();
        
          if (sensors[i].valid == false) {
            Serial.println("No data in memory. Filling with defaults.");

            strcpy(sensors[i].ID, SOUND_ID);
            strcpy(sensors[i].ruleID, SOUND_RULE_ID);
            sensors[i].threshold = SOUND_THRESHOLD;
            sensors[i].high_threshold = SOUND_THRESH_HIGH;
            sensors[i].aqui_rate = SOUND_AQUI_RATE;
            sensors[i].calib_multiplier = SOUND_CALIB_MULT;
            sensors[i].valid = true;
      
            soundSensor.write(sensors[i]);
          } else {
            Serial.println("Data found in memory.");
          }
          break;

        case AMMONIA_SENSOR:
          sensors[i] = ammoniaSensor.read();
        
          if (sensors[i].valid == false) {
            Serial.println("No data in memory. Filling with defaults.");

            strcpy(sensors[i].ID, AMMONIA_ID);
            strcpy(sensors[i].ruleID, AMMONIA_RULE_ID);
            sensors[i].threshold = AMMONIA_THRESHOLD;
            sensors[i].high_threshold = AMMONIA_THRESH_HIGH;
            sensors[i].aqui_rate = AMMONIA_AQUI_RATE;
            sensors[i].calib_multiplier = AMMONIA_CALIB_MULT;
            sensors[i].valid = true;
      
            ammoniaSensor.write(sensors[i]);
          } else {
            Serial.println("Data found in memory.");
          }
          break;

        default:
          break;
      }
    } else {
      Serial.println("Sensor not supported...Stopping");
      while (1) {};
    }
  }
  
  for (int i=0; i < deviceAmount; i++) {
    auto task = new ExecWithParameter<int>(readSensor, i);
    sensors[i].task_id = taskManager.scheduleFixedRate(sensors[i].aqui_rate, task, TIME_MILLIS, true); 
  }

  //taskManager.scheduleOnce(10000, jsonCode, TIME_MILLIS);
    //>->->->->->->->->->JSON Code<-<-<<-<-<-<-<-<-<-<
  //This all will go on the loop section
  // run code when we get json LoRa message from central

  char json[200];
   
  doc["sensor"] = "dht11";
  doc["aqui_rate"] = 8000;
  doc["rules"][0] = "averageIsAboveThreshold";
  doc["threshold"] = 55;
  doc["high_threshold"] = 82;
  
  serializeJson(doc, json);

  const char* sensor = doc["sensor"]; // "dht11"
  int new_aqui_rate = doc["aqui_rate"]; // 8000
  const char* rules_0 = doc["rules"][0]; // "isBetweenThresholds"
  int threshold = doc["threshold"]; // 55
  int high_threshold = doc["high_threshold"]; // 82


  if ( isSensorConnected(sensor) ) {

    Serial.print("Sensor is connected: ");
    Serial.println(sensor);
    
    int sensorNumber = getSensorNumber(sensor);
    
    
    if ( new_aqui_rate != sensors[sensorNumber].aqui_rate ) {
      
      //change aquisition rate of this sensor
      sensors[sensorNumber].aqui_rate = new_aqui_rate;

      //TODO: write to flash memory the new aquisition rate and other settings
      

      //cancels the previous task and adds a new one with the new aquisition rate
      taskManager.cancelTask(sensors[sensorNumber].task_id);
      auto task = new ExecWithParameter<int>(readSensor, sensorNumber);
      sensors[sensorNumber].task_id = taskManager.scheduleFixedRate(sensors[sensorNumber].aqui_rate, task, TIME_MILLIS, true); 

      Serial.println("Changed aquisition rate!");
    }

    if ( rules_0 != sensors[sensorNumber].ruleID ) {

      //change rule of this sensor (Only for one rule per sensor)
      strcpy(sensors[sensorNumber].ruleID, rules_0);
      
    }

    if ( threshold != sensors[sensorNumber].threshold ) {
      sensors[sensorNumber].threshold = threshold;
    }

    if ( high_threshold != sensors[sensorNumber].high_threshold ) {
      sensors[sensorNumber].high_threshold = high_threshold;
    }
    
  }
//>->->->->->->->->->END OF JSON Code<-<-<<-<-<-<-<-<-<-<
  
}

void loop() {
  taskManager.runLoop();
  
  int millisDelay = (taskManager.microsToNextTask() / 1000UL);
  
//  if(millisDelay > 100) {
//    Serial.print("Enter low power for ");
//    Serial.println(millisDelay);
//  
//    // here we call into the low power library for SAMD to reduce power usage for
//    // the time that no tasks are running.
////    USBDevice.detach();
//    LowPower.idle(millisDelay);
////    USBDevice.attach();
////    delay(1000);
////    while(!Serial) {}
//
//    //use a LED instead of Serial 
//  }
  
}

void readSensor(int deviceNumber) {
  Serial.print("Toggle - ");
  Serial.print(sensors[deviceNumber].ID);
  Serial.print(" - ");
  Serial.println(sensors[deviceNumber].aqui_rate); 

  //TODO: DO an analog read of the sensor (this is just template code)
  //float reading = (float) analogRead(A3);
  int BUFFER_FULL_FLAG = 0;
  
  float reading = 500; //Delete this later, just for testing

  //Convert reading to 0-100 scale
  reading = reading / 10.23;
  reading = reading * sensors[deviceNumber].calib_multiplier;


  if ( sensors[deviceNumber].buffer_length < READINGS_BUFF_SIZE ) {
    sensors[deviceNumber].buffer_length++;
    
  } else {
    BUFFER_FULL_FLAG = 1;
  }
  
  for (int i=0; i < sensors[deviceNumber].buffer_length; i++) {

    if ( i == (READINGS_BUFF_SIZE-1) ) {  //To not cause buffer overflow
      continue;
    } else {
      sensors[deviceNumber].readings[i+1] = sensors[deviceNumber].readings[i];
    }
  }

  sensors[deviceNumber].readings[0] = reading;
    
  //Use the rule on the reading to determine if should send alert to cloud or not
  if ( useRule(sensors[deviceNumber].ruleID, sensors[deviceNumber].readings, READINGS_BUFF_SIZE, sensors[deviceNumber].threshold, sensors[deviceNumber].high_threshold, BUFFER_FULL_FLAG) ) {

    Serial.print("Rule activated! Rule: ");
    //send alert to cloud
    //by setting upload flag to 1 and filling the data variable with the alert
    //(this is on the other sleep_cycle code) 
    
  } else {
    Serial.print("Rule NOT activated! Rule: ");
  }
  
    Serial.print(sensors[deviceNumber].ruleID);
    Serial.print(" Reading: ");
    Serial.print(reading);
    Serial.print(" Threshold(s): (1) ");
    Serial.print(sensors[deviceNumber].threshold);
    Serial.print(" (2) ");
    Serial.print(sensors[deviceNumber].high_threshold);
    Serial.println();
    Serial.println();
  
  
}

boolean isSensorConnected(const char *sensor) {

  for (int i=0; i < MAX_SENSORS; i++) {

    if ( strcmp(sensors[i].ID, sensor) == 0 )
      return true;
  }
  
  return false;
}

int getSensorNumber(const char *sensor) {

  for (int i=0; i < deviceAmount; i++) {

    if ( strcmp(sensors[i].ID, sensor) == 0 )
      return i;
  }
  
  return -1;
}


void ReadAndSave()
{
  int i;
  ds.reset();
  ds.select(addr);
  ds.write(0xF0,1);  // Read Memory
  ds.write(0x00,1);  //Read Offset 0000h
  ds.write(0x00,1);

  for ( i = 0; i < 128; i++) //whole mem is 144 
  {
    memory[i] = ds.read();
  }
}

boolean SearchAddress(byte* address) //Search for address and confirm it
{
  int i;
  if ( !ds.search(address))
  {
    Serial.print("No device found.\n");
    ds.reset_search();
    delay(250);
    return false;
  }

  Serial.print("ADDR= ");
  for( i = 0; i < 8; i++)
  {
    Serial.print(address[i], HEX);
    Serial.print(" ");
  }

  if ( OneWire::crc8( address, 7) != address[7])
  {
    Serial.print("CRC is not valid, address is corrupted\n");
    return false;
  }

  if ( address[0] != 0x2D) 
  {
    Serial.print("Device is not a 1-wire Eeprom.\n");
    return false;
  }
  Serial.println();
  
  return true;
}

void getBasicTEDS(int& man_ID, int& model, int& ver_letter, int& version, int& serial) {

  man_ID = ( ( memory[1] & 0x3F ) << 8) | memory[0];
  model =  ( ( memory[3] & 0x1F ) << 10) | (memory[2] << 2) | ( memory[1] >> 6);
  ver_letter = ( ( memory[4] & 0x03 ) << 3) | (memory[3] >> 5);
  version = memory[4] >> 2;
  serial = (memory[7] << 16) | (memory[6] << 8) | (memory[5]);
}
