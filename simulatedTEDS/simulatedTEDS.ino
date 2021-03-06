#include <FlashStorage.h>
#include <OneWire.h>
#include "TEDSDevices.h"
#include <TaskManagerIO.h>
#include <ExecWithParameter.h>
#include <ArduinoLowPower.h>
#include "Rules.h"
#include <ArduinoJson.h>

#define MAX_SENSORS 10

const int selectPins[3]   = {2, 3, 4};  // S-pins to Arduino pins: S0~2, S1~3, S2~4
const int sensorMuxPin    = A0;  //Common output of multiplexer
const int TEDSMuxPin      = 8;
const int sensorEnablePin = 1;   //Enable pin for the multiplexer 
const int TEDSEnablePin   = 0;

OneWire  ds(TEDSMuxPin);    // 1-wire on pin 8
byte     addr[8];  // Contains the eeprom unique ID
byte memory[128];

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
int deviceAmount = 0;

StaticJsonDocument<128> doc;

void setup() {
  Serial.begin(9600);
  while (!Serial) { }

  //Config multiplexers
  for (int i=0; i<3; i++)
  {
    pinMode(selectPins[i], OUTPUT);
    digitalWrite(selectPins[i], LOW);
  }
  pinMode(sensorEnablePin, OUTPUT);
  pinMode(TEDSEnablePin, OUTPUT);
  digitalWrite(sensorEnablePin, LOW);
  digitalWrite(TEDSEnablePin, LOW);

  BasicTEDS teds[MAX_SENSORS];

  for (int pin=0; pin < 8; pin++) {
    
    selectMuxPin(pin);

    Serial.print("Pin "); Serial.print(pin); Serial.print(" --> ");
    
    if (SearchAddress(addr)) {

      ReadAndSave();
      getBasicTEDS(teds[deviceAmount].man_ID, teds[deviceAmount].model, teds[deviceAmount].ver_letter, teds[deviceAmount].version, teds[deviceAmount].serial);
      setupSensor(teds[deviceAmount], deviceAmount, pin);
      
      Serial.print("Found device on pin ");
      Serial.print(pin);
      Serial.print(" -> Sensor: ");
      Serial.println(sensors[deviceAmount].ID);
      
      deviceAmount++;
    }
    delay(100);
  }

  changeMuxState(TEDSEnablePin, HIGH);  //disable TEDS mux, no longer needed
 
  if (deviceAmount == 0) {
    Serial.println("No TEDS device connected. Stopping...");
    while (1) {}
  }

  Serial.print("Device amount: ");
  Serial.println(deviceAmount);
  
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
  doc["rules"][0] = "isAboveThreshold";
  doc["threshold"] = 47;
  doc["high_threshold"] = 82;
  doc["logic_operator"] = "AND";
  doc["sensor2"] = "water";
  
  serializeJson(doc, json);

  const char* sensor = doc["sensor"]; // "dht11"
  int new_aqui_rate = doc["aqui_rate"]; // 8000
  const char* rules_0 = doc["rules"][0]; // "isBetweenThresholds"
  int threshold = doc["threshold"]; // 55
  int high_threshold = doc["high_threshold"]; // 82
  const char* op = doc["logic_operator"]; // "AND"
  const char* second_sensor = doc["sensor2"]; // "water"
    
  if ( isSensorConnected(sensor) ) {

    Serial.print("Sensor is connected: ");
    Serial.println(sensor);

    Serial.print("Second sensor: ");
    Serial.println(second_sensor);
    int sensorNumber = getSensorNumber(sensor);

    //TODO: in every if(), check if json field is zero or NULL
    
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
      
    if ( strcmp(op, "") == 0 && strcmp(sensors[sensorNumber].op, "") != 0 ) {
      //When changing back from complex rule to normal, correct also the second sensor

      Serial.println("changing back to simple rules");
      
      strcpy(sensors[sensorNumber].op, op);
      strcpy(sensors[sensorNumber].second_sensor, "");
      
      sensors[getSensorNumber(sensors[sensorNumber].second_sensor)].isSecondSensor = false;
    }
    
    if (op != NULL) {

        Serial.println("op not null");
        
        if ( isSensorConnected(second_sensor) ) {

          Serial.println("second_sensor connected");

          sensors[getSensorNumber(second_sensor)].isSecondSensor = true;
          
          strcpy(sensors[sensorNumber].second_sensor, second_sensor);
          strcpy(sensors[sensorNumber].op, op);
        }
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
  Serial.print(" num - ");
  Serial.print(deviceNumber);
  Serial.print(" second sensor id ");
  Serial.print(sensors[deviceNumber].second_sensor);
  Serial.print(" - ");
  Serial.println(sensors[deviceNumber].aqui_rate); 

  
  float reading;

  changeMuxState(sensorEnablePin, LOW);
  selectMuxPin(sensors[deviceNumber].pin);
  
  reading = analogRead(sensorMuxPin);
  
  //Convert reading to 0-100 scale
  reading = reading / 10.23;
  
  //reading = reading * sensors[deviceNumber].calib_multiplier;


  if ( sensors[deviceNumber].buffer_length < READINGS_BUFF_SIZE ) {
    sensors[deviceNumber].buffer_length++;
    
  } else {
    sensors[deviceNumber].BUFFER_FULL_FLAG = true;
  }
  
  for (int i=0; i < sensors[deviceNumber].buffer_length; i++) {

    if ( i == (READINGS_BUFF_SIZE-1) ) {  //To not cause buffer overflow
      continue;
    } else {
      sensors[deviceNumber].readings[i+1] = sensors[deviceNumber].readings[i];
    }
  }

  sensors[deviceNumber].readings[0] = reading;

  //If this sensor is part of a complex rule, dont use its own rule now
  if (sensors[deviceNumber].isSecondSensor) {
    Serial.println("Is second sensor in complex rule");
  }
  
  //Check if this sensor is using complex rule first
  else if ( strcmp(sensors[deviceNumber].op, "") != 0 ) {
    Serial.println("op is not null");
    //TODO: get the other sensor info
    int secondSensorNumber = getSensorNumber(sensors[deviceNumber].second_sensor);

    
    if ( useComplexRule(sensors[deviceNumber], sensors[secondSensorNumber]) ) {
      Serial.println("Complex rule active!");
    }
  }
  //Use the rule on the reading to determine if should send alert to cloud or not
  else if ( useRule(sensors[deviceNumber]) ) {

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
  

  changeMuxState(sensorEnablePin, HIGH);
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

void setupSensor(BasicTEDS teds, int deviceNum, int pin) {

    if (teds.man_ID == 0) {
      
      switch (teds.model) {
        
        case WATER_LVL_SENSOR:
          Serial.println("In water level sensor code");
          sensors[deviceNum] = waterSensor.read();

          if (sensors[deviceNum].valid == false) {
            Serial.println("No data in memory. Filling with defaults.");

            strcpy(sensors[deviceNum].ID, WATER_LVL_ID);
            strcpy(sensors[deviceNum].ruleID, WATER_LVL_RULE_ID);
            sensors[deviceNum].threshold = WATER_LVL_THRESHOLD;
            sensors[deviceNum].high_threshold = WATER_LVL_THRESH_HIGH;
            sensors[deviceNum].aqui_rate = WATER_LVL_AQUI_RATE;
            sensors[deviceNum].calib_multiplier = WATER_LVL_CALIB_MULT;
            sensors[deviceNum].valid = true;
      
            waterSensor.write(sensors[deviceNum]);
          } else {
            Serial.println("Data found in memory.");
          }
          break;

        case TEMP_HUM_SENSOR:
          Serial.println("In heat sensor code");
          sensors[deviceNum] = heatSensor.read();

          if (sensors[deviceNum].valid == false) {
            Serial.println("No data in memory. Filling with defaults.");

            strcpy(sensors[deviceNum].ID, TEMP_HUM_ID);
            strcpy(sensors[deviceNum].ruleID, TEMP_HUM_RULE_ID);
            sensors[deviceNum].threshold = TEMP_HUM_THRESHOLD;
            sensors[deviceNum].high_threshold = TEMP_HUM_THRESH_HIGH;
            sensors[deviceNum].aqui_rate = TEMP_HUM_AQUI_RATE;
            sensors[deviceNum].calib_multiplier = TEMP_HUM_CALIB_MULT;
            sensors[deviceNum].valid = true;
      
            heatSensor.write(sensors[deviceNum]);
          } else {
            Serial.println("Data found in memory.");
          }
          break;

        case CO2_SENSOR:
          sensors[deviceNum] = co2Sensor.read();
        
          if (sensors[deviceNum].valid == false) {
            Serial.println("No data in memory. Filling with defaults.");

            strcpy(sensors[deviceNum].ID, CO2_ID);
            strcpy(sensors[deviceNum].ruleID, CO2_RULE_ID);
            sensors[deviceNum].threshold = CO2_THRESHOLD;
            sensors[deviceNum].high_threshold = CO2_THRESH_HIGH;
            sensors[deviceNum].aqui_rate = CO2_AQUI_RATE;
            sensors[deviceNum].calib_multiplier = CO2_CALIB_MULT;
            sensors[deviceNum].valid = true;
      
            co2Sensor.write(sensors[deviceNum]);
          } else {
            Serial.println("Data found in memory.");
          }
          break;

        case SOUND_SENSOR:
          sensors[deviceNum] = soundSensor.read();
        
          if (sensors[deviceNum].valid == false) {
            Serial.println("No data in memory. Filling with defaults.");

            strcpy(sensors[deviceNum].ID, SOUND_ID);
            strcpy(sensors[deviceNum].ruleID, SOUND_RULE_ID);
            sensors[deviceNum].threshold = SOUND_THRESHOLD;
            sensors[deviceNum].high_threshold = SOUND_THRESH_HIGH;
            sensors[deviceNum].aqui_rate = SOUND_AQUI_RATE;
            sensors[deviceNum].calib_multiplier = SOUND_CALIB_MULT;
            sensors[deviceNum].valid = true;
      
            soundSensor.write(sensors[deviceNum]);
          } else {
            Serial.println("Data found in memory.");
          }
          break;

        case AMMONIA_SENSOR:
          sensors[deviceNum] = ammoniaSensor.read();
        
          if (sensors[deviceNum].valid == false) {
            Serial.println("No data in memory. Filling with defaults.");

            strcpy(sensors[deviceNum].ID, AMMONIA_ID);
            strcpy(sensors[deviceNum].ruleID, AMMONIA_RULE_ID);
            sensors[deviceNum].threshold = AMMONIA_THRESHOLD;
            sensors[deviceNum].high_threshold = AMMONIA_THRESH_HIGH;
            sensors[deviceNum].aqui_rate = AMMONIA_AQUI_RATE;
            sensors[deviceNum].calib_multiplier = AMMONIA_CALIB_MULT;
            sensors[deviceNum].valid = true;
      
            ammoniaSensor.write(sensors[deviceNum]);
          } else {
            Serial.println("Data found in memory.");
          }
          break;

        default:
          break;
      }

      sensors[deviceNum].pin = pin;
      
    } else {
      Serial.println("Sensor not supported...Stopping");
      while (1) {};
    }
}

void selectMuxPin(byte pin)
{
  if (pin > 7) return; // Exit if pin is out of scope
  for (int i=0; i<3; i++)
  {
    if (pin & (1<<i))
      digitalWrite(selectPins[i], HIGH);
    else
      digitalWrite(selectPins[i], LOW);
  }
}

void changeMuxState(int enableMuxPin, int state) {
  
  digitalWrite(enableMuxPin, state);
}
