#include <FlashStorage.h>
#include <OneWire.h>
#include "TEDSDevices.h"
#include <TaskManagerIO.h>
#include "ArduinoLowPower.h"

#define MAX_SENSORS 10

OneWire  ds(8);    // 1-wire on pin 2
byte     addr[8];  // Contains the eeprom unique ID
byte memory[128];

typedef struct {
  
  boolean valid;
  int threshold;          //threshold for sending LoRa message
  int aqui_rate;          //aquisition rate in millis
  float calib_multiplier;  //multiplier for calibration of analog sensor
  char ID[20];
  
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

//Have to be global variables to pass to taskManager function
int deviceCount = 0;
float deviceCalib = 0;

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
  }
  

  Serial.println(teds[0].man_ID);

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
            sensors[i].threshold = WATER_LVL_THRESHOLD;
            sensors[i].aqui_rate = WATER_LVL_AQUI_RATE;
            sensors[i].calib_multiplier = WATER_LVL_CALIB_MULT;
            sensors[i].valid = true;
      
            waterSensor.write(sensors[i]);
          } else {
            Serial.println("Data found in memory.");
          }
          break;

        case TEMP_HUM_SENSOR:
          sensors[i] = heatSensor.read();

          if (sensors[i].valid == false) {
            Serial.println("No data in memory. Filling with defaults.");

            strcpy(sensors[i].ID, TEMP_HUM_ID);
            sensors[i].threshold = TEMP_HUM_THRESHOLD;
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
            sensors[i].threshold = CO2_THRESHOLD;
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
            sensors[i].threshold = SOUND_THRESHOLD;
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
            sensors[i].threshold = AMMONIA_THRESHOLD;
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
      Serial.println("Sensor not supported");
      while (1) {};
    }
  }
  
  
  //TEST this later
  //taskManager.scheduleFixedRate(1000, [] { Serial.println("1000 millis past!"); });

  //taskManager.scheduleFixedRate(2345, [] { Serial.println("2345 millis past!"); });

//  -------Actual code for the tasks
  for (int i=0; i < deviceAmount; i++) {
    deviceCalib = sensors[i].calib_multiplier;
    taskManager.scheduleFixedRate(sensors[i].aqui_rate, [deviceCalib]() {readSensor(deviceCalib);});
  }
  
}

void loop() {
  taskManager.runLoop();
  
  int millisDelay = (taskManager.microsToNextTask() / 1000UL);
  
  if(millisDelay > 100) {
    Serial.print("Enter low power for ");
    Serial.println(millisDelay);
  
    // here we call into the low power library for SAMD to reduce power usage for
    // the time that no tasks are running.
    USBDevice.detach();
    LowPower.deepSleep(millisDelay);
    USBDevice.attach();
    delay(1000);
    while(!Serial) {}

    //use a LED instead of Serial 
  }
  
}

void readSensor(float calib) {
  Serial.print("Toggle - ");
  Serial.println(calib); 
  
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
