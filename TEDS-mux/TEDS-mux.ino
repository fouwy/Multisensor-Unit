#include <FlashStorage.h>
#include <OneWire.h>
#include "TEDSDevices.h"

OneWire  ds(2);    // 1-wire on pin 2
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


void setup() {
  Serial.begin(9600);
  while (!Serial) { }

  //TODO: recognize which sensors are connected
  
  int deviceAmount = 2; //REVERT THIS VALUE TO 0 AFTER TESTING
  
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
  Sensor sensors[deviceAmount];
  
  for (int i=0; i<deviceAmount; i++) {
    SearchAddress(addr);
    ReadAndSave();
    getBasicTEDS(teds[i].man_ID, teds[i].model, teds[i].ver_letter, teds[i].version, teds[i].serial);
  }
  

  Serial.println(teds[0].man_ID);
  Serial.println(teds[1].man_ID);


  //Setup each sensor
  for (int i=0; i<deviceAmount; i++) {

    if (teds[i].man_ID == 0) {
      switch (teds[i].model) {
        case WATER_LVL_SENSOR:
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
    }
    
  }
  
  while (1) { }
  
  Sensor waterLevel;

  waterLevel = waterSensor.read();

  if (waterLevel.valid == false) {

      Serial.println("No data in memory");

      strcpy(waterLevel.ID, "water");
      waterLevel.threshold = 20;
      waterLevel.aqui_rate = 5000;
      waterLevel.calib_multiplier = 1.0;
      waterLevel.valid = true;

      waterSensor.write(waterLevel);
  }

  else {
    
    Serial.println("Data found");
    Serial.print("Sensor ID: ");
    Serial.println(waterLevel.ID); 
    Serial.print("Threshold: ");
    Serial.println(waterLevel.threshold);
    Serial.print("Aquisition Rate: ");
    Serial.println(waterLevel.aqui_rate);
    Serial.print("Calibration Multiplier: ");
    Serial.println(waterLevel.calib_multiplier);
  }
  
}

void loop() {

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
  man_ID = 1;

//  man_ID = ( ( memory[1] & 0x3F ) << 8) | memory[0];
//  model =  ( ( memory[3] & 0x1F ) << 10) | (memory[2] << 2) | ( memory[1] >> 6);
//  ver_letter = ( ( memory[4] & 0x03 ) << 3) | (memory[3] >> 5);
//  version = memory[4] >> 2;
//  serial = (memory[7] << 16) | (memory[6] << 8) | (memory[5]);
}
