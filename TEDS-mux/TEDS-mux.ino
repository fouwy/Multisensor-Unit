#include <FlashStorage.h>
#include <OneWire.h>
#include "TEDSDevices.h"
#include <TaskManagerIO.h>
#include <ExecWithParameter.h>
#include "ArduinoLowPower.h"

#define MAX_SENSORS 10

const int selectPins[3] = {2, 3, 4};  // S-pins to Arduino pins: S0~2, S1~3, S2~4
const int sensorMuxPin  = A0;  //Common output of multiplexer
const int TEDSMuxPin    = 8;

OneWire  ds(TEDSMuxPin);    // 1-wire on pin TEDSMuxPin
byte     addr[8];  // Contains the eeprom unique ID
byte memory[128];

typedef struct {
  
  boolean valid;
  int pin;
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

int deviceAmount = 0;

//Have to be global variables to pass to taskManager function
int deviceCount = 0;
float deviceCalib = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) { }


  for (int i=0; i<3; i++)
  {
    pinMode(selectPins[i], OUTPUT);
    digitalWrite(selectPins[i], LOW);
  }
  
  //TODO: recognize which sensors are connected
  //

  BasicTEDS teds[MAX_SENSORS];
  
  for (int pin=0; pin<=7; pin++) {
    
    selectMuxPin(pin);
    
    if (SearchAddress(addr)) {

      Serial.print("Found device on pin ");
      Serial.println(pin);
      ReadAndSave();
      getBasicTEDS(teds[pin].man_ID, teds[pin].model, teds[pin].ver_letter, teds[pin].version, teds[pin].serial);
      setupSensor(teds[pin], deviceAmount, pin);
      
      deviceAmount++;
    }
  }
  
  if (deviceAmount == 0) {
    Serial.println("No TEDS device connected. Stopping...");
    while (1) {}
  }
  
  Serial.print("In setup Pin: ");
  Serial.println(sensors[1].pin);
  
  Serial.print("Device amount: ");
  Serial.println(deviceAmount);

  for (int i=0; i < deviceAmount; i++) {
    auto task = new ExecWithParameter<int>(readSensor, i);
    taskManager.scheduleFixedRate(sensors[i].aqui_rate, task, TIME_MILLIS, true); 
  }
  
}

void loop() {
  taskManager.runLoop();
  
}



void readSensor(int deviceNumber) {
  Serial.print("Toggle - ");
  Serial.println(deviceNumber); 
  
  int val = 0;

  Serial.print("Pin: ");
  Serial.println(sensors[deviceNumber].pin);
  selectMuxPin(sensors[deviceNumber].pin);
  
  //put mux on correct output
  //Read sensor
  //Calibrate it 
  //Store it
  
  val = analogRead(sensorMuxPin);

  Serial.print(sensors[deviceNumber].ID);
  Serial.print(" sensor reading: ");
  Serial.println(val);
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

void setupSensor(BasicTEDS teds, int deviceNum, int pin) {

  if (teds.man_ID == 0) {

      switch (teds.model) {
        case WATER_LVL_SENSOR:
          sensors[deviceNum] = waterSensor.read();

          if (sensors[deviceNum].valid == false) {
            Serial.println("Water. No data in memory. Filling with defaults.");

            strcpy(sensors[deviceNum].ID, WATER_LVL_ID);
            sensors[deviceNum].threshold = WATER_LVL_THRESHOLD;
            sensors[deviceNum].aqui_rate = WATER_LVL_AQUI_RATE;
            sensors[deviceNum].calib_multiplier = WATER_LVL_CALIB_MULT;
            sensors[deviceNum].valid = true;
            
            waterSensor.write(sensors[deviceNum]);
          } else {
            Serial.println("Data found in memory.");
          }
          break;

        case TEMP_HUM_SENSOR:
          sensors[deviceNum] = heatSensor.read();

          if (sensors[deviceNum].valid == false) {
            Serial.println("Heat. No data in memory. Filling with defaults.");

            strcpy(sensors[deviceNum].ID, TEMP_HUM_ID);
            sensors[deviceNum].threshold = TEMP_HUM_THRESHOLD;
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
            sensors[deviceNum].threshold = CO2_THRESHOLD;
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
            sensors[deviceNum].threshold = SOUND_THRESHOLD;
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
            sensors[deviceNum].threshold = AMMONIA_THRESHOLD;
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
      Serial.println("Sensor not supported");
    }
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
