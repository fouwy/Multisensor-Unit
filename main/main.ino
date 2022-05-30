#include <FlashStorage.h>
#include <OneWire.h>
#include "TEDSDevices.h"
#include <TaskManagerIO.h>
#include <ExecWithParameter.h>
#include <ArduinoLowPower.h>
#include "Rules.h"
#include <ArduinoJson.h>
// LoRa Library
#include <MKRWAN.h>
//Digital Sensors Libraries
#include "DHT.h"

#define MAX_SENSORS 10

//LoRa Stuff
LoRaModem modem;
const char* appEui = "0000000000000000"; 
const char* appKey = "FB6DDF75D139AEC02424D78F1DE3DCCD"; 
char message[200] = ""; // LoRa Packet Content

//Multiplexer pins
const int selectPins[3]           = {2, 3, 4};  // S-pins to Arduino pins: S0~2, S1~3, S2~4
const int sensorMuxPin            = A0;  //Common output of multiplexer
const int digitalSensorMuxPin     = 10;
const int TEDSMuxPin              = 8;
const int sensorEnablePin         = 1;   //Enable pin for the multiplexer
const int digitalSensorEnablePin  = 5;
const int TEDSEnablePin           = 0;

//Digital sensors setup
DHT dht(digitalSensorMuxPin, DHT11);

//TEDS chip
OneWire  ds(TEDSMuxPin);    // 1-wire pin
byte     addr[8];  // Contains the eeprom unique ID
byte memory[128];

//Flash memory space for sensors
FlashStorage(waterSensor, Sensor);
FlashStorage(heatSensor, Sensor);   //This is temp+hum sensor
FlashStorage(co2Sensor, Sensor);
FlashStorage(soundSensor, Sensor);
FlashStorage(ammoniaSensor, Sensor);
FlashStorage(dht11Sensor, Sensor);

typedef struct {
  
  int man_ID;
  int model;
  int ver_letter;
  int version;
  int serial;
    
} BasicTEDS;

Sensor sensors[MAX_SENSORS];
int deviceAmount = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) { }

  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  };

  int connected = modem.joinOTAA(appEui, appKey);

  if (!connected) {
    Serial.println("Something went wrong...Exiting");
    while (1) {}
  }
  delay(2000);
  
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
    delay(200);
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

  //TODO: read digital sensor

  selectMuxPin(sensors[deviceNumber].pin);
  
  if (sensors[deviceNumber].calib_multiplier == 0) {  //Means sensor is digital
    changeMuxState(digitalSensorEnablePin, LOW);
    reading = readDigitalSensor(sensors[deviceNumber].ID);
  } 
  else {
    changeMuxState(sensorEnablePin, LOW);
    reading = analogRead(sensorMuxPin);
    reading = reading / 10.23;
    //reading = reading * sensors[deviceNumber].calib_multiplier;
  }

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

      LoRaPacketSender();
    }
  }
  //Use the rule on the reading to determine if should send alert to cloud or not
  else if ( useRule(sensors[deviceNumber]) ) {

    Serial.print("Rule activated! Rule: ");
    
    LoRaPacketSender();
    
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
  changeMuxState(digitalSensorEnablePin, HIGH);
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

    if (teds.man_ID == 0) {   //This is an analog sensor
      
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
      
    } 
    else if (teds.man_ID == 1) {  //This is a digital sensor
      switch (teds.model) {
        case DHT11_SENSOR:
          dht.begin();
          sensors[deviceNum] = dht11Sensor.read();
        
          if (sensors[deviceNum].valid == false) {
            Serial.println("No data in memory. Filling with defaults.");

            strcpy(sensors[deviceNum].ID, DHT11_ID);
            strcpy(sensors[deviceNum].ruleID, DHT11_RULE_ID);
            sensors[deviceNum].threshold = DHT11_THRESHOLD;
            sensors[deviceNum].high_threshold = DHT11_THRESH_HIGH;
            sensors[deviceNum].aqui_rate = DHT11_AQUI_RATE;
            sensors[deviceNum].calib_multiplier = 0;                //Calib multiplier at zero means its digital sensor
            sensors[deviceNum].valid = true; 

            dht11Sensor.write(sensors[deviceNum]);
          } else {
            Serial.println("Data found in memory.");
          }
          break;

        default:
          break;
      }
      
       sensors[deviceNum].pin = pin;
    } 
    else {
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

void LoRaPacketSender() {

  int err;

  modem.beginPacket();
  modem.print("hi");
  err = modem.endPacket(false);

  if (err > 0) {
    Serial.println("Message sent correctly!");
  } else {
    Serial.println("Error sending message :(");
  }

  if (!modem.available()) {
    Serial.println("No downlink message received at this time.");
  }
  else  {
    LoRaPacketReceiver();
  }
}

void LoRaPacketReceiver() {
  char rcv[200];
  int i = 0;
  while (modem.available()) {
    rcv[i++] = (char)modem.read();
  }
  Serial.print("Received: ");
  for (unsigned int j = 0; j < i; j++) {
    Serial.print(rcv[j] >> 4, HEX);
    Serial.print(rcv[j] & 0xF, HEX);
    Serial.print(" ");
    message[j] = rcv[j];
  }
  Serial.println();

  taskManager.execute([] {
        processJSON(message);
    });
  
}

void processJSON(char * message) {
  StaticJsonDocument<200> doc;
  deserializeMsgPack(doc, message);
  serializeJson(doc, Serial);
  Serial.println();

  const char* sensor = doc["sensor"];
  int aqui_rate = doc["aqui_rate"];
  const char* rule = doc["rule"];
  int threshold = doc["thresh"];
  int high_threshold = doc["h_thresh"];
  const char* op = doc["op"];
  const char* second_sensor = doc["sensor2"]; 
  
  if ( isSensorConnected(sensor) ) {

    Serial.print("Sensor is connected: ");
    Serial.println(sensor);

    Serial.print("Second sensor: ");
    Serial.println(second_sensor);
    int sensorNumber = getSensorNumber(sensor);

    //TODO: in every if(), check if json field is zero or NULL
    
    if ( aqui_rate != sensors[sensorNumber].aqui_rate && aqui_rate != 0) {
      
      //change aquisition rate of this sensor
      sensors[sensorNumber].aqui_rate = aqui_rate;

      //TODO: write to flash memory the new aquisition rate and other settings
      

      //cancels the previous task and adds a new one with the new aquisition rate
      taskManager.cancelTask(sensors[sensorNumber].task_id);
      auto task = new ExecWithParameter<int>(readSensor, sensorNumber);
      sensors[sensorNumber].task_id = taskManager.scheduleFixedRate(sensors[sensorNumber].aqui_rate, task, TIME_MILLIS, true); 

      Serial.println("Changed aquisition rate!");
    }

    if ( rule != sensors[sensorNumber].ruleID && rule != NULL) {

      //change rule of this sensor (Only for one rule per sensor)
      strcpy(sensors[sensorNumber].ruleID, rule);

      if ( threshold != sensors[sensorNumber].threshold ) {
        sensors[sensorNumber].threshold = threshold;
      }

      if ( high_threshold != sensors[sensorNumber].high_threshold ) {
        sensors[sensorNumber].high_threshold = high_threshold;
      }
      
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
}

float readDigitalSensor(char *sensorName) {

  if ( strcmp(sensorName, DHT11_ID) == 0 ) {

    Serial.println("in read digital sensor");
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float hic = dht.computeHeatIndex(t, h, false);

    return hic;
  }

  
}
