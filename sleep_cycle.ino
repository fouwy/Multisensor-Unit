// Low Power Library
#include "ArduinoLowPower.h"

//Temp+humidity sensor
#include "DHT.h"

//Payload Formatter
#include<CayenneLPP.h>

// LoRa Library
#include <SPI.h>
#include <LoRa.h>
#include <MKRWAN.h>

//Duration of sleep mode in miliseconds
#define SLEEP_TIME 5000

#define DHTPIN 2
#define DHTTYPE DHT11

//Thresholds 
#define tempTH 1.0
#define humTH 1.0
//Threshold flags
int temp_FLAG, hum_FLAG = 0;

DHT dht(DHTPIN, DHTTYPE);

LoRaModem modem;

CayenneLPP lpp(51); //Max size (bytes) of message

// Set your AppEUI and AppKey 
const char* appEui = "0000000000000000"; 
const char* appKey = "FB6DDF75D139AEC02424D78F1DE3DCCD"; 

// LoRa Packet Content 
char* message = "Hello LoRa!";

void setup(void) {
  Serial.begin(9600);
  while (!Serial);

  dht.begin();
  
  // LoRa Setup
  Serial.println(F("LoRa Sender"));
  if (!LoRa.begin(868E6)) {
    Serial.println(F("Starting LoRa failed!"));
    while (1);
  } else {
    Serial.println(F("Starting LoRa Successful!"));
  }

  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  };
  Serial.print("Your module version is: ");
  Serial.println(modem.version());
  if (modem.version() != ARDUINO_FW_VERSION) {
    Serial.println("Please make sure that the latest modem firmware is installed.");
    Serial.println("To update the firmware upload the 'MKRWANFWUpdate_standalone.ino' sketch.");
  }
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());

  int connected = modem.joinOTAA(appEui, appKey);

  if (!connected) {
    Serial.println("Something went wrong; are you indoor? Move near a window and retry");
    while (1) {}
  }

  delay(2000);

  
}

void loop(void) {
  
  Serial.println("In Loop");
  //ReadSensor();
  LoRa_Packet_Sender();
  GoToSleep();
}

void ReadSensor() {
  
  temp_FLAG = 0;
  hum_FLAG = 0;
  
  //temp+humidity sensor
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  //thresholds
  if (t > tempTH) {
     temp_FLAG = 1;
  }
  if (h > humTH) {
    hum_FLAG = 1;
  }

  lpp.reset();
  lpp.addTemperature(1, t);
  lpp.addRelativeHumidity(2, h);
  
}

// LoRa Task
void LoRa_Packet_Sender() {

  int err;

  Serial.println("In LoRa_Packet_Sender");
  
  modem.beginPacket();
  modem.print("Hi");
  err = modem.endPacket(true);
  
  if (err > 0) {
    Serial.println("Message sent correctly!");
  } else {
    Serial.println("Error sending message :(");
  }

  delay(1000); //(See if can use deepsleep here or not ?)
  
  if (!modem.available()) {
    Serial.println("No downlink message received at this time.");
  }
  
  char rcv[64];
  int i = 0;
  while (modem.available()) {
    rcv[i++] = (char)modem.read();
  }
  Serial.print("Received: ");
  for (unsigned int j = 0; j < i; j++) {
    Serial.print(rcv[j] >> 4, HEX);
    Serial.print(rcv[j] & 0xF, HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // Putting LoRa Module to Sleep 
  //Serial.println(F("LoRa Going in Sleep"));
  //LoRa.sleep();
}

// Sleep Task 
void GoToSleep(){
  
  Serial.println(F("MKR WAN 1310 - Going in Sleep"));
  Serial.end();
  
  LowPower.deepSleep(5000);
  
  Serial.begin(9600); 
  while (!Serial);
}
