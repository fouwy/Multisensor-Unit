#include <OneWire.h>

#define MAX_SENSORS 10

OneWire  ds(8);    // 1-wire on pin 2
byte     addr[8];  // Contains the eeprom unique ID
byte memory[128];


typedef struct {
  
  int man_ID;
  int model;
  int ver_letter;
  int version;
  int serial;
    
} BasicTEDS;

int deviceAmount = 0; //REVERT THIS VALUE TO 0 AFTER TESTING

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

  byte dat[13];
  
  dat[0] = 0x00;
  dat[1] = 0x00;
  dat[2] = 0x00;
  dat[3] = 0x00;
  dat[4] = 0x00;
  dat[5] = 0x00;
  dat[6] = 0x00;
  dat[7] = 0x00;

  WriteRow(0,dat);
  ReadAllMem();
}

void loop() {
  
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
    Serial.println("No device found.");
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

void WriteReadScratchPad(byte TA1, byte TA2, byte* data)
{
  int i;
  ds.reset();
  ds.select(addr);
  ds.write(0x0F,1);  // Write ScratchPad
  ds.write(TA1,1); 
  ds.write(TA2,1); 

  for ( i = 0; i < 8; i++)
    ds.write(data[i],1);  

  ds.reset();
  ds.select(addr);    
  ds.write(0xAA);         // Read Scratchpad

  for ( i = 0; i < 13; i++)     
    data[i] = ds.read();
}

void CopyScratchPad(byte* data)
{
  ds.reset();
  ds.select(addr);
  ds.write(0x55,1);  // Copy ScratchPad
  ds.write(data[0],1); 
  ds.write(data[1],1);  // Send TA1 TA2 and ES for copy authorization
  ds.write(data[2],1); 
  delay(25); // Waiting for copy completion
  //Serial.print("Copy done!\n");
}

void ReadAllMem()
{
  int i;
  ds.reset();
  ds.select(addr);
  ds.write(0xF0,1);  // Read Memory
  ds.write(0x00,1);  //Read Offset 0000h
  ds.write(0x00,1);

  for ( i = 0; i < 128; i++) //whole mem is 144 
  {
    Serial.print(ds.read(), HEX);
    //Serial.print(" ");
  }
  Serial.println();
}

void WriteRow(byte row, byte* buffer)
{
  int i;
  if (row < 0 || row > 15) //There are 16 row of 8 bytes in the main memory
    return;                //The remaining are for the 64 bits register page

  WriteReadScratchPad(row*8, 0x00, buffer);

  /*  Print result of the ReadScratchPad
  for ( i = 0; i < 13; i++) 
  {
  Serial.print(buffer[i], HEX);
  Serial.print(" ");
  }
  */
  CopyScratchPad(buffer);
  
}
