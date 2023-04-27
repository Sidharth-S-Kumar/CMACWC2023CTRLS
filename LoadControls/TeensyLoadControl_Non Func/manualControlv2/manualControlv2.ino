#include<Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 adc1;  // Construct an ads1115 
Adafruit_MCP4725 dac1;

//Globals
float vLocal, vADC; 
float iLocal, iADC; 
float resLocal, resADC;

typedef struct{
  float v, i, r; 
} ioRead; 

ioRead teensyIO; 
ioRead adcIO; 

float voltageScaling = 15.78; 
float currentVOffset = 0.0917;
float currentVscaling = 0.9362; 



//IO Functions 
ioRead readLocalParams(){
  //ReadTeesnyAnalogPins
  int voltageBitTeensy = analogRead(A0);
  int currentBitTeensy = analogRead(A1);
  //Convert 10Bit to V & I 
  vLocal = ((voltageBitTeensy/1023.00)*3.30*voltageScaling); 
  iLocal = ((((currentBitTeensy/1023.00)*3.30)-currentVOffset)/currentVscaling);
  resLocal  = vLocal/iLocal; 
  teensyIO = {vLocal,iLocal,resLocal};
  Serial.print(teensyIO.v);
  Serial.print(","); 
 
  Serial.print(teensyIO.i);
  Serial.print(teensyIO.r); 
  return teensyIO; 
}

ioRead readADCParams(){
  //ReadADCAnalogPins
  int voltageBitADC = adc1.readADC_SingleEnded(0);
  int currentBitADC = adc1.readADC_SingleEnded(1);
  //Convert 16Bit to V & I 
  vADC = ((voltageBitADC/65536.00)*5.00*voltageScaling); 
  iADC = ((((currentBitADC/65536.00)*5.00)-currentVOffset)/currentVscaling);
  resADC  = vADC/iADC; 
  adcIO = {vADC,iADC,resADC};
  //Serial.println(adcIO);
  return adcIO; 
}





void setup() {
  // put your setup code here, to run once:
  adc1.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

}
