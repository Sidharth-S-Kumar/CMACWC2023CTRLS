#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SD.h>
#include <SPI.h>


//Creating ADC Object 
Adafruit_ADS1115 adc; 


//Global Parameters

int adc_A0; // Voltage Sensor
int adc_A1; // Current Sensor
int adc_A2; // Pitot Tube ? 
int adc_A3; // 

float adc_V0; 
float adc_V1;
float adc_V2; 
float adc_V3; 

//Calibration Functiosn for Instrumentain 
float INA169 = 0 ; 
float V_divider = 0; 
float pitot_Tube =0; 


//State Stuff
int state =0; 



void readParameters(){
  adc_A0 = adc.readADC_SingleEnded(0);
  adc_A0 = adc.readADC_SingleEnded(1); 
  adc_A0 = adc.readADC_SingleEnded(2);

  adc_V0 = adc.computeVolts(adc_A0);
  adc_V1 = adc.computeVolts(adc_A1);
  adc_V2 = adc.computeVolts(adc_A2);


  



}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  adc.begin();

}

void loop() {
  // put your main code here, to run repeatedly:

  switch (state)
  {
  case 0: // Start Up
    
    break;

  case 1: //Constant Rotor ? Constant Power

    break;
  case 2: //Breaking
  
    break;

  case 3:

    break; 
  
  default:
    break;
  }


}