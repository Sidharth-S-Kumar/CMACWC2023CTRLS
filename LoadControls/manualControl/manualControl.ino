
#include<Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 adc;  // Construct an ads1115 
Adafruit_MCP4725 dac;


int voltageBit1;
int currentBit1;
int voltageBit2;
int currentBit2;

float realVoltage1;
float realCurrent1;
float realVoltage2;
float realCurrent2;

float voltageScaling = 15.78; 
float currentVOffset = 0.0917;
float currentVscaling = 0.9362; 

void readADC(){
  

}
void readTeensy(){


  
}
}
void setup() {
  Serial.begin(9600);
  Serial.println("Initializing");
  dac1.begin(0x62);
  ads1115.begin();
  Serial.println("Initialized");
}

void loop() {
   int bitValue = Serial.parseInt();
   int sensorValue = analogRead(A1);
     if (Serial.available() > 0) {
            Serial.print("New Voltage Out:");
            Serial.println(bitValue);
            dac1.setVoltage(bitValue,false); 
            delay(2000);
          }
    Serial.println(sensorValue);

}
