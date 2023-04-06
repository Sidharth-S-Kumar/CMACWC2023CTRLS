#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac1;
int dacOutput; 
unsigned long previousTime =millis(); 


void setup() {
  Serial.begin(9600);
  Serial.println("Hello!");
  dac1.begin(0x62);
}

void loop() {
  if ((millis()-previousTime)>=10000){
      Serial.println("this is working");
      Serial.println(analogRead(A3));
      previousTime = millis(); 
  }
     if (Serial.available() > 0) {
      dacOutput = Serial.parseInt(); 
      if((dacOutput>=0) && (dacOutput<=4095)){
            Serial.print("Manual DAC control: Bits:");
            Serial.println(dacOutput);
            dac1.setVoltage(dacOutput,false);
          }
    }
}