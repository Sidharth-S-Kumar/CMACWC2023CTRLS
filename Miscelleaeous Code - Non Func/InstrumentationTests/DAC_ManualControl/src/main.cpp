#include <Arduino.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac1; 
uint16_t manualDAC; 

void setup() {
    Serial.begin(9600);
    dac1.begin(0x62);
}

void loop() {
   manualDAC = Serial.parseInt();
     if ((Serial.available() > 0)&&((manualDAC>=0)&&(manualDAC<=4095))) {
            
            Serial.print("Manual DAC: ");
            Serial.println((float)(manualDAC/4095));
            dac1.setVoltage(manualDAC,false);
            delay(2000);
          }
  
}