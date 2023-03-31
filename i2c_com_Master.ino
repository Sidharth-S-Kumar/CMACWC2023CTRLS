//Include Arduino Wire library for I2C
#include <Wire.h>

//Include SD & SPI library for SD datalogging
#include <SD.h>
#include <SPI.h>

//=================================================================================================

//Define Slave I2C Address
#define SLAVE_ADDR 9
//Define Slave answer size
//#define ANSWERSIZE 5
int analogPin = 0;
int val = 0; 

//=================================================================================================

void setup() {
  //Initialize I2C communications as Master
  Wire.begin();
  //Setup serial monitor
  Serial.begin(9600);
  Serial.println("I2C Master Demonstration");
}

//=================================================================================================

void loop() {
  delay(500);
  //Read pot value
  // Map to rang of  1-255 for flash rate
    val = map(analogRead(analogPin), 0, 1023, 255, 1);

    //Write a character to the Slave
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(val);
    Wire.endTransmission();
}

