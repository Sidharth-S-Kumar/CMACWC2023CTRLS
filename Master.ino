//Include Arduino Wire library for I2C
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

//Include SD & SPI library for SD datalogging
#include <SD.h>
#include <SPI.h>

//=================================================================================================

//Create I2C Device Objects
#define SLAVE_ADDR 9  //Define Slave I2C Address
#define adcWindPin 0 //ADC DP Sensor Measurement Pin
#define adcPotPin 2 //ADC Potentiometer Measurement Pin
Adafruit_ADS1115 adc1;  // Construct an ads1115 
const int chipSelect = BUILTIN_SDCARD;

int analogPin = 3;
int val = 0; 

//=================================================================================================

void setup() {
  //Initialize I2C communications as Master
  Wire2.begin();
  adc1.begin();

  //Setup serial monitor
  Serial.begin(9600);  
  Serial1.begin(9600);

  Serial.println("I2C Master Demonstration");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");       
}

//=================================================================================================

void loop() {
  delay(500);
  readADCParams();

    /*
    //Write a character to the Slave
    Wire2.beginTransmission(SLAVE_ADDR);
    Wire2.write("");
    Wire2.write(val);
    Wire2.endTransmission();
    */
}

void readADCParams(){
  //ReadADCAnalogPins
  int WindBitADC = adc1.readADC_SingleEnded(adcWindPin);
  int PotBitADC = adc1.readADC_SingleEnded(adcPotPin); 
  float _WindADC = adc1.computeVolts(WindBitADC);
  float _PotADC = adc1.computeVolts(PotBitADC);
  val = map(analogRead(analogPin), 0, 1023, 255, 1);
  float Windspeed = (22.36067977*_WindADC)-21.48255106;
  //Serial.print("Wind Sensor Voltage: "); Serial.println(_WindADC);
  Serial1.print("Windspeed: "); Serial1.print(Windspeed); Serial1.println(" m/s");
  Serial.print("Windspeed: "); Serial.print(Windspeed); Serial.println(" m/s");
//  Serial.print("Wind Sensor BITS: "); Serial.println(WindBitADC);
  //Serial.print("Potentiometer Voltage: "); Serial.println(_PotADC);

//=================================================================================================
      //if button wired to digital pin 7 is pressed, print "button 7 pressed - data saved!" to serial
        
        if (digitalRead(7) == HIGH) {
        Serial.println("button 7 pressed - data saved!");

      // open the file named datalog.txt on the sd card
          File dataFile = SD.open("datalog.txt", FILE_WRITE);

          // if the file is available, write the contents of datastring to it
          if (dataFile) {
          dataFile.print("Pot Reading"); dataFile.print(val); dataFile.print(", ");
          dataFile.print("Windspeed: "); dataFile.print(Windspeed); dataFile.println(" m/s");
          dataFile.close();
          }  
          // if the file isn't open, pop up an error:
          else {
          Serial.println("error opening datalog.txt");
        }   
  }

//=================================================================================================

//if button wired to digital pin 8 is pressed, print "button 8 pressed - load" to serial

        if (digitalRead(8) == HIGH) {
        Serial.println("button 8 pressed - load");

      //open up datalog2.txt and then print all of its contents  
        File dataFile = SD.open("datalog.txt");
        if(dataFile) {
          Serial.println("datalog:");
          while (dataFile.available()) {
              Serial.write(dataFile.read());
           }
      // close the file:
         dataFile.close();
    
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening datalog.txt");
  }
  
}

}

