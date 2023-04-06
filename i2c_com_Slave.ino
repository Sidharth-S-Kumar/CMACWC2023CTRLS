// Include Arduino Wire library for I2C
#include <Wire.h>

//Include SD & SPI library for SD datalogging
#include <SD.h>
#include <SPI.h>
 
//=================================================================================================

// Define Slave I2C Address
#define SLAVE_ADDR 9

int LED = 8; //define LED pin
int rd; //variable for received data
int br; //variable for blink rate
const int chipSelect = BUILTIN_SDCARD;
char fileName[13] = "xx.CSV";
int i = 0;

//=================================================================================================

void setup() {
  // Initialize I2C communications as Slave
  Wire2.begin(SLAVE_ADDR);

  // Function to run when data received from master
  Wire2.onReceive(receiveEvent);

  // Setup Serial Monitor 
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("I2C Slave Demonstration");
  Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}
 
//=================================================================================================

void receiveEvent() {

  rd = Wire2.read();//read one character from the I2C

/* Use to see the incoming value from the Master

  //Print to Serial Monitor
  Serial.println("Receive event");
  Serial.println(rd);//print value of incoming data

*/
}

//=================================================================================================

void loop() {
  // make a string for assembling the data to log:
  String dataString = "";

//=================================================================================================
      //if button wired to digital pin 7 is pressed, print "button 7 pressed - data saved!" to serial
        
        if (digitalRead(7) == HIGH) {
        Serial.println("button 7 pressed - data saved!");
        
      //read the potentiometer's value. then use this value as the new value of dataString
          //int pot = analogRead(A0);
          dataString = String(rd);

      // open the file named datalog.txt on the sd card
          File dataFile = SD.open("datalog.txt", FILE_WRITE);

          // if the file is available, write the contents of datastring to it
          if (dataFile) {
          dataFile.println(dataString);
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
  delay(150);
}




