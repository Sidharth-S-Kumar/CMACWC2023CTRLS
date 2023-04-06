// '#--' (35) is for setting stroke of actuator
//test: 90:1 takes 7 seconds to go from 1 stroke to 99 stroke'
//open-loop testing

//R = 
//U =
//stroke1 = 
//stroke2 = 
//stroke3 =
//stroke4 = 


#include <Servo.h> // using servo library
Servo actuator; // device name is actuator
float stroke_percent;
int actuator_Pin = 20
; // outputting signal to actuator
int manual_Stroke; // manually set70ting stroke


void setup() {
  Serial.begin(9600);
  actuator.attach(actuator_Pin);
  dac1.
}

void set_Stroke(float stroke_percent) //setting stroke from 0-100 (no stroke to full stroke)
{
  if ( stroke_percent >= 1.0 && stroke_percent <= 99.0 ) // range of motion
  {
    float micro_sec = 1000 + stroke_percent * ( 2000 - 1000 ) / 100.0 ;
    actuator.writeMicroseconds(micro_sec);
    
  }
}

void loop() {
  // manual code
   manual_Stroke = Serial.parseInt();
     if (Serial.available() > 0) {
            Serial.print("Manual actuator control: Stroke of ");
            Serial.println(manual_Stroke);
            set_Stroke(manual_Stroke); //input #set_stroke in serial monitor
            delay(2000);
          }
}
