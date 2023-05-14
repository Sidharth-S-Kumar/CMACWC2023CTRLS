//TURBINE SIDE TEENSY
//Objectives:
//State 1: Set pitch to cut-in pitch until sufficient power is reached
//State 2: Optimize pitch until rated power is reached, then maintain rated power
//State 3: When button is sensed, feather blades only in state 2. Return to previous stroke. (Emergency stop)
//State 4: When current is 0, stroke goes to zero only in state 2. Return to previous stroke. (No-Load)
//State 4 alternative: When voltage reached a maximum threshhold, stroke goes to zero only in state 2




//Constants
const int currentPin = 16; //pin for current sensor
const int voltagePin = 18; //pin for voltage sensor
const int buttonPin = 22; //pin for button
unsigned long previousTime = 0; //initialize code to zero
const int n = 50; //Number of samples 
unsigned long nextprint; 

//servo global variables

float stroke_percent; //stroke command to actuator
const int actuator_Pin = 20; // outputting signal to actuator
float optStroke = 82.0; //optimal stroke
float cutInStroke = 69.0; //cut-in stroke
float stroke_curr = optStroke; // current stroke for PI control
float fullFeathStroke = 7.0; //full feather stroke

//power sensor global variables
float voltSense; //analog reading for voltage
float currSense; //analog reading for current
float readVoltage; //bus voltage reading
float readCurrent; //line current reading
float avgVoltage; //average voltage
float avgCurrent; //average current
float measPower; //average power
float currOffset = 0.004848368; //determined from calibration (turbine side INA 169)
float voltOffset = 0.333946348; //determined from calibration (R1 = 10kOhm, R2 = 150kOhm)

//control law global variables
float sig = 1.6535; //Pole for critically damped approximation
float Kstatic = 0.514; //Static gain (W/%)
float err; //Error
float Kpi = 1.0/Kstatic; //Proportional gain (%/W)
float Kd = 0.0; //Derivative gain (%/W/s)
float errPrev = 0.0; //initialize previous error
float errInt = 0.0; //initialize error integral
float errMax = 5.0; //error constraint for integral windup
float errMin = -errMax; //error constraint for integral windup
float ratedPower = 25.8; //rated power (W)
float cutInPower = 0.9; //cut in power
float currThresh = 0.03; //low current threshold
float lowVoltThresh = 2.7; //limit of INA169
float highVoltThresh = 30.0; //high voltage threshold
int buttonState; //state of button
unsigned long currentTime;
unsigned long delayTime = 10000.0;


//state variables
int state = 1; //initialize state for startup


void setup() {

  Serial.begin(9600);
  actuator.attach(actuator_Pin); 


}
//Reading the sensor
void readParameters() {

 //compute voltage (V)
voltSense = analogRead(voltagePin); //analog input to teensy
readVoltage = 16.0*voltSense*(3.3/1023.0) + voltOffset; //analog output for voltage (10 bit teensy takes 3V)
 
 //compute current (A)
if (readVoltage >= 2.3) {
currSense = analogRead(currentPin); //analog reading of current (1V = 1A)
readCurrent = (currSense*3.3/1023.0);
}

else {
  readCurrent = 0; //negligible current at <2.3V
}

}

void avgPowerCalculator() {
  //initialize 1st variables
  float sumVoltage = 0;
  float sumCurrent = 0;

  //compute average and call that the power measured
  for (int i = 1; i <= n; i = i + 1) {
    readParameters();
    sumVoltage = readVoltage + sumVoltage; //adding up voltages
    sumCurrent = readCurrent + sumCurrent; //adding up currents

    if (i == n) {
      avgVoltage = sumVoltage/(float)n; //taking average voltage
      avgCurrent = sumCurrent/(float)n; //taking average current
      measPower = avgVoltage*avgCurrent; //voltage times current is power
      }
    }
  }

void controlLaw() {
  currentTime = millis(); //counting time
  float dt = (float)(currentTime - previousTime)/1000.0; //time increment dt
  err = ratedPower-measPower; //computing the error
  err = constrain(err, errMin, errMax); //anti-integral windup 
        
  //Proportional
  float comPI = Kpi*err; //proportional component

  //Derivative
  float errDer = (err-errPrev)/dt; //Derivative of erro
  float comD = Kd*errDer; //derivative component
  errPrev = err; //resetting error
  
  //sum of all command components
  float comStroke = comPI + comD; //+ comI;

  stroke_percent = stroke_curr + comStroke*dt; //first option
  
  stroke_curr = stroke_percent; //resetting current stroke
  previousTime = currentTime; //resetting time
}

void set_Stroke(float stroke_percent) //setting stroke from 0-100 (no stroke to full stroke)
{
  if ( stroke_percent >= fullFeathStroke && stroke_percent <= optStroke ) //Set bounds for full range of motion
  {
  float micro_sec = 1000 + stroke_percent * ( 2000 - 1000 ) / 100.0; //range from 1ms to 2ms
    actuator.writeMicroseconds(micro_sec); //commands stroke to certain pulsewidth
    
  }
}

void printVar() {
  if (millis() > nextprint) {
    
Serial.print("Stroke command: ");
Serial.print(stroke_percent); //reading stroke command
Serial.print(" Power is: ");
Serial.print(measPower); //reading measured power
Serial.print(", Voltage is: ");
Serial.print(readVoltage); //reading voltage
Serial.print(", state is: ");
Serial.print(state);
Serial.print(", button state is: ");
Serial.print(buttonState);
Serial.print(", error is: ");
Serial.print(err);
Serial.print(", Current is: ");
Serial.println(readCurrent); //reading current

nextprint = millis()+1000.0; //print every second
  } 
}

void loop() {
  printVar();
  avgPowerCalculator(); //reads data from power sensors
  buttonState = digitalRead(buttonPin); //reads data from button
  switch (state) {
    case 1: //maintain cut in stroke until cut in power is reached
    stroke_percent = cutInStroke;
    set_Stroke(stroke_percent);
    
    if (measPower >= cutInPower) {
      state = 2; //cut in power is exceeded, move to state 2
      break;
    }
    break;

    case 2: //rated power is not reached, maintain optimal stroke

    if (measPower < ratedPower) {
      stroke_percent = optStroke;
      set_Stroke(stroke_percent); //rated power is not reached, maintain optimal stroke
    }
    if (measPower >= ratedPower) {
      controlLaw(); //PID controller to tend towards rated power
      set_Stroke(stroke_percent); //input command to regulate power
    }

    if (buttonState == LOW) {
      currentTime = millis();
      state = 3; //E-stop state; button is pressed
      break;
    }

    if ((avgCurrent <= currThresh) && (avgVoltage >= highVoltThresh)) {
      currentTime = millis();
      state = 3; //Load is disconnected, move to state 3
      break;
    }
    break;

    case 3: //Brake state 
    stroke_percent = fullFeathStroke;
    set_Stroke(stroke_percent);
    if ((buttonState == HIGH) && (millis() > (currentTime + delayTime))) {
      state = 1; //cut back in
      break;
    }
    break;
  }
}
