
  #include <Adafruit_MCP4725.h>
  #include <Adafruit_ADS1X15.h>
  #include <SD.h>

  #include <parameterRead.h>


// Pin Define 

#define adcVoltagePin 0 //ADC Analog Voltage Measurement Pin 
#define adcCurrentPin 1 //ADC Analog Current Measurement Pin 
#define teensyVoltagePin A0 //Analog Voltage Measurment Pin
#define teensyCurrentPin A1 //Analog Current Measurment Pin
#define togglePin 1 //State Change Pin

//Create I2C Device Objects 
Adafruit_ADS1115 adc1;  // Construct an ads1115 
Adafruit_MCP4725 dac1; // Construct an MCP4725

//General Clock Globals 
unsigned long clock = millis(); //Clock Reference 
unsigned long previousTime = 0; //Control Law Clock
unsigned long messageClock = 0; //Message Trigger Clock

//IO Globals
float vLocal, vADC; //Voltage by Source 
float iLocal, iADC;  //Current by Source
float resLocal, resADC; //Resistance by source 
float voltageScaling = 15.78;  //Scaling Factor for Voltage
float currentVOffset = 0.0917; //Offset on Current Measurement
float currentVscaling = 0.9362; //Scaling factor from voltage to current. 

//Type Defining IORead as a Struct
typedef struct{
  float v, i, r;  
} ioRead; // Input Output Struct for holding measurement parameters

ioRead teensyIO; 
ioRead adcIO; 

//IO Functions 
void readLocalParams(){
  //ReadTeesnyAnalogPins
  int voltageBitTeensy = analogRead(teensyVoltagePin);
  int currentBitTeensy = analogRead(teensyCurrentPin);
  //Convert 10Bit to V & I 
  vLocal = (float)((voltageBitTeensy/1023.00)*3.30*voltageScaling); 
  iLocal = (float)((((currentBitTeensy/1023.00)*3.30)-currentVOffset)/currentVscaling);
  resLocal  = vLocal/iLocal; 
  teensyIO = {vLocal,iLocal,resLocal};
  Serial.print(teensyIO.v);
  Serial.print(","); 
  Serial.print(teensyIO.i);
  Serial.print(","); 
  Serial.println(teensyIO.r); 
}

void readADCParams(){
  //ReadADCAnalogPins
  int voltageBitADC = adc1.readADC_SingleEnded(adcVoltagePin);
  int currentBitADC = adc1.readADC_SingleEnded(adcCurrentPin); 
  //Convert 16Bit to V & I 
  vADC = (float)((voltageBitADC/65536.00)*5.00*voltageScaling); 
  iADC = (float)((((currentBitADC/65536.00)*5.00)-currentVOffset)/currentVscaling);
  resADC  = vADC/iADC; 
  adcIO = {vADC,iADC,resADC};
  //Serial.println(adcIO);

}
//Control Laws

///Globals
float setPoint = 0;
float err;
float errPrev = 0;
float errSum = 0;
int state = 0; 
float modePrevious; 
int dacOutput = 0; 
float measuredVar; 


typedef struct { 
  float Kp,Kd,Ki,errMax,errMin,inverter;
} ctrlValues; 

  ctrlValues constPWR_Values = {0,0,0,10,-10,0};
  ctrlValues constRES_Values = {0,0,0,10,-10,1};
  ctrlValues constVOL_Values = {0,0,0,2,-2,0};
  ctrlValues constCUR_Values = {0,0,0,0.2,-0.2,1};



void ctrlLaw(float sP,float mV,ctrlValues value,int state_val) {
  if(sP!=0){
  if ( state_val != modePrevious)
  {
   err = 0; 
   errPrev = 0; 
   errSum = 0;
   modePrevious = state_val; 
  }
  
  float dt = (float)(clock-previousTime)/1000.00; //time increment dt
  err = sP - mV;
  err = constrain(err,value.errMax,value.errMin); 

  //Porportional Control
  float P = value.Kp*err; 

  //Derivative Control 
  float errDer = (err-errPrev)/dt; 
  float D = value.Kd*errDer; 
  errPrev = err;

  //Integral Control
  errSum += err; 
  float I = value.Ki*errSum; 
  //Control Output
  if(value.inverter){
    dacOutput = 4096-(int)(P+I+D)*(4096/5);  
  }
  else {
  dacOutput = (int)(P+I+D)*(4096/5);
  } 
  dacOutput = constrain(dacOutput, 0, 4096); 
  dac1.setVoltage(dacOutput, false);
  }
  dac1.setVoltage(0,false);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  adc1.begin();
  dac1.begin(0x62);
  pinMode(togglePin,INPUT);

}

void loop() {

  if (Serial.available() > 0) {
    setPoint =  Serial.parseFloat();
    Serial.print("Setpoint:");
    Serial.println(setPoint); //write setpoint to serial monitor
  }

  readADCParams(); 
  readLocalParams(); 

  int toggleState = digitalRead(togglePin);

  if(toggleState == HIGH) {
    if (state == 5){
      state = 0;
    } 
    else {
      state ++; 
    }
    
  }

  switch (state)
  {
  case 0 : //Off State
    if ((millis()-messageClock)>=50000){
     Serial.println("Sys Off");
     Serial.println("Select Mode and Enter Set Point");
     messageClock=millis(); 
     dac1.setVoltage(0, false);
    }
    break; 
  
  case 1 : //Constant Power
    measuredVar = teensyIO.i * teensyIO.v; 
    ctrlLaw(setPoint,measuredVar,constPWR_Values,state);
    if ((millis()-messageClock)>=50000){
    Serial.print("Constant Power Mode:");
    Serial.print(setPoint);
    Serial.println(" Watts");
    Serial.print("Current Power:");
    Serial.println(measuredVar); 
    }
    break;

  case 2 : //Constant Resistance
    measuredVar = teensyIO.v/teensyIO.i; 
    ctrlLaw(setPoint,measuredVar,constRES_Values,state);
    if ((millis()-messageClock)>=50000){
    Serial.print("Constant Resistance Mode:");
    Serial.print(setPoint);
    Serial.println(" Ohms");
    Serial.print("Current Resistance:");
    Serial.println(measuredVar); 
    }
    break;
  
  case 3 : //Constant Voltage
      measuredVar = teensyIO.v; 
    ctrlLaw(setPoint,measuredVar,constVOL_Values,state);
    if ((millis()-messageClock)>=50000){
    Serial.print("Constant Voltage Mode:");
    Serial.print(setPoint);
    Serial.println(" Volts");
    Serial.print("Current Voltage:");
    Serial.println(measuredVar); 
    }
    break;  
    
    case 4 : //Constant Current
    measuredVar = teensyIO.i; 
    ctrlLaw(setPoint,measuredVar,constCUR_Values,state);
    if ((millis()-messageClock)>=50000){
      Serial.print("Constant Current Mode:");
      Serial.print(setPoint);
      Serial.println(" Amps");
      Serial.print("Current Current:");
      Serial.println(measuredVar); 
    }
    break;

    case 5: //P&O Maybe 

    break; 

  }




}
