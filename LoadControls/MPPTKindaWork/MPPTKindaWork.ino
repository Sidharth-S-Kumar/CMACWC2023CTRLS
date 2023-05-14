#include <Arduino_MachineControl.h>
using namespace machinecontrol;



unsigned long previousClock = 0; //Control Law Clock
unsigned long messageClock = 0; //Message Trigger Clock
unsigned long depressionClock = 0; 
unsigned long backfeedClock = 0;
int state =0; 
//ControlLaw Params 
unsigned long currentClock; 
float err;
float errInt; 
float errMin = -25; 
float errMax = 25; 
float errLast = 0;
float outMax = 10; 
float outMin = 2.3;
float kP = 8; 
float kI = 0;
float kD = 0;
float outPut = 0; 
float setPoint=0;
//MPPT Params
float dV,dI; 
float vBus_Last = 0; 
float iBus_Last = 0; 
float delta = .005;
float MPPT_Targ = 2.65; 
float min_Gate = 2.65; 
float hold;
float absHold; 

//ReadParameters
//float res_divider = 0.28057; //Internal Voltage Divider
//float reference = 3.3; //3.3 Volt Reference System
float vBus, iBus,pBus,rBus; //Global Variable for Voltage and Current
//float busRes_divider = .2029; //External Voltage Divider
//float busRes_dividerOffset = -.0019;
//float vOffset = 0;
//float iConversion = 0.9363; //Converting from Volts to Amps
//float iOffset = 0;
float n = 500; //Sampling for Averaging function
float vK = 0.00086801; //voltage calibration constant
float iK = 0.00018084; //current calibration constant
float vB = -0.1718; //voltage calibration offset
float iB = -0.0386; //current calibration offset




void readParams(){
float vSum, iSum = 0; 
    float raw_voltage_ch0,raw_voltage_ch1; 
    float voltage_ch0,voltage_ch1; 

    for (int i = 0; i<n;i++){
      raw_voltage_ch0 = analog_in.read(0);
      voltage_ch0 = (raw_voltage_ch0 * vK) + vB;
      vSum += voltage_ch0;

      raw_voltage_ch1 = analog_in.read(1);
      voltage_ch1 = (raw_voltage_ch1 * iK) + iB;
      iSum += voltage_ch1; 
    }
    
      vBus = (vSum/(float)n); 
      iBus = (iSum/(float)n); 
      rBus = vBus/iBus;
      pBus = vBus*iBus;
}

/*void controlLaw(float sP,float mV){
  currentClock = millis(); 
  err = sP - mV; 
  Serial.print()
  //err = constrain(err,errMin,errMax);
  float dt = (float)((currentClock-previousClock)*0.001); 
   
  //Porportional Component; 
  float P = kP*err; 
  Serial.println(P);
  //Integral Component; 
  errInt += err*dt; 
  float I = kI*errInt; 
  //Derivative Component
  float de = err-errLast; 
  errLast = err; 
  float D = kD*(de/dt);

  float PID = P+I+D; 

  PID = constrain(PID,outMin,outMax);

   outPut = PID; 

  
}*/



void MPPT(){
 dV = (vBus - vBus_Last);
 dI = (iBus - iBus_Last); 
 /*Serial.print("DV:");
 Serial.println(dV);
 Serial.print("DI:");
 Serial.println(dI);*/
 
if((millis()-depressionClock)>850){
if(dV=0){
    if(dI=0){
      //do Nothing; 
      Serial.println("Do Nothing");
    }
    else{
      if(dI>0){
        MPPT_Targ -=delta; 
        Serial.println("decrease");
        
      }
      else{
        MPPT_Targ +=delta; 
        Serial.println("increase");
      }
    }
  }
else{
   hold = vBus*dI+iBus*dV; 
  Serial.println(hold,9); 
   absHold = abs(hold); 
  if(absHold=0){
    //doNothing
    Serial.println("Do Nothing");
  }
  else{
    if(hold>0){
      if(dV>0){
        MPPT_Targ -=delta;
        Serial.println("decrease");

      }
      else{
        MPPT_Targ +=delta;
        Serial.println("increase");

      }
    }
    else{
      if(dV>0){
        MPPT_Targ +=delta;
        Serial.println("increase");

      }
      else {
        MPPT_Targ -=delta;
        Serial.println("decrease");

      }
    }
  }
  depressionClock = millis(); 
  }
}

outPut=constrain(MPPT_Targ,min_Gate,10.4); 
if(outPut>min_Gate){
  min_Gate = outPut; 
}

vBus_Last = vBus; 
iBus_Last = iBus; 


}

void writeParams(){
  if((millis()-messageClock)>1000){
    messageClock = millis();

    //Analog Inputs
    Serial.print("Voltage: ");
    Serial.println(vBus,3);
    Serial.print("Current: ");
    Serial.println(iBus,3);
    Serial.print("Power: ");
    Serial.println(pBus,3);
    Serial.print("Resistance: ");
    Serial.println(rBus,3);
    Serial.print("Gate Voltage:");
    Serial.println(outPut);
    Serial.print("dV:");
    Serial.println(dV,9);
    Serial.print("dI:");
    Serial.println(dI,9);
    Serial.print("dp: ");
    Serial.println(hold,9);
    Serial.print("State:");
    Serial.println(state);
  }
}


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);


  analog_out.period_ms(0, 4);
  analog_out.period_ms(1, 4);
  analog_out.period_ms(2, 4);
  analog_out.period_ms(3, 4);
  analogReadResolution(16);
  analog_in.set0_10V();
  digital_outputs.setLatch();
  
  delay(2500);
  Serial.println("Testing Digital");
  digital_outputs.setAll(1);

  delay(2500);

  digital_outputs.setAll(0);

  Serial.println("Testing Analog");
  analog_out.write(0, 10);

  delay(2500);
  

  analog_out.write(0, 0);

  delay(2500); 

  
}

void loop() {
  // put your main code here, to run repeatedly:

  readParams();
  //MPPT();
  writeParams();
  //analog_out.write(0,outPut);

  switch(state) {
  case 0:
    if(vBus>15){
      state = 1; 
      analog_out.write(0,2.65);
      Serial.println("Out of Start Up State");
    }
    //Serial.println("Start Up State");
    break;
  case 1: // MPPT
    MPPT();
    analog_out.write(0,outPut);
    if(vBus<5){
      state = 3; 
      Serial.println("BackFeed");
      backfeedClock = millis();
    }
    else if(pBus>38){
      state = 2;
      Serial.println("Cut Out");
    }
 
    break;
  case 2: //Cut Out 
    outPut = 10; 
    analog_out.write(0,outPut);
    if(vBus<5){
      state = 3; 
      Serial.println("BackFeed");
      backfeedClock = millis();
    }
    
    break; 
  case 3: //BackFeed 
    analog_out.write(0,0);
    digital_outputs.set(0, HIGH);
    if((millis()-backfeedClock)>5000){
       digital_outputs.set(0, LOW);
       delay(1500);
      if(vBus>5){
        min_Gate = 2.65; 
        state = 1;
      }
    }
  
   break;
  }
}


