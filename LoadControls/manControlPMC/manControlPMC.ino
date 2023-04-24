/*
  Machine Control - Analog out Example

  This example shows how to use the Analog out channels on
  the Machine Control.
  The example sets the channels PWM period in the setup,
  then loops the channels voltage output value from 0V to 10.4V.

  The circuit:
   - Portenta H7
   - Machine Control

  This example code is in the public domain.
*/

#include <Arduino_MachineControl.h>

using namespace machinecontrol;

unsigned long previousTime = 0; //Control Law Clock
unsigned long messageClock = 0; //Message Trigger Clock

float res_divider = 0.28057; //Internal Voltage Divider
float reference = 3.3; //3.3 Volt Reference System
float vBus, iBus,pBus,rBus; //Global Variable for Voltage and Current
float busRes_divider = .2029; //External Voltage Divider
float iConversion = 0.9363; //Converting from Volts to Amps
float n = 200; //Sampling for Averaging function

void readParams(){
float vSum, iSum = 0; 
    float raw_voltage_ch0,raw_voltage_ch1; 
    float voltage_ch0,voltage_ch1; 
    for (int i = 0; i<n;i++){
      raw_voltage_ch0 = analog_in.read(0);
      float voltage_ch0 = (raw_voltage_ch0 * reference) / 65535 / res_divider / busRes_divider;
      vSum = voltage_ch0 + vSum;

      float raw_voltage_ch1 = analog_in.read(1);
      float voltage_ch1 = (raw_voltage_ch1 * reference) / 65535 / res_divider;
      iSum = voltage_ch1/0.9363 + iSum; 
    }
    
      vBus = (vSum/(float)n)-.64; 
      iBus = (iSum/(float)n)-.21; 
      rBus = vBus/iBus;
      pBus = vBus*iBus;
    if((millis()-messageClock)>1000){
        Serial.print("Bus V: ");
        Serial.println(vBus,4);
        Serial.println(raw_voltage_ch0);
        Serial.print("Bus I: ");
        Serial.println(iBus,4);
        Serial.println(raw_voltage_ch1);
        Serial.print("pwr: ");
        Serial.println(pBus,4); 
        Serial.print("Res:");
        Serial.println(rBus,4);
        messageClock = millis(); 
    }



}


void setup() {
  //analog_out.period_ms(CHANNEL, PERIOD_MILLISECONDS);
  analog_out.period_ms(0, 4);
  analog_out.period_ms(1, 4);
  analog_out.period_ms(2, 4);
  analog_out.period_ms(3, 4);
  
  Serial.begin(9600);
  Serial.println("Analog out test");
  analogReadResolution(16);
  analog_in.set0_10V();

}

//Output values which will be changed with this variable
float setPoint = 0;

void loop() {
  Serial.print("Setpoint at Start of Loop");
  Serial.println(setPoint);
  Serial.print("Serial Availability");
  Serial.println(Serial.available());
  if (Serial.available() > 0) {
    setPoint =  Serial.parseFloat();
    Serial.print("Setpoint:");
    Serial.println(setPoint); //write setpoint to serial monitor
  }
  //analog_out.write(CHANNEL, OUTPUT_VOLTAGE_VALUE);
  analog_out.write(0, setPoint);
  Serial.println("All channels set at "+String(setPoint)+"V");

  readParams();

  delay(5000);
  //Maximum output value is 10.4V

}

