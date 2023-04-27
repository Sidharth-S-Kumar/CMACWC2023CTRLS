#include <TimerThree.h>
#include <TimerOne.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// Declare an instance of INA219 IV sensor
Adafruit_INA219 ina219;

// Switch control variable
unsigned int mode = 0;

// IVP sensing stuff
float avg_I = 0.0;
float avg_V = 0.0;
float P_out = 0.0;
float P_rated = 32.0;
float P_rateH = 32.0;
float P_rateL = 29.0;
float P_old = 0.0;

// RPM sensor stuff
const int rpmPin = 2;
volatile byte quarter_revs = 0;
float rpm = 0.0;
unsigned long timeold = 0;
unsigned int counter = 0;

// Assign BB stuff
float Comm = 20.0;
const int buckenable_pin = 32;
const int buckpwm_pin = 5;
const int boostenable_pin = 30;
const int boostpwm_pin = 12;

// MPPT stuff
float K_mppt = -0.028;
float a = 748.43;
float b = 0.3513;
float rpm_opt;

// RPC stuff
float K_rpc = 1.0;
float K_d = 0.1;

// E-Stop stuff
const int NCswitch = 39;          // Assign switch pin
const int eStopRelay = 41;        // Assign E-Stop pin
bool brake = 0;                   // 0 = brake off, 1 = brake on

// ===========================================================================
// Setup loop
// ===========================================================================
void setup() {
  // Initialize serial port
  Serial.begin(115200);

  // Initialize E-Stop pins
  pinMode(eStopRelay, OUTPUT);
  pinMode(NCswitch, INPUT);
  digitalWrite(eStopRelay, HIGH);

  // Initialize IV sensor
  ina219.begin();

  // Initialize buck driver
  pinMode(buckenable_pin, OUTPUT);
  pinMode(buckpwm_pin, OUTPUT);
  digitalWrite(buckenable_pin, HIGH);
  Timer3.initialize(20);  // 20 ms or 50 kHz

  // Initialize boost driver
  pinMode(boostenable_pin, OUTPUT);
  pinMode(boostpwm_pin, OUTPUT);
  digitalWrite(boostenable_pin, HIGH);
  Timer1.initialize(20);

  // Initialize interrupt pin for RPM sensor
  pinMode(rpmPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rpmPin), magnet, RISING);

  // Initialize BB controls
  bbControl();
}

// ===========================================================================
// FUNCTION DECLARATIONS
// ===========================================================================

// Function for RPM interrupt: Count number of positive rotor pole detections
void magnet() {
  quarter_revs ++;
}

// Function gets RPM ---------------------------------------------------------
void rpmCalcs() {
  // Initialize local variables
  float rpm_old = rpm;

  // Check number of calls to rpm loop since rpm was last calculated
  counter ++;

  if ((quarter_revs >= 20) || (counter >= 10)) {
    rpm = 15.0E6 * (float)quarter_revs / ((float)(micros() - timeold));
    timeold = micros();
    quarter_revs = 0;
    counter = 0;
  }

  // Use old rpm reading if sensor reads garbage or micros() rolls over
  if ((rpm > 10000.0) || (rpm < 0.0)) {
    Serial.println("OOOPS!");
    rpm = rpm_old;
  }

}

// Function reads IV sensor & calculates power -------------------------------
void ivpCalcs() {
  // Initialize local variables
  float read_I;
  float read_V;
  float sum_V = 0.0;
  float sum_I = 0.0;
  unsigned int n = 200; // Number of voltage & current readings used in average calcs
  float R = 0.1 / 0.033; // Multiplier for 0.04 Ohm replacement sense resistor

  // Save old power value
  P_old = P_out;

  // Get I & V from sensors
  for (int i = 0; i < n; i++) {
    read_V = ina219.getBusVoltage_V();
    read_I = ina219.getCurrent_mA();

    // Keep a running sum
    sum_V = sum_V + read_V;
    sum_I = sum_I + read_I;
  }

  // Calculate I, V & P
  avg_V = sum_V / (float)n;
  avg_I = sum_I * R / (float)n / 1000.0;
  P_out = avg_V * avg_I;

}

// Function for buck-boost controls ------------------------------------------
void bbControl() {
  // Local variables
  float BUCK;
  float BOOST;

  if (Comm > 0.0) {                     // Boost if 0 < Comm < 100
    // Limits boosting
    if (Comm > 70.0) {
      Comm = 70.0;
    }

    BOOST = round((1.0 - (Comm / 100.0)) * 1023.0);
    Timer3.pwm(buckpwm_pin, 1023);
    Timer1.pwm(boostpwm_pin, BOOST);
  }
  else if (Comm < 0.0) {                // Buck if -100 < Comm < 0
    // Limits bucking
    if (Comm < -100.0) {
      Comm = -100.0;
    }

    BUCK = round((1.0 - (abs(Comm) / 100.0)) * 1023.0);
    Timer3.pwm(buckpwm_pin, BUCK);
    Timer1.pwm(boostpwm_pin, 1023);
  }
  else {                              // Do nothing if Comm = 0
    Timer3.pwm(buckpwm_pin, 1023);
    Timer1.pwm(boostpwm_pin, 1023);
  }
}

// Function for checking when to enter braking mode --------------------------
void brakeCheck_On() {
  // Check if switch is open (HIGH signal) or if load disconnected
  brake = ((digitalRead(NCswitch)) || ((avg_I < 0.1) && (rpm > 2000.0)));
}

// Function for printing run conditions --------------------------------------
void turbineCond() {
  Serial.print("Comm: ");
  Serial.print(Comm);
  Serial.print("; rpm: ");
  Serial.print(rpm);
  Serial.print("; avg_V: ");
  Serial.print(avg_V);
  Serial.print("; avg_I: ");
  Serial.print(avg_I);
  Serial.print("; P_out: ");
  Serial.print(P_out);
  Serial.print("; rpm_opt: ");
  Serial.println(rpm_opt);
}

// ===========================================================================
// START OF CONTROL CODE
// ===========================================================================
void loop() {

  // Measure RPM & power, then do RPC, MPPT, brake or start-up mode
  rpmCalcs();
  ivpCalcs();
  brakeCheck_On();
  turbineCond();

  switch (mode) {
    case 0: // Start-up ------------------------------------------------------

      Serial.println("\n\nStart-up mode");
      Comm = 20.0;

      // **********************************************************************
      // CHANGE TO 2.5 IF NOT WORKING!!! **************************************
      if (P_out > 1.0) {  // Condition to switch into MPPT mode

        mode = 1;
      }
      break;

    case 1: // MPPT mode -----------------------------------------------------

      Serial.println("\n\nMPPT");

      // Calculate optimal rotor speed
      rpm_opt = a * pow(abs(P_out), b);

      // Get appropriate change in B/B
      Comm = (K_mppt * (rpm_opt - rpm)) + Comm;

      if ((P_out < 3.0) && (Comm > 40.0)) {
        Comm = 40.0;
      }

      if (brake) {  // Actuate braking mode
        // Braking mode = short phases
        digitalWrite(eStopRelay, LOW);
        mode = 3;
        break;
      }

      if (P_out > P_rated) {  // Condition to switch into RPC mode
        mode = 2;
        break;
      }

      break;

    case 2: // RPC mode ------------------------------------------------------
      Serial.println("\n\nRPC");

      // Get appropriate change in B/B
      Comm = (K_rpc * (P_out - P_rated)) + (K_d * (P_out - P_old)) + Comm;

      if (brake) {  // Actuate braking mode
        // Braking mode = short phases
        digitalWrite(eStopRelay, LOW);
        mode = 3;
        break;
      }

      if (P_out < P_rateL) {  // Condition to switch back to MPPT mode
        mode = 1;
        break;
      }

      break;

    case 3: // Braking mode --------------------------------------------------

      // Reinitialize Comm for start-up condition
      Serial.println("Braking!");

      delay(5000);

      if ((avg_V > 5.5) && (digitalRead(NCswitch) == LOW)) {
        digitalWrite(eStopRelay, HIGH);
        brake = 0;
        mode = 0;
        break;
      }

      break;
  }

  // Buck-Boost control stuff
  bbControl();

} // End of loop()

