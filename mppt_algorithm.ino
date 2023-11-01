#include <Arduino.h>

const int CURRENT_SENSOR_PIN = A1; // ACS712 sensor analog input pin
const int VOLTAGE_DIVIDER_PIN = A2; // Voltage divider analog input pin
const int PWM_PIN = 9; // PWM output pin

const double D_INIT = 0.4;
const double D_MAX = 0.9;
const double D_MIN = 0.1;
const double DELTA_D = 0.001;

//const float ACS712_SENSOR_CONSTANT = 0.066; //sensitivity value for the ACS712-30A sensor
const float scalingFactor = 23.898/5; //since the voltage which is fed to arduino cannot be bigger than 5V, the scaling factor is needed
const float resistance = 3.3;
const float OpAmpGain = 20.0;//22.2;

double Vold = 0;
double Pold = 0;
double Dold = D_INIT;

double readCurrent() {
  int sensorValue = analogRead(CURRENT_SENSOR_PIN);
  double voltage = (sensorValue * 5.0) / 800.0;
  double current = voltage/OpAmpGain/resistance; 
  return current;
}

double readVoltage() {
  int sensorValue = analogRead(VOLTAGE_DIVIDER_PIN);
  double voltage = (sensorValue * scalingFactor * 5.0) / 1024.0;
  return voltage;
}

double fcn(double vpv, double ipv) {
  double D = 0;

  double P = vpv * ipv;
  double dV = vpv - Vold;
  double dP = P - Pold;

  if (dP != 0) {
    if (dP < 0) {
      if (dV < 0) {
        D = Dold - DELTA_D;
      } 
      else {
        D = Dold + DELTA_D;
      }
    } 
    else {
      if (dV < 0) {
        D = Dold + DELTA_D;
      } 
      else {
        D = Dold - DELTA_D;
      }
    }
  } 
  else {
    D = Dold;
  }


  if (D > D_MAX) {
    D = D_MAX;
  } 
  else if (D < D_MIN) {
    D = D_MIN;
  }

  Dold = D;
  Vold = vpv;
  Pold = P;

  return D;
}

void setup() {
  Serial.begin(9600);
  pinMode(CURRENT_SENSOR_PIN, INPUT);
  pinMode(VOLTAGE_DIVIDER_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);

  // Set PWM frequency for pin 9 and 10 (Timer1)
  TCCR1B = TCCR1B & 0b11111000 | 0x01; // For maximum base frequency, about 122Hz PWM
}

void loop() {
  double pvVoltage = readVoltage();
  double pvCurrent = readCurrent();

  double dutyCycle = fcn(pvVoltage, pvCurrent);
  int pwmValue = (int)(dutyCycle * 255);

  // Output the PWM signal
  analogWrite(PWM_PIN, pwmValue);

  Serial.print("PV Voltage: ");
  Serial.print(pvVoltage);
  Serial.print(" V, PV Current: ");
  Serial.print(pvCurrent);
  Serial.print(" A, Duty Cycle: ");
  Serial.print(dutyCycle);
  Serial.print(", PWM Value: ");
  Serial.println(pwmValue);

  delay(100); // Wait for 1 second
}
