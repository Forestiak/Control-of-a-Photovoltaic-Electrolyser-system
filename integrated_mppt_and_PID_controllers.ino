#include <PID_v1.h>

int choice; //variable to store the choice(MPPT or PID)
int d = 0; //variable to store the duty cycle
const int CURRENT_SENSOR_PIN_PV = A1; // ACS712 sensor analog input pin
const int VOLTAGE_DIVIDER_PIN_PV = A2; // Voltage divider analog input pin for PV
const int VOLTAGE_DIVIDER_PIN_OUTPUT = A0; //Voltage divider analog input pin for the output from the boost converter
const int pwmPin = 9; //Output pin to send the PWM signal(Duty Cycle) to the boost converter
const int pot = A3; // Analog input pin from the potentiometer to choose the mode of operation
int pwmValue; //Variable to store the PWM value

//Variables for the current sensor
float c_sensorValue;
const float V_ref = 4.2; //The real value of the 5V of the arduino
const float R_shunt = 0.1; //Ohms of the shunt
const float R_gain = 68; //Measured value of the gain setting resistor
const float Gain = 1 + 10000/R_gain; //InAmp gain
float currentError = 1.1473;
//const float ACS712_SENSOR_CONSTANT = 0.066; //sensitivity value for the ACS712-30A sensor

//Variables for PV Voltage divider
const float voltageScalar = 5; //Empirically found correction of voltage divider (r2/r1 + measured fault)

//Variables for MPPT Algorithm
const double D_INIT = 0.4;
const double D_MAX = 0.9;
const double D_MIN = 0.1;
const double DELTA_D = 20e-3;
double Vold = 0;
double Pold = 0;
double Dold = D_INIT;

//Variables for PID Control
double setpoint = 24.0;
double input, output;
double Kp = 0.0135, Ki = 0.09, Kd = 0;
//double Kp = 1, Ki = 0, Kd = 0;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

float readCurrent() { //Read the input current from PV
  c_sensorValue = analogRead(CURRENT_SENSOR_PIN_PV);
  float current = c_sensorValue*((V_ref)/(1023*R_shunt*Gain))*currentError*1000;
  return current;
}

double readVoltage() { //Read the input voltage from PV
  int sensorValue2 = analogRead(VOLTAGE_DIVIDER_PIN_PV);
  double resistorRatio = (99.9+473)/99.9;
  double voltage = (((sensorValue2 * 5.0) / 1023.0) * resistorRatio);
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
  pinMode(CURRENT_SENSOR_PIN_PV, INPUT);
  pinMode(VOLTAGE_DIVIDER_PIN_PV, INPUT);
  pinMode(pot, INPUT);
  pinMode(VOLTAGE_DIVIDER_PIN_OUTPUT, INPUT);
  // Set up PWM output and frequency
  pinMode(pwmPin, OUTPUT);
  // Configure Timer1 for Fast PWM mode (mode 14) with no prescaler
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
  // Set the TOP value to 255 for 8-bit resolution
  ICR1 = 255;
  // Configure PID controller
  myPID.SetMode(AUTOMATIC);
}

void loop(){
  choice = analogRead(pot);
  if(choice>512){
    Serial.print("PID Control");
    Serial.print("\t");
    d = constantv_mode();
  }
  else if(choice<512){
    Serial.print("MPPT Control");
    Serial.print("\t");
    d = mppt_mode();
  }
  analogWrite(pwmPin, d);
  delay(500);
}

int mppt_mode(){
  double pvVoltage = readVoltage();
  float pvCurrent = readCurrent();
  double dutyCycle = fcn(pvVoltage, pvCurrent);
  pwmValue = round(dutyCycle * 255);
  
  //Serial.print(pvVoltage);
  //Serial.print("\tRaw Current Value: ");
  //Serial.print(c_sensorValue);
  //Serial.print("\tCurrent(mA): ");
  Serial.print(pvCurrent);
  Serial.print("\t");
  //Serial.print("\tPOWER(mW): ");
  ///Serial.print(pvCurrent*pvVoltage*1000);
  //Serial.print("\tDuty Cycle: ");
  //Serial.print(dutyCycle);
  //Serial.print("\tPWM Value: ");
  //Serial.println(pwmValue);
  int sensorValue = analogRead(VOLTAGE_DIVIDER_PIN_OUTPUT);
  input = (sensorValue * (5.0 / 1023.0)) * ((19.9 + 2.18) / 2.18);
  //Serial.print("\tInput Voltage: ");
  Serial.print(pvVoltage);
  Serial.print("\t");
  //Serial.print("\tOutput Voltage: ");
  Serial.print(input);
  Serial.print("\t");
  Serial.println(dutyCycle);
  return pwmValue;
}

int constantv_mode(){
  int sensorValue = analogRead(VOLTAGE_DIVIDER_PIN_OUTPUT);
  input = (sensorValue * (5.0 / 1023.0)) * ((19.9 + 2.18) / 2.18);
  myPID.Compute();
  if(output>0.9){
    output = 0.9;
  }
  pwmValue = round(output * 255);
  double pvVoltage = readVoltage();
  float pvCurrent = readCurrent();
  Serial.print(pvCurrent);
  Serial.print("\t");
  //Serial.print("\tInput Voltage: ");
  Serial.print(pvVoltage);
  Serial.print("\t");
  //Serial.print("\tOutput Voltage: ");
  Serial.print(input);
  Serial.print("\t");
  Serial.println(output);
  //Serial.print("\tDuty Cycle: ");
  //Serial.print(output);
  //Serial.print("\tPWM Value: ");
  //Serial.println(pwmValue);
  return pwmValue;
