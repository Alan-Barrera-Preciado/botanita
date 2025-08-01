#include <Wire.h>
#include <INA226_WE.h>
INA226_WE ina226_Izq = INA226_WE(0x40);  // Dirección I2C por defecto
INA226_WE ina226_Der = INA226_WE(0x41);  // Dirección I2C por defecto

int MotorSH_Izq = 6;
int MotorSA_Izq = 5;

int MotorSH_Der = 9;
int MotorSA_Der = 10;

int Encoder_Izq = 2;
int Encoder_Der = 3;
volatile byte Pulsos_Izq = 0;
volatile byte Pulsos_Der = 0;
int PulsosIntervalo_Izq = 0;
int PulsosIntervalo_Der = 0;
float RPM_Izq = 0;
float RPM_Der = 0;
int Direccion_Izq = 1;
int Direccion_Der = 1;

// Sensor de corriente
float Suma_Corriente_Izq = 0;
float Corriente_Promedio_Izq = 0;
int lecturaCorriente_Izq = 0;
float Corriente_Izq = 0;

float Suma_Corriente_Der = 0;
float Corriente_Promedio_Der = 0;
int lecturaCorriente_Der = 0;
float Corriente_Der = 0;

unsigned long TiempoActual_Izq = 0;
unsigned long TiempoAnterior_Izq = 0;
unsigned long TiempoActual_Der = 0;
unsigned long TiempoAnterior_Der = 0;
float dt_Izq = 0;
float dt_Der = 0;

float Control_Izq = 0;
int PWM_Izq = 0;

float Control_Der = 0;
int PWM_Der = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(MotorSH_Izq, OUTPUT);
  pinMode(MotorSA_Izq, OUTPUT);
  analogWrite(MotorSH_Izq, 0);
  analogWrite(MotorSA_Izq, 0);

  pinMode(MotorSH_Der, OUTPUT);
  pinMode(MotorSA_Der, OUTPUT);
  analogWrite(MotorSH_Der, 0);
  analogWrite(MotorSA_Der, 0);

  pinMode(Encoder_Izq, INPUT);
  attachInterrupt(digitalPinToInterrupt(Encoder_Izq), Contador_Izq, RISING);
  pinMode(Encoder_Der, INPUT);
  attachInterrupt(digitalPinToInterrupt(Encoder_Der), Contador_Der, RISING);

  if (!ina226_Izq.init()) {
    Serial.println("INA226 Izq no detectado.");
  }
  if (!ina226_Der.init()) {
    Serial.println("INA226 Der no detectado.");
  }

  ina226_Izq.setAverage(AVERAGE_16);
  ina226_Izq.setConversionTime(CONV_TIME_1100);
  ina226_Der.setAverage(AVERAGE_16);
  ina226_Der.setConversionTime(CONV_TIME_1100);
}

void loop() {
  if (Serial.available()) {
    ControlMotor();
    EnviarDatos();
  }
}

void Contador_Izq(){
  Pulsos_Izq++;
}
void Contador_Der(){
  Pulsos_Der++;
}

void ControlMotor(){
  String cad = Serial.readStringUntil('\n');
  int pos = cad.indexOf(',');
  String cad_Izq = cad.substring(0, pos);
  String cad_Der = cad.substring(pos + 1);
  Control_Izq = cad_Izq.toInt();
  Control_Der = cad_Der.toInt();
  if(Control_Izq < 0){
    Control_Izq = Control_Izq*-1;
    analogWrite(MotorSH_Izq, 0);
    analogWrite(MotorSA_Izq, Control_Izq);
    Direccion_Izq = 1;
  }
  else{
    analogWrite(MotorSH_Izq, Control_Izq);
    analogWrite(MotorSA_Izq, 0);
    Direccion_Izq = 0;
  }
  if(Control_Der < 0){
    Control_Der = Control_Der*-1;
    analogWrite(MotorSH_Der, 0);
    analogWrite(MotorSA_Der, Control_Der);
    Direccion_Der = 1;
  }
  else{
    analogWrite(MotorSH_Der, Control_Der);
    analogWrite(MotorSA_Der, 0);
    Direccion_Der = 0;
  }
}

void EnviarDatos() {
  RPM_Izq = CalcularRPM_Izq();
  RPM_Der = CalcularRPM_Der();
  if(Direccion_Izq == 1){
    RPM_Izq = RPM_Izq*-1;
  }
  if(Direccion_Der == 1){
    RPM_Der = RPM_Der*-1;
  }
  Corriente_Izq = PromedioCorriente_Izq();
  Corriente_Der = PromedioCorriente_Der();
  Serial.print(RPM_Izq);
  Serial.print(",");
  Serial.print(Corriente_Izq);
  Serial.print(":");
  Serial.print(RPM_Der);
  Serial.print(",");
  Serial.println(Corriente_Der);
}

float CalcularRPM_Izq(){
  TiempoActual_Izq = millis();
  dt_Izq = (TiempoActual_Izq - TiempoAnterior_Izq)/1000.0;
  TiempoAnterior_Izq = TiempoActual_Izq;
  noInterrupts();
  PulsosIntervalo_Izq = Pulsos_Izq;
  Pulsos_Izq = 0;
  RPM_Izq = (PulsosIntervalo_Izq/dt_Izq)*60/750;
  interrupts();
  return RPM_Izq;
}

float CalcularRPM_Der(){
  TiempoActual_Der = millis();
  dt_Der = (TiempoActual_Der - TiempoAnterior_Der)/1000.0;
  TiempoAnterior_Der = TiempoActual_Der;
  noInterrupts();
  PulsosIntervalo_Der = Pulsos_Der;
  Pulsos_Der = 0;
  RPM_Der = (PulsosIntervalo_Der/dt_Der)*60/750;
  interrupts();
  return RPM_Der;
}

int PromedioCorriente_Izq() {
  for (int i = 0; i < 64; i++) {
    Suma_Corriente_Izq += ina226_Izq.getCurrent_mA();;
  }
  Corriente_Promedio_Izq = Suma_Corriente_Izq / 64.0;
  Suma_Corriente_Izq = 0;
  return (int)Corriente_Promedio_Izq;
}

int PromedioCorriente_Der() {
  for (int i = 0; i < 64; i++) {
    Suma_Corriente_Der += ina226_Der.getCurrent_mA();;
  }
  Corriente_Promedio_Der = Suma_Corriente_Der / 64.0;
  Suma_Corriente_Der = 0;
  return (int)Corriente_Promedio_Der;
}