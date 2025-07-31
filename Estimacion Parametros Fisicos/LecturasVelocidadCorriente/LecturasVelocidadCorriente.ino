#include <Wire.h>
#include <INA226_WE.h>
INA226_WE ina226_Izq = INA226_WE(0x40);  // Dirección I2C por defecto
INA226_WE ina226_Der = INA226_WE(0x41);  // Dirección I2C por defecto

int MotorSH_Izq = 6;
int MotorSA_Izq = 5;

int MotorSH_Der = 9;
int MotorSA_Der = 10;

int RPM_Izq = 0;
int RPM_Der = 0;

int Encoder_Izq = 2;
int Encoder_Der = 3;
int Resolucion_Izq = 750;
int Resolucion_Der = 750;
volatile byte Pulsos_Izq = 0;
volatile byte Pulsos_Der = 0;
int PulsosIntervalo_Izq = 0;
int PulsosIntervalo_Der = 0;

float Corriente_Izq = 0;
float Corriente_Der = 0;

unsigned long TiempoActual_Izq = 0;
unsigned long TiempoAnterior_Izq = 0;
unsigned long TiempoActual_Der = 0;
unsigned long TiempoAnterior_Der = 0;
float dt_Izq = 0;
float dt_Der = 0;

int Aux = 1;
float t = 0;

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
    while(Aux == 1){
      delay(50);
      if(t < 2){
        analogWrite(MotorSH_Izq, 0);
        analogWrite(MotorSH_Der, 0);
      }
      else if(t < 4 && t >= 2){
        analogWrite(MotorSH_Izq, 255);
        analogWrite(MotorSH_Der, 255);
      }
      else if(t < 6 && t >= 4){
        analogWrite(MotorSH_Izq, 0);
        analogWrite(MotorSH_Der, 0);
      }
      else if(t < 8 && t >= 6){
        analogWrite(MotorSH_Izq, 200);
        analogWrite(MotorSH_Der, 200);
      }
      else if(t < 10 && t >= 8){
        analogWrite(MotorSH_Izq, 0);
        analogWrite(MotorSH_Der, 0);
      }
      else if(t < 12 && t >= 10){
        analogWrite(MotorSH_Izq, 150);
        analogWrite(MotorSH_Der, 150);
      }
      else if(t < 14 && t >= 12){
        analogWrite(MotorSH_Izq, 0);
        analogWrite(MotorSH_Der, 0);
      }
      else if(t < 16 && t >= 14){
        analogWrite(MotorSH_Izq, 100);
        analogWrite(MotorSH_Der, 100);
      }
      else if(t < 18 && t >= 16){
        analogWrite(MotorSH_Izq, 0);
        analogWrite(MotorSH_Der, 0);
      }
      else if(t < 20){
        analogWrite(MotorSH_Izq, 50);
        analogWrite(MotorSH_Der, 50);
      }
      EnviarDatos();
      String cad = Serial.readStringUntil('\n');
      Aux = cad.toInt();
      t = t + 0.05;
    }
    analogWrite(MotorSH_Izq, 0);
    analogWrite(MotorSH_Der, 0);
  }
}

void Contador_Izq(){
  Pulsos_Izq++;
}
void Contador_Der(){
  Pulsos_Der++;
}

float CalcularRPM_Izq(){
  TiempoActual_Izq = millis();
  dt_Izq = (TiempoActual_Izq - TiempoAnterior_Izq)/1000.0;
  TiempoAnterior_Izq = TiempoActual_Izq;
  noInterrupts();
  PulsosIntervalo_Izq = Pulsos_Izq;
  Pulsos_Izq = 0;
  RPM_Izq = (PulsosIntervalo_Izq/dt_Izq)*60/Resolucion_Izq;
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
  RPM_Der = (PulsosIntervalo_Der/dt_Der)*60/Resolucion_Der;
  interrupts();
  return RPM_Der;
}

void EnviarDatos() {
  RPM_Izq = CalcularRPM_Izq();
  RPM_Der = CalcularRPM_Der();
  Corriente_Izq = ina226_Izq.getCurrent_mA();
  Corriente_Der = ina226_Der.getCurrent_mA();
  Serial.print(RPM_Izq);
  Serial.print(",");
  Serial.print(Corriente_Izq);
  Serial.print(":");
  Serial.print(RPM_Der);
  Serial.print(",");
  Serial.println(Corriente_Der);
}