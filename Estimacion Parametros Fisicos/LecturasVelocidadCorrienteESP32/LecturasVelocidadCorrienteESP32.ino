#include <Wire.h>
#include <INA226_WE.h>

INA226_WE ina226_Izq = INA226_WE(0x40);
INA226_WE ina226_Der = INA226_WE(0x41);

int MotorSH_Izq = 17;
int MotorSA_Izq = 5;

int MotorSH_Der = 2;
int MotorSA_Der = 15;

int RPM_Izq = 0;
int RPM_Der = 0;

int Encoder_Izq = 16;
int Encoder_Der = 4;
int Resolucion_Izq = 800;
int Resolucion_Der = 700;
volatile byte Pulsos_Izq = 0;
volatile byte Pulsos_Der = 0;
int PulsosIntervalo_Izq = 0;
int PulsosIntervalo_Der = 0;

float Corriente_Izq = 0;
float Corriente_Der = 0;

unsigned long TiempoAnterior = 0;
unsigned long TiempoActual = 0;
float IntervaloTiempo = 0;
int t = 0;
int Aux = 1;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Ina Setup

  Wire.begin();

  ina226_Izq.init();
  ina226_Izq.setAverage(AVERAGE_16);
  ina226_Izq.setConversionTime(CONV_TIME_1100);
  ina226_Izq.waitUntilConversionCompleted();

  ina226_Der.init();
  ina226_Der.setAverage(AVERAGE_16);
  ina226_Der.setConversionTime(CONV_TIME_1100);
  ina226_Der.waitUntilConversionCompleted();

  // Encoders Setup

  pinMode(Encoder_Izq, INPUT);
  attachInterrupt(digitalPinToInterrupt(Encoder_Izq), Contador_Izq, RISING);
  pinMode(Encoder_Der, INPUT);
  attachInterrupt(digitalPinToInterrupt(Encoder_Der), Contador_Der, RISING);
  
  // Motores Setup

  pinMode(MotorSH_Izq, OUTPUT);
  pinMode(MotorSA_Izq, OUTPUT);
  analogWrite(MotorSH_Izq, 0);
  analogWrite(MotorSA_Izq, 0);

  pinMode(MotorSH_Der, OUTPUT);
  pinMode(MotorSA_Der, OUTPUT);
  analogWrite(MotorSH_Der, 0);
  analogWrite(MotorSA_Der, 0);

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

float CalcularRPM(float IntervaloTiempo, int Pulsos, int Resolucion){
  float RPM = (Pulsos/IntervaloTiempo)*60/Resolucion;
  return RPM;
}

void EnviarDatos() {
  
  noInterrupts();
  TiempoActual = millis();
  IntervaloTiempo = (TiempoActual - TiempoAnterior) / 1000;
  TiempoAnterior = TiempoActual;
  RPM_Izq = CalcularRPM(IntervaloTiempo, Pulsos_Izq, Resolucion_Izq);
  RPM_Der = CalcularRPM(IntervaloTiempo, Pulsos_Der, Resolucion_Der);
  Pulsos_Izq = 0;
  Pulsos_Der = 0;
  interrupts();

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