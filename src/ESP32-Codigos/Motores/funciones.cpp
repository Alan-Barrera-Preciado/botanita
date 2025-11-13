#include "funciones.h"

void ina226Setup(INA226_WE &ina226, String name){
  if(!ina226.init()){
    Serial.println("INA226 no encontrado en " + name);
    while(1);
  }
  ina226.setAverage(AVERAGE_64);
  ina226.setConversionTime(CONV_TIME_332);
  ina226.waitUntilConversionCompleted();

  Serial.println("INA226 " + name + " configurado.");
}

float ina226Read(INA226_WE &ina226, String name){
  float current = ina226.getCurrent_mA();
  
  if (!isfinite(current)) {
    current = 0.0;
  }
  
  String outstr = String(current, 3);

  return current;
}

void setupEncoder(int pinInterrupcion, String RL){
  pinMode(pinInterrupcion, INPUT_PULLUP);
  if(RL == "R"){
    attachInterrupt(digitalPinToInterrupt(pinInterrupcion), incrementarR, RISING);  
  } else if (RL == "L") {
    attachInterrupt(digitalPinToInterrupt(pinInterrupcion), incrementarL, RISING);  
  }
  
}

void IRAM_ATTR incrementarR(){contadorR++;}
void IRAM_ATTR incrementarL(){contadorL++;}

void pwmSetup(int pinAdelante, int pinAtras){
  ledcAttach(pinAdelante, 5000, 8);   
  ledcWrite(pinAdelante, 0);         
  ledcAttach(pinAtras, 5000, 8);  
  ledcWrite(pinAtras, 0); 
}

float calcularRPM(volatile unsigned long &contadorPulsos, int res, float dt){
  noInterrupts();

  int intervaloPulsos = contadorPulsos;
  contadorPulsos = 0;
  interrupts();

  float rpm = (intervaloPulsos/dt*60/res);
  String outstr = String(rpm, 3);

  return rpm;
}

void changePWM(int pinAdelante, int pinAtras, int trabajo){
  if(trabajo >= 0){
    ledcWrite(pinAtras, 0);
    ledcWrite(pinAdelante, trabajo);
  } else {
    ledcWrite(pinAdelante, 0);
    ledcWrite(pinAtras, trabajo*-1);
  }
}

String getFromPy(){
  while (!Serial.available());
  String idxStr = Serial.readStringUntil('\n');
  return idxStr;
}

void serialToArray(int *arrayDestino) {
  String cad = Serial.readStringUntil('\n');
  cad.trim();

  if (cad.length() > 0) {
    int sep = cad.indexOf(',');
    if (sep > 0) {
      ArrayControl[0] = cad.substring(0, sep).toInt();
      ArrayControl[1] = cad.substring(sep + 1).toInt();
    }
  }
}

void control(){
  serialToArray(ArrayControl);
  changePWM(pwmRAdelante, pwmRAtras, ArrayControl[0]);  // R
  changePWM(pwmLAdelante, pwmLAtras, ArrayControl[1]);  // L
}