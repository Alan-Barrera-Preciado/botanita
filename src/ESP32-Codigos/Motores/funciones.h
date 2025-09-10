#ifndef FUNCIONES_H // include guard
#define FUNCIONES_H

#include <Wire.h>
#include <INA226_WE.h>

// INA226 //
void ina226Setup(INA226_WE &ina226, String name);
float ina226Read(INA226_WE &ina226, String name); // String

// MOTORES //
void setupEncoder(int pinInterrupcion, String RL);

extern volatile unsigned long contadorR;
extern volatile unsigned long contadorL;
void IRAM_ATTR incrementarR();
void IRAM_ATTR incrementarL();

void pwmSetup(int pinAdelante, int pinAtras);
void changePWM(int pinAdelante, int pinAtras, int trabajo);
float calcularRPM(volatile unsigned long &contadorPulsos, int res, float dt); // String

// COMUNICACION //
String getFromPy();
void serialToArray(int *arrayDestino);

extern int ArrayControl[3];
extern int pwmRAdelante, pwmRAtras, pwmLAdelante, pwmLAtras;
void control();


#endif
