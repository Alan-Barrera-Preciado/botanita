#include <Wire.h>
#include <INA226_WE.h>

// Dirección I2C del INA226
#define I2C_MOTOR_RIGHT_ADD 0x41
#define I2C_MOTOR_LEFT_ADD 0x40

INA226_WE ina226MD(I2C_MOTOR_RIGHT_ADD);
INA226_WE ina226MI(I2C_MOTOR_LEFT_ADD);

// Protocolo
#define HANDSHAKE 0
#define MEASURE_REQUEST 1

int inByte;
String outstr;
String idxStr;

// Pines PWM
int pwmRAdelante = 15;
int pwmRAtras = 2;

int pwmLAdelante = 5;
int pwmLAtras  = 17;

// pines encoders

int encoderR = 4;
int encoderL = 16;

// incrementos de encoders

volatile unsigned long contadorR = 0;
volatile unsigned long contadorL = 0;

// arreglo Control
int ArrayControl[3];

// para calcular dt
unsigned long tiempoAnterior = 0;
unsigned long tiempoActual = 0;
float dt = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Ina Setup
  ina226Setup(ina226MD, "Derecha");
  ina226Setup(ina226MI, "Izquierda");

  // Encoders Setup
  setupEncoder(encoderR, "R");
  setupEncoder(encoderL, "L");
  
  // PWM setup
  pwmSetup(pwmRAdelante, pwmRAtras);
  pwmSetup(pwmLAdelante, pwmLAtras);
}

int test = 0;

void loop() {
  if (Serial.available() > 0) {
    // Serial.flush();
    inByte = Serial.read();

    if (inByte == HANDSHAKE) {
      Serial.println("OK");
    }

    else if (inByte >= MEASURE_REQUEST) {
      // Leer y enviar datos (corriente)
      tiempoActual = millis();
      dt = (tiempoActual - tiempoAnterior) / 1000.0; // pasa a segundos
      tiempoAnterior = tiempoActual;

      String rigthRPM = calcularRPM(contadorR, 700, dt);
      String leftRPM = calcularRPM(contadorL, 800, dt);
      
      String rightRead = ina226Read(ina226MD, "Derecha");
      String leftRead = ina226Read(ina226MI, "Izq");

      Serial.println("\n");
      // rightRead + "," + rigthRPM + "," + leftRead + "," + leftRPM
      outstr = rightRead + "," + rigthRPM + "," + leftRead + "," + leftRPM;
      Serial.println(outstr);

      // Control
      control();

      //delay(25); // dt      

    } //
  }
}


// Funciones 

void ina226Setup(INA226_WE &ina226, String name){
  Wire.begin();
  ina226.init();
  //ina226.setResistorRange(0.002, 10.0);
  ina226.setAverage(AVERAGE_64);
  ina226.setConversionTime(CONV_TIME_332);

  ina226.waitUntilConversionCompleted();
  //Serial.println(name);
  //Serial.println("Setup Done");
}

String ina226Read(INA226_WE &ina226, String name){
  float current = ina226.getCurrent_mA();
  float voltage = ina226.getBusVoltage_V();
  float power   = ina226.getBusPower();

  // Formato: corriente,voltaje,potencia
  // String outstr = String(current, 3) + "," + String(voltage, 3) + "," + String(power, 3);
  String outstr = String(current, 3);
  // Serial.println(outstr);

  return outstr;
}

String getFromPy(){
  // Esperar índice desde Python y responderlo
  while (!Serial.available());
  String idxStr = Serial.readStringUntil('\n');
  return idxStr;
  // Serial.println(idxStr); // reenviarlo para validación en Python
}

void pwmSetup(int pinAdelante, int pinAtras){
  pinMode(pinAdelante, OUTPUT);
  analogWrite(pinAdelante, 0);
  pinMode(pinAtras, OUTPUT);
  analogWrite(pinAtras, 0);
}

void changePWM(int pinAdelante, int pinAtras, int trabajo){
  if(trabajo >= 0){
    analogWrite(pinAtras, 0);
    analogWrite(pinAdelante, trabajo);  
  } else {
    analogWrite(pinAtras, trabajo*-1); 
    analogWrite(pinAdelante, 0);
  }
}

// para encoder 

void IRAM_ATTR incrementarR(){
  contadorR++;
}


void IRAM_ATTR incrementarL(){
  contadorL++;
}


void setupEncoder(int pinInterrupcion, String RL){
  pinMode(pinInterrupcion, INPUT_PULLUP);
  if(RL == "R"){
    attachInterrupt(digitalPinToInterrupt(pinInterrupcion), incrementarR, RISING);  
  } else if (RL == "L") {
    attachInterrupt(digitalPinToInterrupt(pinInterrupcion), incrementarL, RISING);  
  }
  
}

void serialToArray(int *arrayDestino) {
  String cad = Serial.readStringUntil('\n');
  cad.trim();

  // Serial.println("Cadena original: [" + cad + "]");

  int pwmR = 0, pwmL = 0;
  sscanf(cad.c_str(), "%*[^0-9-]%d,%d", &pwmR, &pwmL);  // limpia basura

  arrayDestino[0] = pwmR;
  arrayDestino[1] = pwmL;
}

void control(){
  serialToArray(ArrayControl);
  changePWM(pwmRAdelante, pwmRAtras, ArrayControl[0]);  // R
  changePWM(pwmLAdelante, pwmLAtras, ArrayControl[1]);  // L
}

String calcularRPM(volatile unsigned long &contadorPulsos, int res, float dt){
  noInterrupts();

  int intervaloPulsos = contadorPulsos;
  contadorPulsos = 0;
  interrupts();

  float rpm = (intervaloPulsos/dt*60/res);
  String outstr = String(rpm, 3);
  // Serial.println(outstr);

  return outstr;
}