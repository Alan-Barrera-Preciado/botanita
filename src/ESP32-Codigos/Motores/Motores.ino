#include "funciones.h"

#define I2C_MOTOR_LEFT_ADD  0x40
#define I2C_MOTOR_RIGHT_ADD 0x41

INA226_WE ina226MD = INA226_WE(I2C_MOTOR_RIGHT_ADD);
INA226_WE ina226MI = INA226_WE(I2C_MOTOR_LEFT_ADD);

#define HANDSHAKE_START1 0xAA
#define HANDSHAKE_START2 0x55

#define HANDSHAKE_CMD    0x01
#define MEASURE_CMD      0x02
#define HANDSHAKE_ACK    0xCC

unsigned long handshakeTimeout = 1000;
int maxRetries = 5;

void clearSerialInput() {
  while (Serial.available()) Serial.read();
}

bool waitForFrame(unsigned long timeout, uint8_t &b1, uint8_t &b2, uint8_t &cmd) {
  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (Serial.available() >= 3) {
      b1 = Serial.read();
      b2 = Serial.read();
      cmd = Serial.read();
      return true;
    }
    delay(1);
  }
  return false;
}

bool doHandshake() {
  clearSerialInput();
  int tries = 0;
  while (tries++ < maxRetries) {
    Serial.write(HANDSHAKE_START1);
    Serial.write(HANDSHAKE_START2);
    Serial.write(HANDSHAKE_CMD);
    Serial.flush();

    uint8_t b1, b2, cmd;
    if (waitForFrame(handshakeTimeout, b1, b2, cmd)) {
      if (b1 == HANDSHAKE_START1 && b2 == HANDSHAKE_START2 && cmd == HANDSHAKE_CMD) {
        Serial.write(HANDSHAKE_ACK);
        return true;
      }
    }
    delay(100);
  }
  return false;
}

int inByte;
String outstr;
String idxStr;

int encoderL = 32;
int encoderR = 23;

int pwmLAdelante = 33;
int pwmLAtras  = 25;

int pwmRAdelante = 26;
int pwmRAtras = 27;

volatile unsigned long contadorR = 0;
volatile unsigned long contadorL = 0;

int ArrayControl[3];

unsigned long tiempoAnterior = 0;
unsigned long tiempoActual = 0;
float dt = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Wire.begin(21, 22);
  delay(200);

  ina226Setup(ina226MD, "Derecha");
  ina226Setup(ina226MI, "Izquierda");

  setupEncoder(encoderR, "R");
  setupEncoder(encoderL, "L");
  
  pwmSetup(pwmLAdelante, pwmLAtras);
  pwmSetup(pwmRAdelante, pwmRAtras);

  clearSerialInput();

  if (doHandshake()) {
    Serial.println("Handshake OK");
  } else {
    Serial.println("Handshake Fallo");
  }
}

static uint8_t serial_state = 0;
static char asciiBuf[64];
static uint8_t asciiPos = 0;

void handleCommand(uint8_t cmd) {
  if (cmd == MEASURE_CMD) {
    tiempoActual = millis();
    dt = (tiempoActual - tiempoAnterior) / 1000.0f;
    tiempoAnterior = tiempoActual;

    float rightRPM = calcularRPM(contadorR, 820, dt);
    float leftRPM  = calcularRPM(contadorL, 820, dt);
    float rightI   = ina226Read(ina226MD, "Derecha");
    float leftI    = ina226Read(ina226MI, "Izquierda");

    Serial.printf("%.2f,%.2f,%.2f,%.2f\n", rightI, rightRPM, leftI, leftRPM);
  }
  else if (cmd == HANDSHAKE_CMD) {
    Serial.write(HANDSHAKE_START1);
    Serial.write(HANDSHAKE_START2);
    Serial.write(HANDSHAKE_ACK);
  }
}

void processAsciiLine(const char *line) {
  int comma = -1;
  for (int i=0; line[i] != '\0'; ++i) if (line[i] == ',') { comma = i; break; }
  if (comma > 0) {
    int pwmL = atoi(String(line).substring(0, comma).c_str());
    int pwmR = atoi(String(line + comma + 1).c_str());
    changePWM(pwmRAdelante, pwmRAtras, pwmR);
    changePWM(pwmLAdelante, pwmLAtras, pwmL);
  }
}

void loop() {
  while (Serial.available()) {
    int c = Serial.read();
    if (serial_state == 0) {
      if ((uint8_t)c == HANDSHAKE_START1) {
        serial_state = 1;
      } else {
        if (c == '\n' || asciiPos >= (sizeof(asciiBuf)-2)) {
          asciiBuf[asciiPos] = '\0';
          if (asciiPos > 0) processAsciiLine(asciiBuf);
          asciiPos = 0;
        } else if (c != '\r') {
          asciiBuf[asciiPos++] = (char)c;
        }
      }
    } else if (serial_state == 1) {
      if ((uint8_t)c == HANDSHAKE_START2) serial_state = 2;
      else serial_state = ((uint8_t)c == HANDSHAKE_START1) ? 1 : 0;
    } else if (serial_state == 2) {
      uint8_t cmd = (uint8_t)c;
      serial_state = 0;
      handleCommand(cmd);
    }
  }
}
