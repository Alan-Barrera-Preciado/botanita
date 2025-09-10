#include "funciones.h" // mas includes en este .h

// Dirección I2C del INA226
#define I2C_MOTOR_LEFT_ADD  0x40
#define I2C_MOTOR_RIGHT_ADD 0x41

// Declaracion objetos INA226

INA226_WE ina226MD = INA226_WE(I2C_MOTOR_RIGHT_ADD);
INA226_WE ina226MI = INA226_WE(I2C_MOTOR_LEFT_ADD);

// Protocolo
// Handshake robusto (frame + timeout)
#define HANDSHAKE_START1 0xAA
#define HANDSHAKE_START2 0x55

#define HANDSHAKE_CMD    0x01   // comando handshake
#define MEASURE_CMD      0x02   // comando de medición
#define HANDSHAKE_ACK    0xCC   // respuesta del ESP

unsigned long handshakeTimeout = 1000; // ms
int maxRetries = 5;

void clearSerialInput() {
  while (Serial.available()) Serial.read();
}

bool waitForFrame(unsigned long timeout, uint8_t &b1, uint8_t &b2, uint8_t &cmd) {
  unsigned long start = millis();
  while (millis() - start < timeout) {
    // buscamos los bytes de inicio
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
    // Enviar petición de handshake (host debería hacerlo; aquí ESP puede iniciar)
    Serial.write(HANDSHAKE_START1);
    Serial.write(HANDSHAKE_START2);
    Serial.write(HANDSHAKE_CMD);
    Serial.flush(); // espera transmisión salida (opcional)

    // Esperar ACK
    uint8_t b1, b2, cmd;
    if (waitForFrame(handshakeTimeout, b1, b2, cmd)) {
      // Si recibimos un frame, validar y responder
      if (b1 == HANDSHAKE_START1 && b2 == HANDSHAKE_START2 && cmd == HANDSHAKE_CMD) {
        Serial.write(HANDSHAKE_ACK);
        return true;
      }
    }
    delay(100); // pequeño retardo antes de reintento
  }
  return false;
}

int inByte;
String outstr;
String idxStr;


// pines encoders
int encoderL = 32;
int encoderR = 23;

// Pines PWM
int pwmLAdelante = 33; // no se puede 35 
int pwmLAtras  = 25;

int pwmRAdelante = 26;
int pwmRAtras = 27;

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
  //unsigned long start = millis();
  //while (!Serial && millis() - start < 2000) 
  while(!Serial);
  // Inicializar I2C
  Wire.begin(21, 22);
  delay(200); // dar tiempo al host para abrir puerto

  // Ina Setup
  ina226Setup(ina226MD, "Derecha");
  ina226Setup(ina226MI, "Izquierda");

  // Encoders Setup
  setupEncoder(encoderR, "R");
  setupEncoder(encoderL, "L");
  
  // PWM setup
  pwmSetup(pwmLAdelante, pwmLAtras);
  pwmSetup(pwmRAdelante, pwmRAtras);

  clearSerialInput();

  if (doHandshake()) {
    Serial.println("Handshake OK");
  } else {
    Serial.println("Handshake FALLó");
  }
}

// ===== Loop =====
// VARIABLES PARA PARSER
static uint8_t serial_state = 0; // 0 = waiting start1, 1 = waiting start2, 2 = waiting cmd
static char asciiBuf[64];
static uint8_t asciiPos = 0;

void handleCommand(uint8_t cmd) {
  if (cmd == MEASURE_CMD) {
    // medida rápida
    tiempoActual = millis();
    dt = (tiempoActual - tiempoAnterior) / 1000.0f;
    tiempoAnterior = tiempoActual;

    float rightRPM = calcularRPM(contadorR, 820, dt);
    float leftRPM  = calcularRPM(contadorL, 820, dt);
    float rightI   = ina226Read(ina226MD, "Derecha");
    float leftI    = ina226Read(ina226MI, "Izquierda");

    // Enviar CSV (formato: corrienteD(mA), rpmD, corrienteI(mA), rpmI)
    // evita muchos Serial.println debug
    Serial.printf("%.2f,%.2f,%.2f,%.2f\n", rightI, rightRPM, leftI, leftRPM);
  }
  else if (cmd == HANDSHAKE_CMD) {
    // responder ACK completo
    Serial.write(HANDSHAKE_START1);
    Serial.write(HANDSHAKE_START2);
    Serial.write(HANDSHAKE_ACK);
    // opcional: delay(2); Serial.println("Handshake OK");
  }
}

void processAsciiLine(const char *line) {
  // ejemplo: "150,150"
  int comma = -1;
  for (int i=0; line[i] != '\0'; ++i) if (line[i] == ',') { comma = i; break; }
  if (comma > 0) {
    int pwmL = atoi(String(line).substring(0, comma).c_str()); // o parse manual
    int pwmR = atoi(String(line + comma + 1).c_str());
    // aplica control
    changePWM(pwmRAdelante, pwmRAtras, pwmR);
    changePWM(pwmLAdelante, pwmLAtras, pwmL);
  }
}

void loop() {
  // leer todos los bytes disponibles
  while (Serial.available()) {
    int c = Serial.read();
    if (serial_state == 0) {
      if ((uint8_t)c == HANDSHAKE_START1) {
        serial_state = 1;
      } else {
        // acumular ASCII hasta '\n'
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

  // aquí puedes hacer tareas periódicas no bloqueantes si hace falta
}
