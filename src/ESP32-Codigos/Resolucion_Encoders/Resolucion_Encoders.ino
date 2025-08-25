volatile unsigned long contadorL = 0;  // Pulsos encoder izquierdo
volatile unsigned long contadorR = 0;  // Pulsos encoder derecho

const int encoderL_Pin = 16;  // Pin encoder izquierdo
const int encoderR_Pin = 4;   // Pin encoder derecho

// Función de interrupción para el encoder izquierdo
void IRAM_ATTR contarPulsosL() {
  contadorL++;
}

// Función de interrupción para el encoder derecho
void IRAM_ATTR contarPulsosR() {
  contadorR++;
}

void setup() {
  Serial.begin(115200);

  pinMode(encoderL_Pin, INPUT);
  pinMode(encoderR_Pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderL_Pin), contarPulsosL, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderR_Pin), contarPulsosR, RISING);

  Serial.println("Gira cada motor manualmente hasta completar una vuelta completa.");
  Serial.println("Los pulsos de cada encoder se mostrarán en tiempo real.");
}

void loop() {
  Serial.print("Pulsos Encoder Izquierdo: ");
  Serial.print(contadorL);
  Serial.print(" | Pulsos Encoder Derecho: ");
  Serial.println(contadorR);
  
  delay(500);  // Muestra cada medio segundo
}
