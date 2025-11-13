volatile unsigned long contadorL = 0;
volatile unsigned long contadorR = 0;

const int encoderL_Pin = 16;
const int encoderR_Pin = 4;

void IRAM_ATTR contarPulsosL() {
  contadorL++;
}

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
  
  delay(500);
}