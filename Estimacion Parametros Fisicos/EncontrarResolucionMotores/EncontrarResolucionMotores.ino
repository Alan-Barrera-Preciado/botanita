volatile long Pulsos = 0;
bool Contando = false;
int PWM = 100;

const int encoderPin = 16;
const int pwmPin = 15;
const int pwmChannel = 0;
const int pwmFreq = 5000;
const int pwmResolution = 8;

void IRAM_ATTR Contador() {
  if (Contando) {
    Pulsos++;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), Contador, RISING);

  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);
  ledcWrite(pwmChannel, 0);

  Serial.println("Escribe 'inicio' para comenzar. El motor girará lentamente.");
  Serial.println("Cuando dé 100 vueltas, escribe 'fin' para calcular la resolución.");
}

void loop() {
  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando.equalsIgnoreCase("inicio")) {
      Pulsos = 0;
      Contando = true;
      ledcWrite(pwmChannel, PWM);
      Serial.println("Contando pulsos... Escribe 'fin' cuando dé 100 vueltas.");
    }
    else if (comando.equalsIgnoreCase("fin")) {
      Contando = false;
      ledcWrite(pwmChannel, 0);

      float Resolucion = (float)Pulsos / 100.0;
      Serial.println("---------- RESULTADO ----------");
      Serial.print("Pulsos totales contados: ");
      Serial.println(Pulsos);
      Serial.print("Resolución estimada (PPR): ");
      Serial.println(Resolucion);
      Serial.println("--------------------------------");
    }
  }
}
