volatile long Pulsos = 0;
bool Contando = false;
int PWM = 100;

void setup() {
  Serial.begin(9600);

  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), Contador, RISING);

  // Pin PWM del motor
  pinMode(6, OUTPUT);

  Serial.println("Escribe 'inicio' para comenzar. El motor girará lentamente.");
  Serial.println("Cuando dé 100 vueltas, escribe 'fin' para calcular la resolución.");
}

void loop() {
  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando.equalsIgnoreCase("inicio")) {
      encoderPulses = 0;
      contando = true;
      analogWrite(6, PWM);
      Serial.println("Contando pulsos... Escribe 'fin' cuando dé 100 vueltas.");
    }

    else if (comando.equalsIgnoreCase("fin")) {
      Contando = false;
      analogWrite(6, 0);

      float Resolucion = (float)Pulsos / 100;

      Serial.println("---------- RESULTADO ----------");
      Serial.print("Pulsos totales contados: ");
      Serial.println(Pulsos);
      Serial.print("Resolución estimada (PPR): ");
      Serial.println(Resolucion);
      Serial.println("--------------------------------");
    }
  }
}

void Contador() {
  if (Contando) {
    Pulsos++;
  }
}