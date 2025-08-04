volatile long Pulsos = 0;
bool Contando = false;
int PWM = 100;

// Izquierdo Resolucion = 800
// Derecho Resolucion = 700

const int encoderPin = 3;
const int pwmPin = 9;

void Contador() {
  if (Contando) {
    Pulsos++;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(pwmPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), Contador, RISING);

  Serial.println("Escribe 'inicio' para comenzar. El motor girará lentamente.");
  Serial.println("Cuando dé 10 vueltas, escribe 'fin' para calcular la resolución.");
}

void loop() {
  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando.equalsIgnoreCase("inicio")) {
      Pulsos = 0;
      Contando = true;
      analogWrite(pwmPin, 100);
      Serial.println("Contando pulsos... Escribe 'fin' cuando dé 100 vueltas.");
    }
    else if (comando.equalsIgnoreCase("fin")) {
      Contando = false;
      analogWrite(pwmPin, 0);

      float Resolucion = (float)Pulsos / 10.0;
      Serial.println("---------- RESULTADO ----------");
      Serial.print("Pulsos totales contados: ");
      Serial.println(Pulsos);
      Serial.print("Resolución estimada (PPR): ");
      Serial.println(Resolucion);
      Serial.println("--------------------------------");
    }
  }
}
