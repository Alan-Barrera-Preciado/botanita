int MotorSH = 6;
int MotorSA = 5;

int Resolucion = 800;
int Encoder = 2;
volatile byte Pulsos = 0;
int PulsosIntervalo = 0;

float RPM = 0;
int Aux = 1;
float Rad = 0;
float t = 0;

void setup() {
  Serial.begin(115200);
  pinMode(MotorSH, OUTPUT);
  pinMode(MotorSA, OUTPUT);
  analogWrite(MotorSH, 0);
  analogWrite(MotorSA, 0);
  pinMode(Encoder, INPUT);
  attachInterrupt(digitalPinToInterrupt(Encoder), Contador, RISING);
}

void loop() {
  if (Serial.available()) {
    while(Aux == 1){
      delay(50);
      if(t < 2){
        analogWrite(MotorSH, 0);
      }
      else if(t < 4 && t >= 2){
        analogWrite(MotorSH, 255);
      }
      else if(t < 6 && t >= 4){
        analogWrite(MotorSH, 0);
      }
      else if(t < 8 && t >= 6){
        analogWrite(MotorSH, 200);
      }
      else if(t < 10 && t >= 8){
        analogWrite(MotorSH, 0);
      }
      else if(t < 12 && t >= 10){
        analogWrite(MotorSH, 150);
      }
      else if(t < 14 && t >= 12){
        analogWrite(MotorSH, 0);
      }
      else if(t < 16 && t >= 14){
        analogWrite(MotorSH, 100);
      }
      else if(t < 18 && t >= 16){
        analogWrite(MotorSH, 0);
      }
      else if(t < 20){
        analogWrite(MotorSH, 50);
      }
      RPM = CalcularRPM();
      Rad = RPM*0.10472;
      String cad = Serial.readStringUntil('\n');
      Serial.println(Rad);
      Aux = cad.toInt();
      t = t + 0.05;
    }
    analogWrite(MotorSH, 0);
  }
}

void Contador(){
  Pulsos++;
}

float CalcularRPM(){
  noInterrupts();
  PulsosIntervalo = Pulsos;
  Pulsos = 0;
  RPM = (PulsosIntervalo/0.05)*60/Resolucion;
  interrupts();
  return RPM;
}