const int PWMA = 0;   // GPIO 0
const int AIN1 = 3;   // GPIO 3
const int AIN2 = 2;   // GPIO 2

const int PWMB = 1;   // GPIO 1
const int BIN1 = 4;   // GPIO 4
const int BIN2 = 5;   // GPIO 5

// Velocidade Base de Teste (Valor de 0 a 255)
const int TEST_SPEED = 50;

// Funcao para configurar o sentido de um motor (A ou B)
void setMotorDirection(int in1, int in2, int dir) {
  if (dir == 1) { // Sentido Horario (Forward)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) { // Sentido Anti-horario (Reverse)
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else { // Parar ou Freio (Short Brake)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH); // Ambos HIGH = Freio Rapido (TB6612FN)
  }
}

// Funcao para configurar a velocidade de um motor (A ou B)
void setMotorSpeed(int pwmPin, int speed) {
  analogWrite(pwmPin, speed); // PWM de 0 a 255
}


void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
}

void loop() {
  setMotorDirection(BIN1, BIN2, 1); // Motor B: Sentido 1 (Frente)
  setMotorSpeed(PWMB, TEST_SPEED);   // Motor B: Velocidade de teste

  setMotorDirection(AIN1, AIN2, 1);  // Motor A: Sentido 1 (Frente)
  setMotorSpeed(PWMA, TEST_SPEED);   // Motor A: Velocidade de teste
  
  delay(2000);

  setMotorDirection(AIN1, AIN2, 0); // Motor A: parar
  setMotorSpeed(PWMA, 0);
  
  setMotorDirection(BIN1, BIN2, 0);  // Motor B: parar
  setMotorSpeed(PWMB, 0);
  
  delay(5000);
}