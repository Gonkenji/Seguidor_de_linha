// Define os pinos de controle do multiplexador e o pino de entrada analogica
const int ADC_PIN = 28;  // Pino analogico para ler o valor do sensor
const int S0_PIN = 16;   // Pino de controle A (Pino 11 do CD4051B)
const int S1_PIN = 17;   // Pino de controle B (Pino 10 do CD4051B)
const int S2_PIN = 18;   // Pino de controle C (Pino 9 do CD4051B)

const int Bcalibrar = 20; // Botão de calibragem

const int PWMA = 0;   // GPIO 0
const int AIN1 = 3;   // GPIO 3
const int AIN2 = 2;   // GPIO 2

const int PWMB = 1;   // GPIO 1
const int BIN1 = 4;   // GPIO 4
const int BIN2 = 5;   // GPIO 5

const int ledplaca = 25;

bool SLcalibrado = 0;

float escala[8][2];

int* dadosSL(void);

void calibragemSL(void);

void printSL(void);

void setMotorDirection(int,int, int);

void setMotorSpeed(int,int);

void controle_P(void);

void setup() {
  // Inicia a comunicacao serial para exibir os dados no Plotter Serial
  Serial.begin(115200);
  delay(1000);

  // Configura os pinos de controle do multiplexador como saida
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);

  // Configura o pino analogico como entrada
  pinMode(ADC_PIN, INPUT);
  setMotorDirection(AIN1, AIN2, 1);
  setMotorDirection(BIN1, BIN2, 1);
  digitalWrite(ledplaca, 1);
}

void loop() {

  if (digitalRead(Bcalibrar) == 1){
    while (digitalRead(Bcalibrar)){
    } 
      calibragemSL();
  }
  if (SLcalibrado){
    controle_P();
  }
  else {
  printSL();
  }
}

void calibragemSL(){
  int calibrado = 0;
  int* sensor = dadosSL();
  int maxmin[8][2]; 
  int sensorA[8];

  digitalWrite(ledplaca, !digitalRead(ledplaca));

  for (int channel = 0; channel < 8; channel++)
    maxmin[channel][0] = maxmin[channel][1] = sensorA[channel] = sensor[channel];

  while (calibrado < 2000){
    dadosSL();

    for (int channel = 0; channel < 8; channel++) {
      sensor[channel] = sensorA[channel]*0 + sensor[channel]*1;

      if (sensor[channel] < maxmin[channel][0]) 
        maxmin[channel][0] = sensor[channel];
      
      else if (sensor[channel] > maxmin[channel][1])
        maxmin[channel][1] = sensor[channel];
      
      else
        calibrado++;
    }    
    Serial.println(maxmin[0][0]);
    Serial.println(maxmin[0][1]);
    Serial.println("");
  }

  for (int channel = 0; channel < 8; channel++){
    escala[channel][0] = 1000.0/(maxmin[channel][1] - maxmin[channel][0]);
    escala[channel][1] = maxmin[channel][0];
  }

  SLcalibrado = 1;
  for (int channel = 0; channel < 8; channel++){
    Serial.println(maxmin[channel][0]);
    Serial.println(maxmin[channel][1]);
    Serial.println(escala[channel][0]);
    Serial.println(escala[channel][1]);
    Serial.println("");
  }
  digitalWrite(ledplaca, !digitalRead(ledplaca));
  delay(1000);
}

int* dadosSL(){
  static int sensor[8];
  for (int channel = 0; channel < 8; channel++) {
      // Seleciona o canal do multiplexador.
      digitalWrite(S0_PIN, (channel >> 0) & 0x01);
      digitalWrite(S1_PIN, (channel >> 1) & 0x01);
      digitalWrite(S2_PIN, (channel >> 2) & 0x01);

      // Aguarda um pequeno tempo para o multiplexador estabilizar
      delay(10);
      
      // Le o valor do sensor no canal selecionado e armazena no array
      sensor[channel] = analogRead(ADC_PIN);
    }
  return sensor;
}

void printSL(){
  static int sensorA[8];
  int* rawsensor = dadosSL();
  int Satual;
  int Slfiltrado;

  static bool init = true;

  if (init){
    for (int channel = 0; channel < 8; channel++) {
      init = false;
      sensorA[channel] = rawsensor[channel];
    }
  }

  // Imprime os 8 valores do array na mesma linha
  for (int channel = 0; channel < 8; channel++) {
    if (SLcalibrado){
      Satual = escala[channel][0]*(rawsensor[channel]-escala[channel][1]);
      Slfiltrado = Satual*1 + sensorA[channel]*0;
      Serial.print(Slfiltrado);
      Serial.print('\t');
      sensorA[channel] = Slfiltrado;
    }
    else{
      Satual = rawsensor[channel];
      Slfiltrado = Satual*1 + sensorA[channel]*0;
      Serial.print(Slfiltrado);
      sensorA[channel] = rawsensor[channel];
      Serial.print('\t'); // Usa tabulacao para separar os valores
    }
  }

  Serial.println(); // Pula uma linha no final de cada varredura completa

  // Atraso para nao sobrecarregar a porta serial
  delay(50);
}

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

void controle_P(){
  int velA;
  int velB;
  int vel = 50;
  int *rawsensor = dadosSL();
  int Satual;
  int Slfiltrado;
  static int sensorA[8];

  int errod;
  int erroe;
  int erro;

  int refM = 1000;

  int refL = 0;

  static float kp = 0.1;

  static bool init = true;

  if (init){
    for (int channel = 0; channel < 8; channel++) {
      init = false;
      sensorA[channel] = rawsensor[channel];
    }
  }

  for (int channel = 0; channel < 8; channel++) {
    Satual = escala[channel][0]*(rawsensor[channel]-escala[channel][1]);
    Slfiltrado = Satual*1 + sensorA[channel]*0;
    sensorA[channel] = Slfiltrado;
  }

  errod = refL - sensorA[2] + refM - sensorA[3];
  erroe = refL - sensorA[5] + refM - sensorA[4];
  erro = errod - erroe;

  if (erro < 0){
    velB = abs(vel + kp*erro);
    velA = abs(vel - kp*erro);
  }
  else{
    velB = abs(vel - kp*erro);
    velA = abs(vel + kp*erro);
  }

  setMotorSpeed(PWMA, velA); //esquerda
  setMotorSpeed(PWMB, velB); //direita

  Serial.print(errod);
  Serial.print('\t');
  Serial.print(erroe);
  Serial.print('\t');
  Serial.print(erro);
  Serial.print('\t');
  Serial.print(velB);
  Serial.print('\t');
  Serial.print(velA);
  Serial.print('\t');
  Serial.println();
}