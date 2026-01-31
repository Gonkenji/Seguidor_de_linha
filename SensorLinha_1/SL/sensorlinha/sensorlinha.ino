// Define os pinos de controle do multiplexador e o pino de entrada analogica
const int ADC_PIN = 28;  // Pino analogico para ler o valor do sensor
const int S0_PIN = 13;   // Pino de controle A (Pino 11 do CD4051B)
const int S1_PIN = 14;   // Pino de controle B (Pino 10 do CD4051B)
const int S2_PIN = 15;   // Pino de controle C (Pino 9 do CD4051B)

// Array para armazenar os valores de cada sensor
int sensorValues[8];
int sensorValuesA[8];

void setup() {
  // Inicia a comunicacao serial para exibir os dados no Plotter Serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("Pronto para plotar todos os 8 sensores.");

  // Configura os pinos de controle do multiplexador como saida
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);

  // Configura o pino analogico como entrada
  pinMode(ADC_PIN, INPUT);
}

void loop() {
  // 1. Leitura dos 8 sensores
  for (int channel = 0; channel < 8; channel++) {
    // Seleciona o canal do multiplexador.
    digitalWrite(S0_PIN, (channel >> 0) & 0x01);
    digitalWrite(S1_PIN, (channel >> 1) & 0x01);
    digitalWrite(S2_PIN, (channel >> 2) & 0x01);

    // Aguarda um pequeno tempo para o multiplexador estabilizar
    delay(5);
    
    // Le o valor do sensor no canal selecionado e armazena no array
    sensorValues[channel] = analogRead(ADC_PIN);
    
  }

  // 2. Impressao dos valores para Plotagem
  
  // Imprime os 8 valores do array na mesma linha
  for (int channel = 0; channel < 8; channel++) {
    Serial.print(sensorValues[channel]*0.5 + sensorValuesA[channel]*0.5);
    sensorValuesA[channel] = sensorValues[channel];
    Serial.print('\t'); // Usa tabulacao para separar os valores
  }
  
  Serial.println(); // Pula uma linha no final de cada varredura completa

  // Atraso para nao sobrecarregar a porta serial
  delay(50);
}
