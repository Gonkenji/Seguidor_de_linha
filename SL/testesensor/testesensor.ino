const int adcteste = 27;

void setup() {
  // Inicia a comunicacao serial para exibir os dados no Plotter Serial
  Serial.begin(115200);
}

void loop() {
  Serial.println(analogRead(adcteste));

  // Atraso para nao sobrecarregar a porta serial
  delay(50);
}
