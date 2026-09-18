#include <Arduino.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>
#include <Preferences.h>

#define BOOT_BUTTON 0
// variaveis goblais
BluetoothSerial SerialBT;
String btCmd = "";
bool running = false; // O seguidor de linha só anda ao digitar o cmd "START" pelo Bluetooth
bool testeSensor = false;
bool sensorUnitario = false;
bool bootPressionado = false;
unsigned long tempoPressionado = 0;

const int LED = 2;
// configurações da memoria
Preferences preferences;
bool calibracaoM = false;
bool corDaLinha = false; // false para linha preta, true para linha branca.
// Sensor
QTRSensors qtr;
const uint8_t SensorCount = 8;
unsigned short qtrValues[SensorCount];
// Ponte H
// Motor A
const int IN1 = 25;
const int IN2 = 26;
// Motor B
const int IN3 = 27;
const int IN4 = 32;

// Parâmetros PID
float KP = 2.5;
float KI = 0.05;
float KD = 5.0;

float VelMax = 225;
int SetPoint = 3500;

int ERRO = 0;
int ERRO_ANTERIOR = 0;
float Proporcional = 0, Integral = 0, Derivativo = 0, Velocidade = 0;

void setup() {
  //Inicializando
  Serial.begin(115200);
  delay(1000);
  SerialBT.begin("Egg_line"); // Nome que o seguidor ira aparecer na conexão bluetooth
  SerialBT.println("Iniciando...");
  digitalWrite(LED, LOW);
  delay(50);
  digitalWrite(LED, HIGH);
  delay(50);
  digitalWrite(LED, LOW);
  delay(50);
  digitalWrite(LED, HIGH);
  
  //Setando pinos
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(LED,OUTPUT);
  stop_motor();

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){13,14,16,17,18,19,21,22}, SensorCount);
  delay(2000);
  preferences.begin("memoria", true); // le os dados, caso não tenha dado utiliza os proprios valores
  KP = preferences.getFloat("kp", KP);
  KI = preferences.getFloat("ki", KI);
  KD = preferences.getFloat("kd", KD);
  VelMax = preferences.getFloat("velMax", VelMax);
  calibracaoM = preferences.getBool("calibracaoStatus", false);

  preferences.end();

  SerialBT.println("Parametros inicias:");
  SerialBT.print("KP: ");
  SerialBT.println(KP);
  SerialBT.print("KI: ");
  SerialBT.println(KI);
  SerialBT.print("KD: ");
  SerialBT.println(KD);

  //Adicionar controle de decisão aqui para saber se 
  if(calibracaoM == false){
    //Calibração
    SerialBT.println("CALIBRANDO... mova o sensor sobre a linha");
    for (uint16_t i = 0; i < 400; i++){
     qtr.calibrate();
      delay(5);
      // posteriormente adicionar lad
    }
    for(int i = 0; i <= 6; i++){
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, LOW);
      delay(50);
    }
    digitalWrite(LED, HIGH);
    preferences.begin("memoria", false);
    // guardando dados da calibração
    preferences.putBytes("cal_min", qtr.calibrationOn.minimum, sizeof(qtr.calibrationOn.minimum));
    preferences.putBytes("cal_max", qtr.calibrationOn.maximum, sizeof(qtr.calibrationOn.maximum));
    preferences.putBool("calibracaoStatus", true);
    preferences.end();
    delay(150);

    SerialBT.println("Calibração Concluida!");
    SerialBT.println("Qual tipo da pista? linha: (Preto/Branco)"); // precisa identificar qual o tipo?
  } else {
    // fazendo leitura de dados salvos na memoria;
    digitalWrite(LED, HIGH);
    preferences.begin("memoria", true);
    preferences.getBytes("cal_min", qtr.calibrationOn.minimum, sizeof(qtr.calibrationOn.minimum));
    preferences.getBytes("cal_max", qtr.calibrationOn.maximum, sizeof(qtr.calibrationOn.maximum));
    corDaLinha = preferences.getBool("corDaLinha", corDaLinha);
    preferences.end();
    delay(2000);
    running = true;
  }
}

void loop() {
  verificarBotao();
  if (running) {
    run_robot();
  }else {
    stop_motor();
  }
  // Provisorio para depuração e coleta de dados do sensor;
  if (testeSensor && !running){
    stop_motor();
    qtr.read(qtrValues);
    if(sensorUnitario){
      for(int i = 0; i < SensorCount; i++){
          SerialBT.print("S");
          SerialBT.print(i + 1);
          SerialBT.print(": ");
          SerialBT.print(qtrValues[i]);
          SerialBT.print("\t");
        }
        SerialBT.println();
        delay(300);
    } else{
      if(corDaLinha){
        SerialBT.print(qtr.readLineWhite(qtrValues));
      } else{
        SerialBT.print(qtr.readLineBlack(qtrValues));
      }
    }

  }
  // verifica mensagem do bluetooth
  while (SerialBT.available()){
    char c = SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (btCmd.length() > 0) {
        processamentoCmdBluetooth(btCmd);
        btCmd = "";
      }
    } else {
      btCmd += c;
    }
  }

}
// modifica está logica
void run_fwd(int speedL, int speedR) {
  if(speedL >= 0){
    analogWrite(IN1, speedL);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, abs(speedL));
  }
  if(speedR >=0) 
  {
    analogWrite(IN3, speedR);
    analogWrite(IN4, 0);
  } else{
    analogWrite(IN3, 0);
    analogWrite(IN4, abs(speedR));
  }
}

void stop_motor() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void run_robot() {
  int position = 0;
  uint8_t sensorIntersecao = 0;
  if (corDaLinha){
    //true le linha branca
    // o sensor está configurado para identificar a linha na cor branca.
    position = qtr.readLineWhite(qtrValues);
  } else {
    // o sensor está configurado para identifcar a linha na cor preta.
    position = qtr.readLineBlack(qtrValues);
  }
  // identifica se é intersecção
  for (uint8_t i = 0; i < 8 ;i++){
    if (qtrValues[i] < 200 && corDaLinha) {
      sensorIntersecao++;
    } else if (qtrValues[i] > 700 && !corDaLinha){
      sensorIntersecao++;
    }
  }
  if (sensorIntersecao >= 6)
  {
    run_fwd(VelMax, VelMax);
  }
  else{
    // calculo do erro
    ERRO = position - SetPoint;
    Proporcional = ERRO * KP;
    Integral += (ERRO + ERRO_ANTERIOR) * KI;
    Derivativo = (ERRO - ERRO_ANTERIOR) * KD;

    Velocidade = Proporcional + Integral + Derivativo;
    if (Velocidade > VelMax) {
        Velocidade = VelMax;
    } else if (Velocidade < -VelMax) {
        Velocidade = -VelMax;
    }

    ERRO_ANTERIOR = ERRO;

    int speedLeft = VelMax + Velocidade;
    int speedRight = VelMax - Velocidade;

    speedLeft = constrain(speedLeft, -VelMax, VelMax);
    speedRight = constrain(speedRight, -VelMax, VelMax);
    run_fwd(speedLeft, speedRight);
  }
}

void processamentoCmdBluetooth(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd == "BRANCO"){
    corDaLinha = true;
    SerialBT.println("Robô Pronto! Digite START para rodar.");
  }
  else if (cmd == "PRETO"){
    corDaLinha = false;
    SerialBT.println("Robô Pronto! Digite START para rodar.");
  }
  else if (cmd == "START"){
    running = true;
    SerialBT.println("Rodando!");
  }
  else if (cmd == "STOP") {
    running = false;
    SerialBT.println("Parado!");
  }
  else if (cmd.startsWith("KP ")){
    float val = cmd.substring(3).toFloat();
    if (val >= 0) {
      KP = val;
      SerialBT.print("KP alterado para: "); SerialBT.println(KP);
    }
  }
  else if (cmd.startsWith("KD ")){
    float val = cmd.substring(3).toFloat();
    if (val >= 0) {
      KD = val;
      SerialBT.print("KD alterado para: "); SerialBT.println(KD);
    }
  }
  else if (cmd.startsWith("KI ")){
    float val = cmd.substring(3).toFloat();
    if (val >= 0) {
      KI = val;
      SerialBT.print("KI alterado para: "); SerialBT.println(KI);
    }
  }
  else if (cmd.startsWith("VEL ")){
    float val = cmd.substring(4).toFloat();
    if (val >=0 && val <=255) {
      VelMax = val;
      SerialBT.print("Velocidade máxima alterada para: "); SerialBT.println(VelMax);
    } else {
      VelMax = 255;
      SerialBT.print("Velocidade máxima alterada para: 255 (maximo)");
    }
  }
  // teste de sensor
  else if (cmd == "TS"){
    running = false;
    if(testeSensor == false){
      testeSensor = true;
    } else {
      testeSensor = false;
    }
  }
  else if (cmd == "TSU"){
    running = false;
    if(testeSensor == false){
      testeSensor = true;
      sensorUnitario = true;
    } else{
      testeSensor = false;
      sensorUnitario = false;
    }
  }
  // salvando parametros na memoria flash
  else if (cmd == "SALVAR"){
    salva_memoria();
  }
  else{
    SerialBT.printf("KP atual: %.2f | KI atual: %.2f | KD atual: %.2f | Velmax atual: %.2f", KP, KI, KD, VelMax);
  }
}

void salva_memoria(){
  preferences.begin("memoria", false);
    preferences.putFloat("kp", KP);
    preferences.putFloat("ki", KI);
    preferences.putFloat("kd", KD);
    preferences.putFloat("velMax", VelMax);
    preferences.putBool("corDaLinha", corDaLinha);
    preferences.end();
}

void verificarBotao(){
  bool pressionado = (digitalRead(BOOT_BUTTON) == LOW);

  if (pressionado && !bootPressionado){
    tempoPressionado = millis();
    bootPressionado = true;
  }
  else if (!pressionado && bootPressionado){
    bootPressionado = false;
  }
  if (pressionado && bootPressionado){
    if((millis() - tempoPressionado) >= 5000){
      preferences.begin("memoria", false);
      preferences.remove("calibracaoStatus");
      preferences.end();
      bootPressionado = false;
      pressionado = false;
      SerialBT.println("Aperte o botão reset do esp para recalibrar");
    }
  }
}