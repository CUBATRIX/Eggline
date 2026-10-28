#include <Arduino.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>
#include <Preferences.h>

// variaveis goblais
BluetoothSerial SerialBT;
String btCmd = "";
bool running = false; // O seguidor de linha só anda ao digitar o cmd "START" pelo Bluetooth
bool testeSensor = false;
bool sensorUnitario = false;

const int LED = 2;
// configurações da memoria
Preferences preferences;
int calibracaoM = 0;
int corDaLinha = 0; // 0 para linha preta, 1 para linha branca.
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
// Parametro de Compensação;
float MP = 1;
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
  calibracaoM = preferences.getInt("calibracao", 0);
  SerialBT.print("CALIBRACAO LIDA NO BOOT: ");
  SerialBT.println(calibracaoM ? "1" : "0");

  preferences.end();

  SerialBT.println("Parametros inicias:");
  SerialBT.print("KP: ");
  SerialBT.println(KP);
  SerialBT.print("KI: ");
  SerialBT.println(KI);
  SerialBT.print("KD: ");
  SerialBT.println(KD);
  digitalWrite(LED, HIGH);
  delay(150);
  digitalWrite(LED, LOW);

  //Adicionar controle de decisão aqui para saber se 
  if(calibracaoM == 0){
    //Calibração
    digitalWrite(LED, HIGH);
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
    calibracaoM = 1;
    delay(150);

    SerialBT.println("Calibração Concluida!");
    SerialBT.println("Qual tipo da pista? linha: (Preto/Branco)"); // precisa identificar qual o tipo?
  } else {
    // fazendo leitura de dados salvos na memoria;
    digitalWrite(LED, LOW);
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    preferences.begin("memoria", true);
    preferences.getBytes("cal_min", qtr.calibrationOn.minimum, sizeof(qtr.calibrationOn.minimum));
    preferences.getBytes("cal_max", qtr.calibrationOn.maximum, sizeof(qtr.calibrationOn.maximum));
    corDaLinha = preferences.getInt("corDaLinha", corDaLinha);
    preferences.end();
    delay(2000);
    running = true;
  }
}

void loop() {
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
      if(corDaLinha == 1){
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
    analogWrite(IN1, speedL*MP);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, abs(speedL*MP));
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
  if (corDaLinha == 1){
    //true le linha branca
    // o sensor está configurado para identificar a linha na cor branca.
    position = qtr.readLineWhite(qtrValues);
  } else {
    // o sensor está configurado para identifcar a linha na cor preta.
    position = qtr.readLineBlack(qtrValues);
  }
  // identifica se é intersecção
  for (uint8_t i = 0; i < 8 ;i++){
    if (qtrValues[i] < 200 && (corDaLinha == 1)) {
      sensorIntersecao++;
    } else if (qtrValues[i] > 700 && (corDaLinha == 0)){
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
    Integral = (ERRO + ERRO_ANTERIOR) * KI;
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
  // ----------------- CALIBRAÇÃO ----------------------------
  if (cmd == "BRANCO"){
    corDaLinha = 1;
    SerialBT.println("Robô Pronto! Digite START para rodar.");
  }
  else if (cmd == "PRETO"){
    corDaLinha = 0;
    SerialBT.println("Robô Pronto! Digite START para rodar.");
  }
  // ----------------- CALIBRAÇÃO ----------------------------
  // ----------------- COMANDOS DE AÇÃO E PARAMETROS ----------------------------
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
  else if (cmd.startsWith("MP ")){
    float val = cmd.substring(3).toFloat();
    if (val >= 0){
      MP = val;
      SerialBT.print("MP alterado para: "); SerialBT.println(MP);
    }
  }
  // ----------------- COMANDOS DE AÇÃO E PARAMETROS ----------------------------
  // ----------------- COMANDOS DE TESTE  ----------------------------
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
  else if ( cmd == "RFW"){
    // roda os motores para frente na velocidade maxima configurada
    for ( int i = 0; i <= 100; i++){
      analogWrite(IN1, VelMax*MP);
      analogWrite(IN2, 0);

      analogWrite(IN3, VelMax);
      analogWrite(IN4, 0);
    }
  }
  else if (cmd == "RBW"){
    // roda os motores para trás na velocidade maxima
    for (int i = 0; i <= 100; i++){
      analogWrite(IN1, 0);
      analogWrite(IN2, VelMax*MP);

      analogWrite(IN3, 0);
      analogWrite(IN4, VelMax);
    }
  }
  // ----------------- COMANDOS DE SALVAR E RESET DE CALIBRAÇÃO ----------------------------
  else if (cmd == "SALVAR"){
    salva_memoria();
  }
  else if (cmd == "RESET"){
    preferences.begin("memoria", false);
    preferences.remove("calibracao");
    preferences.end();
    SerialBT.println("Desligue e ligue o seguidor de linha para recalibrar!");
  }
  // ----------------- RESPOSTA PADRÃO --------------------------------
  else{
    SerialBT.printf("KP atual: %.2f | KI atual: %.2f | KD atual: %.2f | Velmax atual: %.2f", KP, KI, KD, VelMax);
  }
}

void salva_memoria(){
  preferences.begin("memoria", false);
    preferences.putFloat("kp", KP);
    preferences.putFloat("ki", KI);
    preferences.putFloat("kd", KD);
    preferences.putFloat("mp", MP);
    preferences.putFloat("velMax", VelMax);
    preferences.putInt("corDaLinha", corDaLinha);
    preferences.putBytes("cal_min", qtr.calibrationOn.minimum, sizeof(qtr.calibrationOn.minimum));
    preferences.putBytes("cal_max", qtr.calibrationOn.maximum, sizeof(qtr.calibrationOn.maximum));
    preferences.putInt("calibracao", calibracaoM);
    SerialBT.print("CALIBRACAO SALVA: ");
    SerialBT.println(
    preferences.getBool("calibracao", 0)
        ? "1"
        : "0"
    );
    SerialBT.print("KP: ");
    SerialBT.println(preferences.getFloat("kp", 0));

    SerialBT.print("KI: ");
    SerialBT.println(preferences.getFloat("ki", 0));

    SerialBT.print("KD: ");
    SerialBT.println(preferences.getFloat("kd", 0));

    SerialBT.print("MP: ");
    SerialBT.println(preferences.getFloat("mp", 0));

    SerialBT.print("VelMax: ");
    SerialBT.println(preferences.getFloat("velMax", 0));

    SerialBT.print("Cor da linha: ");
    SerialBT.println(preferences.getInt("corDaLinha", 0));

    SerialBT.print("Calibracao: ");
    SerialBT.println(preferences.getInt("calibracao", 0));

    preferences.end();
}
