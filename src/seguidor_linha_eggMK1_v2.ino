#include <Arduino.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>

// variaveis goblais
BluetoothSerial SerialBT;
String btCmd = "";
bool running = false; // O seguidor de linha só anda ao digitar o cmd "START" pelo Bluetooth
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
float KP = 2.0;
float KI = 0.0;
float KD = 5.0;

float VelMax = 200;
int SetPoint = 3500;

int ERRO = 0;
int ERRO_ANTERIOR = 0;
float Proporcional = 0, Integral = 0, Derivativo = 0, Velocidade = 0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Egg_line"); // Nome que o seguidor ira aparecer na conexão bluetooth
  SerialBT.println("Iniciando...");
  SerialBT.println("Parametros inicias:");
  SerialBT.print("KP: ");
  SerialBT.println(KP);
  SerialBT.print("KI: ");
  SerialBT.println(KI);
  SerialBT.print("KD: ");
  SerialBT.println(KD);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stop_motor();

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){13,14,16,17,18,19,21,22}, SensorCount);

  //Calibração
  SerialBT.println("CALIBRANDO... mova o sensor sobre a linha");
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
    delay(10);
    // posteriormente adicionar lad
  }
  SerialBT.println("Calibração Concluida!");
  SerialBT.println("Qual tipo da pista? linha: (Preto/Branco)"); // precisa identificar qual o tipo?
  
}

void loop() {
  if (running) {
    run_robot();
  }else {
    stop_motor();
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
  analogWrite(IN1, speedL);
  analogWrite(IN2, 0);

  analogWrite(IN3, speedR);
  analogWrite(IN4, 0);
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
    if (qtrValues[i] < 200) {
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

    speedLeft = constrain(speedLeft, 0, VelMax);
    speedRight = constrain(speedRight, 0, VelMax);
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
      SerialBT.print("Velocidade máxima alterada para: 225 (maximo)");
    }
  }
  else{
    SerialBT.printf("KP atual: %.2f | KI atual: %.2f | KD atual: %.2f | Velmax atual: %.2f", KP, KI, KD, VelMax);
  }
}