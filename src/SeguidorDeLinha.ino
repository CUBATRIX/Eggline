#include <QTRSensors.h>
#include <BluetoothSerial.h>
#include <Preferences.h>

// Setando pinos
// Led interno para Avisos
#define LED 2
// ----- Sensor ------
#define S0 13
#define S1 14
#define S2 16
#define S3 17
#define S4 18
#define S5 19
#define S6 21
#define S7 22

// ----- Sensor Lateral ----
#define SLE 34
#define SLD 35

// ---- Pinos para ponte H ----
// Motor A - Direito
#define M1 25
#define M2 26
// Motor B - Esquerdo
#define M3 27
#define M4 32

// ---- Parametros é variaveis -----
Preferences preferences;

// Sensor
QTRSensors qtr;
const uint8_t SensorCount = 8;
unsigned short qtrValues[SensorCount];
bool corDaPista = false; // false = linha branca : true = linha preta

// Bluetooth
BluetoothSerial SerialBT;
String btCmd = "";
// Motores
int velocidade_max = 180;
int velocidade_max_curva = 80; // ajustar ainda.

// Estados
bool run = false;

// Sensor lateral direito
int THRESHOLD_LATERAL = 2000;
int contadorMarcasAtual = 0;
int contadorMarcas = 0;
bool marcaCapturada = false;

// Variaveis de temporização para parada
unsigned long temporizador = 0;
unsigned long tempoLimite = 0;
bool temporizadorAtivo = false;
 
// PID
const int SetPoint = 3500;
int ERRO = 0;
int ERRO_ANTERIOR = 0;
float KP = 5;
float KI = 0.1;
float KD = 2;
float Proporcional = 0, Integral = 0, Derivativo = 0, Velocidade = 0;
// variaveis para modo de curva e sensor esquerdo;
float KPC = 15;
float KDC = 10;
bool curva = false;
int tempoMinimoCurva = 300;
int tempoInicioCurva = 0;
// Variaveis para intrepretação de intersecção
int S_inter = 0;
bool PID = true;
// ---- Parametrôs para teste -----
bool TS = false;
bool TSU = false;
bool TSD = false;


void setup() {
  Serial.begin(115200);
  SerialBT.begin("Egg");
  // Inicializando pinos
  pinMode(LED, OUTPUT);
  pinMode(M1, OUTPUT); pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT); pinMode(M4, OUTPUT);
  delay(2000);
  // obetando dados da memoria
  preferences.begin("memoria", true);
  KP = preferences.getFloat("kp", KP);
  KI = preferences.getFloat("ki", KI);
  KD = preferences.getFloat("kd", KD);
  KPC = preferences.getFloat("kpc", KPC);
  KDC = preferences.getFloat("kdc", KDC);
  contadorMarcas = preferences.getInt("cm", contadorMarcas);
  velocidade_max = preferences.getInt("vm", velocidade_max);
  velocidade_max_curva = preferences.getInt("vmc", velocidade_max_curva);
  THRESHOLD_LATERAL = preferences.getInt("ths", THRESHOLD_LATERAL);
  preferences.end();
  SerialBT.println("KP: " + String(KP) + " | KI: " + String(KI) + " | KD: " + String(KD) + " | KPC: " + String(KPC) + " | KDC " + String(KDC));
  SerialBT.println("Velocidade_max: " + String(velocidade_max) + " | Velocidade_max_curva: " + String(velocidade_max_curva));
  SerialBT.println("ContadorMarcas: " + String(contadorMarcas) + " | Threshold_lateral: " + String(THRESHOLD_LATERAL));

  // configurando sensor e calibração inicial
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){S0,S1,S2,S3,S4,S5,S6,S7}, SensorCount);
  SerialBT.println("Iniciando calibração");
  digitalWrite(LED,HIGH);
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
    delay(10);
  }
  digitalWrite(LED,LOW);

  SerialBT.println("Calibrado com sucesso");
  SerialBT.println("Defina o tipo de linha: Preto ou Branco");
  // Trocar posteriomente
  while (SerialBT.available() == 0){
    delay(10);
  }

  String texto = SerialBT.readString();
  texto.trim();
  texto.toUpperCase();
  if(texto == "PRETO"){
    SerialBT.println("Pista configurado para linha preta");
    corDaPista = true;
    run = true;
  } else {
    SerialBT.println("Pista configurado para linha Branca");
    corDaPista = false;
    run = true;
  }
}

void loop() {
  // Estados de funcionamento
  if(run){
    // adicionar sequencia de codigos
    run_robot();
  } else {
    // modo de teste de compoentes
    if(TS){
      TesteSensor();
    } else if(TSD){
      TesteSensorDr();
    }
  }
  // Verifica comandos enviadas via bluetooth
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
// Função para comandos bluetooth
void processamentoCmdBluetooth(String cmd){
  cmd.trim();
  cmd.toUpperCase();
  // ---- inicio e parada
  if (cmd == "START"){
    run = true;
    SerialBT.println("Motores a todo vapor!");
  } else if (cmd == "STOP"){
    run= false;
    Stop();
    SerialBT.println("Parado!");
  }else if (cmd == "TS"){
    if(TS == false){
      TS = true;
      Stop();
      SerialBT.println("Iniciando teste de sensor");
    } else if (TS == true){
      TS = false;
      SerialBT.println("Pronto para voltar");
    }
  } // ---- teste de sensores e motores
   else if (cmd == "TSU"){
    if(TSU == false){
      TSU = true;
      SerialBT.println("Teste de sensor configurado para teste unitario");
    } else{
      TSU = false;
      SerialBT.println("Teste de sensor configurado para teste de posição");
    }
  } else if (cmd == "MA"){
    SerialBT.println("Motor girando para frente");
    for(int i = 0; i < 150; i++){
      Andar_Frente();
    }
  } else if (cmd == "MD"){
    SerialBT.println("Motor da direita girando");
    for(int i = 0; i < 150; i++){
      Andar_Direita();
    }
  } else if (cmd == "ME"){
    SerialBT.println("Motor da Esquerda girando");
    for(int i = 0; i < 150; i++){
      Andar_Esquerda();
    }
  } else if (cmd == "TSD"){
    TSD = true;
    TS = false;
    SerialBT.println("Iniciando teste do sensor direito");
    if(TSD == true){
      TSD = false;
    }
  }// ---- mudanças de parametros
  else if(cmd.startsWith("KD ")){
    float val = cmd.substring(3).toFloat();
    if (val >= 0){
      KD = val;
      SerialBT.println("KD alterado para: " + String(KD));
    }
  }else if(cmd.startsWith("KI ")){
    float val = cmd.substring(3).toFloat();
    if (val >= 0){
      KI = val;
      SerialBT.println("KI alterado para: " + String(KI));
    }
  }else if(cmd.startsWith("KP ")){
    float val = cmd.substring(3).toFloat();
    if (val >= 0){
      KP = val;
      SerialBT.println("KP alterado para: " + String(KP));
    }
  }else if(cmd.startsWith("KPC ")){
    float val = cmd.substring(4).toFloat();
    if (val >= 0) {
      KPC = val;
      SerialBT.println("KPC alterado para: " + String(KPC));
    }
  } else if (cmd.startsWith("KDC ")){
    float val = cmd.substring(4).toFloat();
    if(val>= 0){
      KDC = val;
      SerialBT.println("KDC alterado para: " + String(KDC));
    }
  }else if(cmd.startsWith("VELMAX ")){
    int val = cmd.substring(7).toInt();
    if (val >= 0){
      if(val > 255){
        velocidade_max = 255;
        SerialBT.println("Velocidade máxima alterado para: " + String(velocidade_max));
      } else{
        velocidade_max = val;
        SerialBT.println("Velocidade máxima alterado para: " + String(velocidade_max));
      }
    }
  }else if(cmd.startsWith("VELMAXC ")){
    int val = cmd.substring(8).toInt();
    if(val >= 0){
      if(val > 255){
        velocidade_max_curva = 255;
        SerialBT.println("Velocidade máxima de curva alterado para: " + String(velocidade_max));
      } else {
        velocidade_max_curva = val;
        SerialBT.println("Velocidade máxima de curva alterado para: " + String(velocidade_max));
      }
    }
  }else if(cmd.startsWith("TMC ")){
    int val = cmd.substring(4).toInt();
    if (val >= 0){
      tempoMinimoCurva = val;
      SerialBT.println("Tempo minimo para periodo de curva: " + String(tempoMinimoCurva/1000));
    }
  }else if(cmd.startsWith("TL ")){
    int val = cmd.substring(3).toInt();
    if (val >= 0){
      tempoLimite = val;
      SerialBT.println("Tempo limite para parada alterado para: " + String((tempoLimite)/1000) + "s");
    }
  }else if(cmd.startsWith("TSH ")){
    int val = cmd.substring(4).toInt();
    if (val >= 0){
      THRESHOLD_LATERAL = val;
      SerialBT.println("Threshold lateral direito alterado para: " + String(THRESHOLD_LATERAL));
    }
  }else if(cmd.startsWith("CMC ")){
    int val = cmd.substring(4).toInt();
    if (val >= 0){
      contadorMarcas = val;
      SerialBT.println("Qtd de marcações alterado para: " + String(contadorMarcas));
    }
  } else if (cmd == "SALVAR"){
    salvaMemoria();
  }
}
// funções de teste
void TesteSensor(){
  uint16_t position;

  if (corDaPista) {
    position = qtr.readLineBlack(qtrValues);
  } else {
    position = qtr.readLineWhite(qtrValues);
  }

  if (TS && TSU == false) {
    // Exibe a posição da linha 
    SerialBT.println(position);
  } else {
    // Exibe o valor individual de cada refletância lida
    for(int8_t i = 0; i < SensorCount; i++) {
      SerialBT.print("S");
      SerialBT.print(i + 1);
      SerialBT.print(": ");
      SerialBT.print(qtrValues[i]);
      SerialBT.print("\t");
    }
    SerialBT.println();
  }
  
  delay(200);
} 
void TesteSensorDr(){
  int lerSensor = analogRead(SLD);
  SerialBT.println("Valor do sensor direito: " + String(lerSensor));
  delay(100);
}
//funções auxiliar
void Andar_Direita(){
  // Motor direito
  analogWrite(M1, 255);
  analogWrite(M2, 0);
  // Motor Esquerdo
  analogWrite(M3, 0);
  analogWrite(M4, 0);
}
void Andar_Esquerda(){
  // Motor direito
  analogWrite(M1, 0);
  analogWrite(M2, 0);
  // Motor Esquerdo
  analogWrite(M3, 255);
  analogWrite(M4, 0);
}
void Andar_Frente(){
  // Motor direito
  analogWrite(M1, 255);
  analogWrite(M2, 0);
  // Motor Esquerdo
  analogWrite(M3, 255);
  analogWrite(M4, 0);
}
void Stop(){
  run = false;
  // Motor direito
  analogWrite(M1, LOW);
  analogWrite(M2, LOW);
  // Motor Esquerdo
  analogWrite(M3, LOW);
  analogWrite(M4, LOW);
}
void run_fwd(int SL, int SR){
  // Motor direito
  if(SL > 0) {
    analogWrite(M1, SL);
    analogWrite(M2, LOW);
  } else {
    analogWrite(M1, LOW);
    analogWrite(M2, abs(SL));
  }
  // Motor Esquerdo
  if(SR > 0){
    analogWrite(M3, SR);
    analogWrite(M4, LOW);
  }else{
    analogWrite(M3, LOW);
    analogWrite(M4, abs(SR));
  }
}
void SensorEq(){
  int valorSensor = analogRead(SLE);
  int tempoInicioCurva = 0;
  // para linha preta
  if(corDaPista){
    if (valorSensor > THRESHOLD_LATERAL && curva == false){
      curva = true;
      SerialBT.println("Curva detectada");
      tempoInicioCurva = millis();
    } 
  } else { //linha branca
    if (valorSensor < THRESHOLD_LATERAL && curva == false){
      curva = true;
      SerialBT.println("Curva detectada");
      tempoInicioCurva = millis();
    } 
  }
}
void SensorDr(){
  int valorSensor = analogRead(SLD);
  // para linha preta
  if(corDaPista){
    if (valorSensor > THRESHOLD_LATERAL && marcaCapturada == false){
      marcaCapturada = true;
      contadorMarcasAtual ++;
      SerialBT.println("Marca detectada! Contagem: " + String(contadorMarcasAtual));
    } else if (valorSensor < THRESHOLD_LATERAL && marcaCapturada == true){
      marcaCapturada = false;
    } 
  } else { //linha branca
    if (valorSensor < THRESHOLD_LATERAL && marcaCapturada == false){
      marcaCapturada = true;
      contadorMarcasAtual ++;
      SerialBT.println("Marca detectada! Contagem: " + String(contadorMarcasAtual));
    } else if (valorSensor > THRESHOLD_LATERAL && marcaCapturada == true){
      marcaCapturada = false;
    }
  }
}
void identificarInterseccao(){
  S_inter = 0;
  for (int i = 0; i < 8; i ++){
    S_inter += qtrValues[i];
  }
  int media = S_inter/8;
  if(corDaPista == true && media > 600){
    PID = false;
    run_fwd(velocidade_max, velocidade_max);
  }else if(corDaPista == false && media < 400){
    PID = false;
    run_fwd(velocidade_max, velocidade_max);
  } else{
    PID = true;
  }
}
void contagemParada(){
  if(temporizadorAtivo == false){
    temporizador = millis();
    temporizadorAtivo = true;
  }else{
    if(millis() - temporizador > tempoLimite){
      Stop();
    }
  }
}
void salvaMemoria(){
  preferences.begin("memoria", false);
  preferences.putFloat("kp", KP);
  preferences.putFloat("ki", KI);
  preferences.putFloat("kd", KD);
  preferences.putFloat("kpc", KPC);
  preferences.putFloat("kdc", KDC);
  preferences.putInt("cm", contadorMarcas);
  preferences.putInt("vm", velocidade_max);
  preferences.putInt("vmc", velocidade_max_curva);
  preferences.putInt("ths", THRESHOLD_LATERAL);
  preferences.end();
  SerialBT.println("KP: " + String(KP) + " | KI: " + String(KI) + " | KD: " + String(KD) + " | KPC: " + String(KPC) + " | KDC " + String(KDC));
  SerialBT.println("Velocidade_max: " + String(velocidade_max) + " | Velocidade_max_curva: " + String(velocidade_max_curva));
  SerialBT.println("ContadorMarcas: " + String(contadorMarcas) + " | Threshold_lateral: " + String(THRESHOLD_LATERAL));
}
void run_robot(){
  SensorEq();
  SensorDr();
  uint16_t position;
  if (corDaPista) {
    position = qtr.readLineBlack(qtrValues);
  } else {
    position = qtr.readLineWhite(qtrValues);
  }
  // identifica intersecçao
  identificarInterseccao();
  if(PID){
    if(curva == false && contadorMarcasAtual <= contadorMarcas){
      ERRO = position - SetPoint;
      Proporcional = ERRO * KP;
      Integral = (ERRO + ERRO_ANTERIOR) * KI;
      Derivativo = (ERRO - ERRO_ANTERIOR) * KD;

      Velocidade = Proporcional + Integral + Derivativo;
      if (Velocidade > velocidade_max){
        Velocidade = velocidade_max;
      } else if (Velocidade < -velocidade_max){
        Velocidade = -velocidade_max;
      }

      ERRO_ANTERIOR = ERRO;
      if (contadorMarcasAtual <= contadorMarcas){
        int SL = velocidade_max + Velocidade;
        int SR = velocidade_max - Velocidade;

        SL = constrain(SL, -velocidade_max, velocidade_max);
        SR = constrain(SR, -velocidade_max, velocidade_max);

        run_fwd(SL,SR);
      } else {
        contagemParada();
      }
    } else {
      // identifica se o erro esta alto para permanecer no modo de curva
      if(((abs(ERRO_ANTERIOR)) < 150) && (millis() - tempoInicioCurva > tempoMinimoCurva)){
        curva = false;
        SerialBT.println("Modo pista ativado");
      } else if(curva == true){
        SerialBT.println("Modo curva ativado");
        ERRO = position - SetPoint;
        Proporcional = ERRO * KPC;
        Integral = (ERRO + ERRO_ANTERIOR) * 0;
        Derivativo = (ERRO - ERRO_ANTERIOR) * KDC;
        Velocidade = Proporcional + Integral + Derivativo;
        if(Velocidade > velocidade_max_curva){
          Velocidade = velocidade_max_curva;
        } else if (Velocidade < -velocidade_max_curva){
          Velocidade = -velocidade_max_curva;
        }
        ERRO_ANTERIOR = ERRO;
        int SL = velocidade_max_curva + Velocidade;
        int SR = velocidade_max_curva - Velocidade;

        SL = constrain(SL, -velocidade_max_curva, velocidade_max_curva);
        SR = constrain(SR, -velocidade_max_curva, velocidade_max_curva);

        run_fwd(SL,SR);
      }
    }
  }
}