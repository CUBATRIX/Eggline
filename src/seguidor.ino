#include <QTRSensors.h>
#include <BluetoothSerial.h>
#include <Preferences.h>

// Setando pinos
// Led interno para Avisos
#define LED 2
// ----- Sensor ------
#define S0 23
#define S1 22
#define S2 32
#define S3 33
#define S4 25
#define S5 26
#define S6 27
#define S7 14

// ----- Sensor Lateral ----
#define SLE 39
#define SLD 36

// ---- Pinos para ponte H ----
// Motor A - Direito
#define AIN1 18
#define AIN2 19
#define PWMA 21
// Motor B - Esquerdo
#define BIN1 17
#define BIN2 16
#define PWMB 4

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
int velocidade_max = 120;
int velocidade_base = 90;
int velocidade_max_curva = 80; // ajustar ainda.

// Estados
bool run = false;

// Sensor lateral direito
int THRESHOLD_LATERAL = 4000;
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
float KP = 0.015;
float KI = 0;
float KD = 0.1;
float Proporcional = 0, Integral = 0, Derivativo = 0, Velocidade = 0;
// variaveis para modo de curva e sensor esquerdo;
float KPC = 0.3;
float KDC = 0.2;
bool curva = false;
int tempoMinimoCurva = 300;
unsigned long tempoInicioCurva = 0;
// Variaveis para intrepretação de intersecção
int S_inter = 0;
bool PID = true;
unsigned long tempoIn = 0;
int CDR = 300;
// ---- Parametrôs para teste -----
bool TS = false;
bool TSU = false;
bool TSD = false;


void setup() {
  Serial.begin(115200);
  SerialBT.begin("Egg");
  // Inicializando pinos
  pinMode(LED, OUTPUT);
  // motor direito
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  // moto esquerdo
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  analogWriteFrequency(PWMA, 20000);
  analogWriteFrequency(PWMB, 20000);
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
  CDR = preferences.getInt("cdr", CDR);
  preferences.end();
  SerialBT.println("KP: " + String(KP) + " | KI: " + String(KI) + " | KD: " + String(KD) + " | KPC: " + String(KPC) + " | KDC " + String(KDC));
  SerialBT.println("Velocidade_max: " + String(velocidade_max) + " | Velocidade_max_curva: " + String(velocidade_max_curva));
  SerialBT.println("ContadorMarcas: " + String(contadorMarcas) + " | Threshold_lateral: " + String(THRESHOLD_LATERAL) + " | CDR: " + String(CDR/1000)+"s");

  // configurando sensor e calibração inicial
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){S7,S6,S5,S4,S3,S2,S1,S0}, SensorCount);
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
    //run = true;
  } else {
    SerialBT.println("Pista configurado para linha Branca");
    corDaPista = false;
    //run = true;
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
    contadorMarcasAtual = 0;
    temporizadorAtivo = false;
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
    for(int i = 0; i < 400; i++){
      Andar_Frente();
    }
    delay(1000);
    Stop();
  } else if (cmd == "MD"){
    // motor certo orientação errada
    SerialBT.println("Motor da direita girando");
    for(int i = 0; i < 400; i++){
      Andar_Direita();
    }
    delay(1000);
    Stop();
  } else if (cmd == "ME"){
    // moto esquerdo correto, orientação errada
    SerialBT.println("Motor da Esquerda girando");
    for(int i = 0; i < 400; i++){
      Andar_Esquerda();
    }
    delay(1000);
    Stop();
    // estra reconhencdo sensor esquerdo
  } else if (cmd == "TSL"){
    TSD = !TSD;
    TS = false;
    SerialBT.println(TSD ? "Iniciando teste do sensores lateral" : "Teste do sensor lateral parado");
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
      if(val > 120){
        velocidade_max = 120;
        SerialBT.println("Velocidade máxima alterado para: " + String(velocidade_max));
      } else{
        velocidade_max = val;
        SerialBT.println("Velocidade máxima alterado para: " + String(velocidade_max));
      }
    }
  }else if(cmd.startsWith("VELMAXC ")){
    int val = cmd.substring(8).toInt();
    if(val >= 0){
      if(val > 120){
        velocidade_max_curva = 120;
        SerialBT.println("Velocidade máxima de curva alterado para: " + String(velocidade_max_curva));
      } else {
        velocidade_max_curva = val;
        SerialBT.println("Velocidade máxima de curva alterado para: " + String(velocidade_max_curva));
      }
    }
  }else if(cmd.startsWith("VEL ")){
    int val = cmd.substring(4).toInt();
    if(val >= 0){
      if(val > 120){
        velocidade_base = 120;
        SerialBT.println("Velocidade base alterado para: " + String(velocidade_base));
      } else{
        velocidade_base = val;
        SerialBT.println("Velocidade base alterado para: " + String(velocidade_base));
      }
    }
  }else if(cmd.startsWith("TMC ")){
    int val = cmd.substring(4).toInt();
    if (val >= 0){
      tempoMinimoCurva = val;
      SerialBT.println("Tempo minimo para periodo de curva: " + String(tempoMinimoCurva/1000)+"s");
    }
  }else if(cmd.startsWith("CDR ")){
   int val = cmd.substring(4).toInt();
   if (val >= 0){
      CDR = val;
      SerialBT.println("Tempo de espera apos interseccao alterado para: " + String(CDR/1000)+"s");        
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
    delay(500);
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
    delay(500);
  }
} 
void TesteSensorDr(){
  int lerSensor = analogRead(SLD);
  int lerSensor2 = analogRead(SLE);
  SerialBT.println("Valor do sensor direito: " + String(lerSensor) + " | Valor do sensor esquerdo: " + String(lerSensor2));
  delay(400);
}
//funções auxiliar
void Andar_Direita(){
  // Motor direito
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 120);
  // Motor Esquerdo
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
}
void Andar_Esquerda(){
  // Motor direito
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
  // Motor Esquerdo
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 120);
}
void Andar_Frente(){
  // Motor direito
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 120);
  
  // Motor Esquerdo
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 120);
}
void Stop(){
  run = false;
  // Motor direito
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, LOW);
  // Motor Esquerdo
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, LOW);
  
}
void run_fwd(int SL, int SR){
  // Motor direito
  if(SR > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, SR);
        
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, abs(SR));
  }
  // Motor Esquerdo
  if(SL > 0){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, SL);
  }else{
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, abs(SL));
  }
}
void SensorEq(){
  int valorSensor = analogRead(SLE);
  // para linha preta
  if(corDaPista){
    if (valorSensor > THRESHOLD_LATERAL && curva == false){
      curva = true;
      tempoInicioCurva = millis();
    } 
  } else { //linha branca
    if (valorSensor < THRESHOLD_LATERAL && curva == false){
      curva = true;
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
  // adicionar variaveis para alterar esse parametro posteriormente.
  if(corDaPista == true && media > 600){
    PID = false;
    tempoIn = millis();
    run_fwd(velocidade_max, velocidade_max);
  }else if(corDaPista == false && media < 400){
    PID = false;
    tempoIn = millis();
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
  preferences.putInt("cdr", CDR);
  preferences.putInt("vm", velocidade_max);
  preferences.putInt("vmc", velocidade_max_curva);
  preferences.putInt("ths", THRESHOLD_LATERAL);
  preferences.end();
  SerialBT.println("KP: " + String(KP) + " | KI: " + String(KI) + " | KD: " + String(KD) + " | KPC: " + String(KPC) + " | KDC " + String(KDC));
  SerialBT.println("Velocidade_max: " + String(velocidade_max) + " | Velocidade_max_curva: " + String(velocidade_max_curva));
  SerialBT.println("ContadorMarcas: " + String(contadorMarcas) + " | Threshold_lateral: " + String(THRESHOLD_LATERAL) + " | CDR: " + String(CDR/1000)+ "s");
}
void run_robot(){
  
  uint16_t position;
  if (corDaPista) {
    position = qtr.readLineBlack(qtrValues);
  } else {
    position = qtr.readLineWhite(qtrValues);
  }
  // identifica intersecçao

  identificarInterseccao();
   // identifica se acabou de passar por uma interseccao
  if(millis() - tempoIn > CDR){
    SensorEq();
    SensorDr();      
  }
  if(contadorMarcas > 0 && contadorMarcasAtual >= contadorMarcas) {
    contagemParada();
  }

  ERRO = position - SetPoint;
  if(PID){
    if(curva == false){
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

      int SL = constrain(velocidade_base + Velocidade, -velocidade_max, velocidade_max);
      int SR = constrain(velocidade_base - Velocidade, -velocidade_max,velocidade_max);

      run_fwd(SL,SR);
    } else{
      if(abs(ERRO) < 300 && (millis() - tempoInicioCurva > tempoMinimoCurva)){
        curva = false;
      } else {
        Proporcional = ERRO * KPC;
        Integral = (ERRO + ERRO_ANTERIOR) * 0;
        Derivativo = (ERRO - ERRO_ANTERIOR) * KDC;
        Velocidade = Proporcional + Integral + Derivativo;
        if(Velocidade > velocidade_max_curva){
          Velocidade = velocidade_max_curva;
        } else if (Velocidade < -velocidade_max_curva){
          Velocidade = -velocidade_max_curva;
        }
        int SL = constrain(velocidade_base + Velocidade,-velocidade_max_curva, velocidade_max_curva);
        int SR = constrain(velocidade_base - Velocidade,-velocidade_max_curva, velocidade_max_curva);
        run_fwd(SL,SR);
      }
    }
  }
}
