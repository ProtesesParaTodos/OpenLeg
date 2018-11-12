/* SmartLeg - Prótese Transfemoral Inteligente II
 * Arthur Precht, Leonardo Azzi, Luciano Sampaio e Pedro Lara
 * IFSul Câmpus Charqueadas
 * SmartLeg Beta 1.0 - 2017 */

/* Bibliotecas */
#include <Servo.h>
#include <Wire.h>
#include "Kalman.h"

/* --- Definição das variáveis ---  */

//Motor
Servo Motor;

//Filtro de Kalman
Kalman kalmanX;
Kalman kalmanY;

//IMU
int16_t accX, accY, accZ; //acelerometro
int16_t gyroX, gyroY, gyroZ;// giroscópio
double accXangle, accYangle; //angulo calculado pelo sensor da aceleração
double gyroXangle, gyroYangle;// angulo calculado do sensor giroscópio
double compAngleX, compAngleY;// comparação dos dois angulos acima
double kalAngleX, kalAngleY; // Calculo do angulo utilizando o filtro de Kalman, Angulo final usado.
double gyroXrate; //velocidade angular filtrada, Angulo final usado.
double gyroYrate; //velocidade angular filtrada, Angulo final usado.
double erroGyroX, erroGyroY;
float gyroarr = 0, acel, acelAnterior; //valor de interesse giroscopio

//Palmilha Instrumentada
int p1, p2, p3;

//Posição
int posiDesejada;
int posicao; //valor de posição desejado (0 - 90 graus)
float anguloZero; //Definição do zero de cada pessoa (calibrado pra cada usuario)

uint32_t timer; //timer do I2C
uint8_t i2cData[14]; // Buffer para os dados I2C

/* --- Pinagem --- */

//Palmilha Instrumentada
const int pinP1 = 1;//Palmilha Instrumentada - Calcanhar
const int pinP2 = 2;//Palmilha Instrumentada - Hálux
const int pinP3 = 3;//Palmilha Instrumentada - Calcanhar

//Sensor de Efeito Hall
const int pinPulso = 8;

/* Inicialização de variáveis */

//Filtro de Kalman
#define TAM_FILTRO 150

//Contador de pulsos
int numPulso = 0, posiAnterior = 70, faltaPulsos, l = 0, vel;

//Finite State Machine
boolean estado = false;
boolean PE = false, a1 = false, a2 = false, a3 = false, a4 = false, a5 = false, b1 = false, b2 = false, b3 = false, X = true, Z = false;

/* ---Funções--- */
void configI2C(){
  
  Wire.begin();//comunicação do I2C
  i2cData[0] = 7; // Seta a rate de amostatragem para 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;
  i2cData[2] = 0x00; // Seta a range maxima do Giroscopio para ±250 graus/s
  i2cData[3] = 0x00; // Seta a range maxima do Acelerometro para ±2g
  while (i2cWrite(0x19, i2cData, 4, false));
  while (i2cWrite(0x6B, 0x01, true));
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) {
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100); // Aguarda o sensor se estabilizar
  
  // Seta o angulo inicial de Kalman e do Giroscopio
  while (i2cRead(0x3B, i2cData, 6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  
  // Realiza as conversões necessárias para exibição das saídas em Graus
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  kalmanX.setAngle(accXangle); // Seta o angulo inicial
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  compAngleX = accXangle;
  compAngleY = accYangle;

  erroGyroX = 0;
  erroGyroY = 0;

  for (int i = 0; i < TAM_FILTRO; i++) {
    atualizaAngulo();
    erroGyroX = erroGyroX + gyroXrate;
    erroGyroY = erroGyroY + gyroYrate;
  }

  erroGyroX = erroGyroX / TAM_FILTRO;
  erroGyroY = erroGyroY / TAM_FILTRO;
  
}
bool atualizaAngulo() { //Lê o angulo que a IMU está
  /* Atualiza todos os valores */
  if (i2cRead(0x3B, i2cData, 14) == 0)
  {
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    gyroX = ((i2cData[8] << 8) | i2cData[9]);
    gyroY = ((i2cData[10] << 8) | i2cData[11]);
    gyroZ = ((i2cData[12] << 8) | i2cData[13]);

    // // Realiza as conversões necessárias para exibição das saídas em Graus
    accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
    accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;

    gyroXrate = (double)gyroX / 131.0;
    gyroYrate = -((double)gyroY / 131.0);
    gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // Calcula o angulo do Giroscopio sem filtros
    gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000);

    compAngleX = (0.93 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.07 * accXangle); // Calcula o angulo utilizando um filtro complementar.
    compAngleY = (0.93 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.07 * accYangle);

    kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); // Calcula o angulo utilizando o filtro de Kalman.
    kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);
    timer = micros();
    return true;
  }
  else
  {
    return false;
  }
}
float HSM(){ //Controle hierárquico das atividades
   //PAR - CAM
   if (/*Condição*/){
    PAR = false;
    CAM = true;
   }

   //PAR - SNT
   else if (/*Condição*/){
    PAR = false;
    SNT = true;
   }

   //SNT - PAR
   else if (/*Condição*/){
    SNT = false;
    PAR = true;
   }

   //CAM - PAR
   else if (/*Condição*/){
    CAM = false;
    PAR = true;
   }

   if (PAR == true){
    //desligar motor
   }
   else if (CAM == true){
    FSMCam();
   }
   else if (SNT == true){
    FSMSnt();
   }
}
float FSMCam(){ //Atividade - Caminhar
//Finite State Machine - SmartLeg Beta

  int thP = 800; // Threshold do pé
    
  //Transições
      
  //ADI - MAP
  if (p1 > thP && p2 > thP){
    ADI = false;
    MAP = true;
  }
      
  //MAP - APT
  else if (p2 > thP && p3 > thP){
    MAP = false;
    APT = true;
  }
     
  //APT - BLI
  else if (p1 == 0 && p2 == 0 && p3 == 0){
   APT = false;
   BLI = true;
  }
      
  //BLI - BLT
  else if (/*FLEXÃO MÁXIMA*/){
    BLI = false;
    BLT = true;
  }
 
  //BLT - ADI
  else if (p1 < thP){
    BLT = false;
    API = true;
  }
  
//*CICLO DA MARCHA*
//*Fases de Apoio*

  //Apoio Duplo Inicial - Flexão
  if (ADI == true){
    K = 0.0557;
    b = 0.0003;
    angJoelhoEQ = 12;
  }
  
  //Médio Apoio - Extensão
  else if (MAP == true){
    K = 0.0557;
    b = 0.0003;
    angJoelhoEQ = 12;
  }
  
  //Apoio terminal/Pré-balanço - Flexão
  else if (APT == true){
    K = 0.0557;
    b = 0.0003;
    angJoelhoEQ = 12;
  }

//*Fases de balanço*
 
  //Balanço Inicial - Flexão
  if (BLI == true){
    b = 0.000238;
  }
  //Balanço Terminal - Extensão
  else if (BLT == true){
    //Extensão (máximo)
    //Logarítmico
  }
}
float FSMSnt(){ //Atividade - Sentar/Levantar
  //sentar e levantar
}
float impedanceController(){
  float torque, K, b, angJoelho, angJoelhoEQ, velAngJoelho;
  torque = K * (angJoelho - angJoelhoEQ) + b * velAngJoelho;
  return torque;
}
void controlePosicao(int posiDesejada) { //movimentação da protese old moveperna

  int erro = posiDesejada - posiAnterior;
  //Serial.println(erro);
  int numPulso = 0;

  if (erro > 0)
  {
    faltaPulsos = (erro / 0.9) / 4;
    while (numPulso < faltaPulsos)
    {
      if (digitalRead(pinPulso) == HIGH)
      {
        if (estado == true)
        {
          numPulso++;
          l = faltaPulsos - numPulso;
          Serial.println((l * 0.9) * 4);
        }
        estado = false;
      }
      else {
        estado = true;
      }

      l = faltaPulsos - numPulso;
      if (l == 0)
      {
        goto Top;
      }
      vel = map(l, 0, 100, 77, 50);
      Serial.print(vel);
      Motor.write(vel);//velocidade calcula com base no erro
    }
    erro = 0;
  }
  else if (erro < 0)
  {
    numPulso = 0;
    faltaPulsos = (-erro / 0.9) / 4;
    while (numPulso < faltaPulsos)
    {
      if (digitalRead(pinPulso) == HIGH)
      {
        if (estado == true)
        {
          numPulso++;
          float x = (faltaPulsos - numPulso) * 0.9 * 4;
          x = map(x, 0, 90, 90, 0);
          Serial.println(x);
        }
        estado = false;
      }
      else {
        estado = true;
      }
      l = faltaPulsos - numPulso;
      if (l == 0)
      {
        goto Top;
      }
      vel = map(l, 0, 100, 100, 160);
      Serial.print(vel);
      Motor.write(vel);//velocidade calcula com base no erro
    }
    erro = 0;
  }
  else if (erro == 0)
  {
Top:
    Motor.write(90);
    Serial.print("90");
  }

  posiAnterior = posiDesejada;
}
void setup() {
  Serial.begin(9600);
  Motor.attach(9);
  pinMode(Red, OUTPUT);
  pinMode(Green, OUTPUT);
  pinMode(Blue, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(pinPulso, INPUT);
  pinMode(pinP1, INPUT);
  pinMode(pinP2, INPUT);
  pinMode(pinP3, INPUT);

  configI2C();
  timer = micros();
  b1 = true;
  
  if (atualizaAngulo()) {
    anguloZero = kalAngleX;//De acordo com cada usuário (setar no inicío)
  }
  
}
void loop(){
  if (atualizaAngulo()){

    int Ang = (kalAngleX - anguloZero);
    Ang = map(Ang, -180, 180, 0, 360);
    
    /*if (Ang < 0) {
      Ang = Ang + 360;
      Ang = map(Ang, 0, 360, 360, 0);
    }
    else if (Ang > 360) {
      Ang = Ang - 360;
    }

    acel = round(gyroXrate - erroGyroX);//calcula velocidade da perna atual
    int posiAng = Ang;// posição da atual da perna
    int THposi = anguloZero + 10; //faixa de erro de posição
    int THacel = 40;// faixa de erro de acelera~ção

    if (X == true && Ang > anguloZero + 60) //Função para o usuário sentar
    {
      analogWrite(Red, 0);
      analogWrite(Green, 255);
      analogWrite(Blue, 255);

      posicao = 0;
      Serial.println("37 ANOS CARALHO");
      moveperna(posicao);
      X = false;
      Z = true;
    }
    else if (Z == true && Ang > anguloZero + 40 && acel < 10)
    {
      analogWrite(Red, 0);
      analogWrite(Green, 255);
      analogWrite(Blue, 255);

      posicao = 70;
      Serial.println("AEEEEETA PORRA");
      moveperna(posicao);
      Z = false;
      X = true;
    }

    else if (Z == true && acel >= -10 && acel <= 10 && Ang >= anguloZero - 10 && Ang <= anguloZero + 10) //manter parado
    {
      analogWrite(Red, 0);
      analogWrite(Green, 255);
      analogWrite(Blue, 255);

      posicao = 70;

      Serial.println("AEEEEETA PORRA");
      moveperna(posicao);
      Z = false;
      X = true;
    }

    else if (Z == true && acel > 10 && Ang < anguloZero + 40 && Ang > anguloZero) //Esticar a perna para caminhada
    {
      analogWrite(Red, 0);
      analogWrite(Green, 255);
      analogWrite(Blue, 255);

      posicao = 70;
      Serial.println("AEEEEETA PORRA");
      moveperna(posicao);
      Z = false;
      X = true;
    }

    analogWrite(Red, 0);
    analogWrite(Green, 0);
    analogWrite(Blue, 0);
*/
 
 moveperna(70);
 digitalWrite(13,HIGH);
 moveperna(90);
digitalWrite(13,LOW);


    /* else if (gyroarr > 10 && kalAngleX < anguloZero) //Flexionar perna
      {
       posicao = (kalAngleX - 45) * -1.5;
       levantaperna(posicao);
      }*/

    Serial.print(Ang);
    Serial.print("\n");
    Serial.print(acel);
    Serial.print("\n");
    Serial.print("\n");

    acelAnterior = acel;
    delay(100);
  }
  else {
    Serial.print("\nFalha na comunicacao\n");
  }
}
