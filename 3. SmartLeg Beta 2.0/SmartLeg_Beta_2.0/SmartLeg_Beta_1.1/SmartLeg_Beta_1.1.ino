// SmartBionics
// Project SmartLeg - Beta Prototype
// Embedded Software - v.1.1
// @platform: ATMega 328p (Arduino Uno)
// @authors: Leonardo Azzi Martins <leonardoazzi@smartbionics.com.br>, Arthur de Freitas e Precht <arthurprecht@smartbionics.com.br>

#include <Wire.h> // Comunicação com os MPU's por meio do I2C
#include <Servo.h>  // Comunicação com o ESC por meio da comunicação com servo
#include "Kalman.h" // Filtro de dados para as leituras do MPU
#include <PID_v1.h> // Controle do motor por meio do PID

/*Definição dos pinos*/
const int pinoPalmilhaD = A3, pinoPalmilhaP = A1, pinoPalmilhaC = A2; // Pinos da palmilha instrumentada
const int pinoHall1 = 2, pinoHall2 = 3, pinoHall3 = 4; // Pinos do sensor de efeito hall
const int pinoLEDP = 13, pinoLEDG = 5, pinoLEDR = 6, pinoLEDB = 11; // Pinos do led da placa e dos leds da fita de led
const int pinoESC = 9; // Pino em que o ESC está conectado

/*Definição da Maquina de estados finitos e filtro de kalman*/
#define TAM_FILTRO 150 // Definição da quantidade de medidas utilizadas para ser passado pelo filtro de Kalman 
#define PARADO 0 // Definição do número do estado da máquina de estados finitos
#define FLEXAO 2 // Definição do número do estado da máquina de estados finitos
#define EXTENSAO 4 // Definição do número do estado da máquina de estados finitos
#define VOLTANDO 6 // Definição do número do estado da máquina de estados finitos

/*Declaração das instanmcias de Kalman*/
Kalman kalmanZ; // Cria a Instancia de Kalman para o eixo Z(Eixo de angulação da coxa)
Kalman kalmanY; // Cria a Instancia de Kalman para o eixo Y(Eixo de angulação lateral da perna)

/*Timers, endereço e buffer da comunicação I2C*/
uint32_t timer, timer1; //timers da comunicação I2C
uint8_t MPUAdress = 0x68, i2cData[14]; // Endereços I2C do MPU e Buffer para os dados I2C

/*Dados brutos recebidos do sensor */
int16_t accX, gyroX; // Aceleração e vel angular do eixo X (não utilizado, rotação no plano)
int16_t accY, gyroY; // Aceleração e vel angular do eixo Y (reservado para uso futuro, inclinação lateral do corpo)
int16_t accZ, gyroZ; // Aceleração e vel angular do eixo Z (Utilizado, angulo da coxa em relação ao solo)

/*Angulos calculados a partir de cada sensor)*/
double accZangle, accYangle; // angulo calculado a partir dos dados do acelerometro
double gyroZangle, gyroYangle; // angulo calculado a partir dos dados do giroscópio
double compAngleZ, compAngleY; // comparação dos dois angulos acima

/*Angulos filtrados e prontos para uso*/
double kalAngleZ, kalAngleY; // Angulo calculado utilizando o filtro de Kalman, angulo final usado.
double gyroZrate, gyroYrate;// Velocidade angular filtrada, velocidade angular usada.
double erroGyroZ, erroGyroY; // Erro no giroscopio do eixo X, descoberto pelo filtro de Kalman

/*Variáveis utilizadas para o controle de movimento e comunicação serial*/
const int anguloZero = 82;// Angulo de repouso da coxa
const float resolucao = 1.8; // Resolução do sensor de efeito Hall
bool sentido = 1; // sentido que a prótese está se movimentando.
int estado = 0, pulsosDetectados = 0; // Estado da Máquina de estados e Pulsos detectados pela interrupção

/*Variáveis utilizadas para o controle PID da posição do joelho*/
const double Kp = 1, Ki = 0, Kd = 0; // Parâmetros de sintonia (configuração) do PID
double Setpoint, Input, Output; // Variáveis de entrada e saída do PID

/*Inicialização da comunicação Servo com o ESC e do PID de controle*/
Servo ESC; // Declaração da comunicação com o ESC utilizando a biblioteca Servo
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // Inicialização do PID utilizado para controlar o motor.

//====================================================================
//                            FUNCTIONS
//====================================================================

/* -------------------------------------------------------------------
 * Function name: pulsoDetectado
 * Description: Função dde interrupção responsável por contar os 
 * pulsos lidos pelos sensores de efeito hall
 * Inputs:
 * Outputs:
 * ---------------------------------------------------------------- */
void pulsoDetectado()
{
  byte pinD = (PIND & 0b00011000); //PIND é o conjunto de portas 0-7, o & é um AND de binario, o 00011000 é a mascara, que faz todos os bits exceto o quarto e o quinto ficar 0
  if (pinD == 0b00011000 || pinD == 0b00000000) //quando o pino 3 e 4 estão do mesmo estado numa interrupção, a protese esta se movimentando para tras
  {
    pulsosDetectados--; // Diminui os pulsos(posição atual)
  }
  else
  {
    pulsosDetectados++; // Aumenta os pulsos(posição atual)
  }
}

/* -------------------------------------------------------------------
 * Function name: erroHall
 * Description: Função responsável por parar a prótese e acender luzes 
 * de aviso caso esta atinja um de seus fins de curso
 * Inputs:
 * Outputs:
 * ---------------------------------------------------------------- */

void erroHall()
{
  digitalWrite(pinoLEDP, HIGH); // Liga o led vermelho da placa
  digitalWrite(pinoLEDG, LOW); // Desliga o led verde da fita
  digitalWrite(pinoLEDR, HIGH); // Liga o led vermelho da fita
  digitalWrite(pinoLEDB, LOW); // Desliga o led azul da fita
}

/* -------------------------------------------------------------------
 * Function name: erroHall
 * Description: Função responsável por inicializar e zerar o MPU.
 * Inputs:
 * Outputs:
 * ---------------------------------------------------------------- */

void inicializaMPU(int MPU) //MPU é o endereço I2C sendo utilizado pelo MPU em questão, em Hexadecimal
{
  selectAdress(MPU); // Seleciona o endereço do sensor que está recebendo a comunicação
  Serial.println("Start:");
  i2cData[0] = 7; // Seta a rate de amostatragem para 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;
  i2cData[2] = 0x00; // Seta a range maxima do Giroscopio para ±250 graus/s
  i2cData[3] = 0x00; // Seta a range maxima do Acelerometro para ±2g
  while (i2cWrite(0x19, i2cData, 4, false));
  while (i2cWrite(0x6B, 0x01, true));
  while (i2cRead(0x75, i2cData, 1));
  Serial.println(i2cData[0]); // Mostra o endereço do sensor na tela
  if (i2cData[0] != 0x68) //Verifica se a resposta de qual o endereço o sensor tem está correta.
  {
    Serial.print("Error reading sensor");
    while (1);
  }
  delay(100); // Aguarda o sensor se estabilizar
  /*Recebimento dos valores lidos pelo sensor por  meio do buffer I2C*/
  while (i2cRead(0x3B, i2cData, 6)); // Seta o angulo inicial de Kalman e do Giroscopio
  accX = ((i2cData[0] << 8) | i2cData[1]); // Recebimento do valor bruto da aceleração do eixo X (não utilizado, pois o X é no mesmo plano que o chão)
  accY = ((i2cData[2] << 8) | i2cData[3]); // Recebimento do valor bruto da aceleração do eixo Y (reservado para uso futuro, inclinação lateral do corpo)
  accZ = ((i2cData[4] << 8) | i2cData[5]); // Recebimento do valor bruto da aceleração do eixo Z (Utilizado, angulo da coxa em relação ao solo)
  /*Conversão da aceleração lida para ângulo*/
  accZangle = (atan2(accY, accX) + PI) * RAD_TO_DEG; // Realiza as conversões necessárias para exibição das saídas em Graus
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG; // Realiza as conversões necessárias para exibição das saídas em Graus
  /*Configuração do filtro de Kalman*/
  kalmanZ.setAngle(accZangle); // Seta o angulo inicial
  kalmanY.setAngle(accYangle);
  /*Calibração dos outros ângulos utilizados no filtro*/
  gyroZangle = accZangle;
  gyroYangle = accYangle;
  compAngleZ = accZangle;
  compAngleY = accYangle;
  /*Zeramento do timer para futuros cálculos envolvendo a velocidade angular*/
  timer = micros();
  /*Zeramento do dos erros de leitura da velocidade angular*/
  erroGyroZ = 0;
  erroGyroY = 0;
  /*Medidas para cálculo inicial do erro do acelerômetro e velocidade angular*/
  for (int i = 0; i < TAM_FILTRO; i = i + 1)
  {
    atualizaAngulo(MPU); // Mede o ângulo no momento
    erroGyroZ = erroGyroZ + gyroZrate; // Acumula o erro de leitura do giroscópio do eixo Z
    erroGyroY = erroGyroY + gyroYrate; // Acumula o erro de leitura do giroscópio do eixo Y
  }
  erroGyroZ = erroGyroZ / TAM_FILTRO; // Calcula o erro de leitura do giroscópio do eixo Z
  erroGyroY = erroGyroY / TAM_FILTRO; // Calcula o erro de leitura do giroscópio do eixo Y
}

/* -------------------------------------------------------------------
 * Function name: atualizaAngulo
 * Description: Função responsavel por realizar a leitura do MPU.
 * Inputs:
 * Outputs:
 * ---------------------------------------------------------------- */

bool atualizaAngulo(int MPU) //MPU é o endereço I2C sendo utilizado pelo MPU em questão, em Hexadecimal
{
  selectAdress(MPU);  // Seleciona o endereço do sensor que está recebendo a comunicação
  /*Recebimento dos valores lidos pelo sensor por  meio do buffer I2C*/
  if (i2cRead(0x3B, i2cData, 14) == 0) //Se a leitura dos dados dos sensores ocorre com sucesso, os valores de cada grandeza são armazenados
  {
    accX = ((i2cData[0] << 8) | i2cData[1]); // Recebimento do valor bruto da aceleração do eixo X (não utilizado, pois o X é no mesmo plano que o chão)
    accY = ((i2cData[2] << 8) | i2cData[3]); // Recebimento do valor bruto da aceleração do eixo Y (reservado para uso futuro, inclinação lateral do corpo)
    accZ = ((i2cData[4] << 8) | i2cData[5]); // Recebimento do valor bruto da aceleração do eixo Z (Utilizado, angulo da coxa em relação ao solo)
    gyroX = ((i2cData[8] << 8) | i2cData[9]); // Recebimento do valor bruto da velocidade angular do eixo X (não utilizado, pois o X é no mesmo plano que o chão)
    gyroY = ((i2cData[10] << 8) | i2cData[11]); // Recebimento do valor bruto da velocidade angular do eixo Y (reservado para uso futuro, inclinação lateral do corpo)
    gyroZ = ((i2cData[12] << 8) | i2cData[13]); // Recebimento do valor bruto da velocidade angular do eixo Z (Utilizado, angulo da coxa em relação ao solo)
    /*Conversão da aceleração lida para ângulo*/
    accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG; //Realiza as conversões necessárias para exibição das saídas em Graus
    accZangle = (atan2(accY, accX) + PI) * RAD_TO_DEG; //Realiza as conversões necessárias para exibição das saídas em Graus
    /*Conversão da velocidade angular*/
    gyroZrate = (double)gyroZ / 131.0;
    gyroYrate = -((double)gyroY / 131.0);
    //gyroZangle += gyroXrate * ((double)(micros() - timer) / 1000000); // Calcula o angulo do Giroscopio sem filtros
    //gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000);
    /*Cálculo do filtro complementar juntando angulo lido pelo giroscópio com ângulo lido pelo acelerômetro*/
    compAngleZ = (0.93 * (compAngleZ + (gyroZrate * (double)(micros() - timer) / 1000000))) + (0.07 * accZangle); // Calcula o angulo utilizando um filtro complementar.
    compAngleY = (0.93 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.07 * accYangle);
    /*Passagem de dados para o filtro de Kalman*/
    kalAngleZ = kalmanZ.getAngle(accZangle, gyroZrate, (double)(micros() - timer) / 1000000); // Calcula o angulo utilizando o filtro de Kalman.
    kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);
    /*Zeramento do timer para futuros cálculos envolvendo a velocidade angular*/
    timer = micros();
    return true;
  }
  else // caso a função não tenha sucesso, é retornado um "falso"
  {
    return false;
  }
}

/* -------------------------------------------------------------------
 * Function name: atualizaAngulo
 * Description: Função que faz parte da máquina de estados, responsável 
 * por descobrir em qual etapa do ciclo de marcha a prótese está.
 * Inputs:
 * Outputs:
 * ---------------------------------------------------------------- */

void selecionaAtividade(int anguloDaCoxa, int velocidadeAngular)
{
  if (velocidadeAngular < 5 && velocidadeAngular > -5 && anguloDaCoxa < anguloZero - 10 && anguloDaCoxa > anguloZero + 10)
  {
    estado = PARADO;
  }
  else if (velocidadeAngular >= 5 && anguloDaCoxa < anguloZero + 5) // provavelmente os 25 são uma função da velocidade em que se caminha
  {
    estado = FLEXAO;
  }
  else if (velocidadeAngular >= 5 && anguloDaCoxa > anguloZero + 5)
  {
    estado = EXTENSAO;
  }
  else
  {
    estado = VOLTANDO;
  }
}

/* -------------------------------------------------------------------
 * Function name: atualizaAngulo
 * Description: Função que faz parte da máquina de estados, responsável 
 * por decidir qual a ação a ser realizada pela prótese.
 * Inputs:
 * Outputs:
 * ---------------------------------------------------------------- */

int selecionaMovimento(int estado, int anguloDaCoxa)
{
  static int menorAnguloCoxa = anguloZero;//Variável utilizada para guardar o menor ângulo que a coxa chegou, para flexionar a perna de acordo
  /*Seleciona a ação de acordo com a fase descoberta na função anterior*/
  switch (estado)
  {
    case PARADO: //branco
      if (anguloDaCoxa < menorAnguloCoxa)
      {
        menorAnguloCoxa = anguloDaCoxa;
      }
      analogWrite(pinoLEDG, 90);
      analogWrite(pinoLEDR, 90);
      analogWrite(pinoLEDB, 90);
      return 0;
      break;

    case FLEXAO: //laranja
      analogWrite(pinoLEDG, 150);
      analogWrite(pinoLEDR, 100);
      analogWrite(pinoLEDB, 0);
      return 2 * (anguloDaCoxa - menorAnguloCoxa);
      break;

    case EXTENSAO:  //amarelo
      analogWrite(pinoLEDG, 125);
      analogWrite(pinoLEDR, 125);
      analogWrite(pinoLEDB, 0);
      menorAnguloCoxa = anguloZero;
      return 0;
      break;

    case VOLTANDO: //verde claro
      analogWrite(pinoLEDG, 100);
      analogWrite(pinoLEDR, 100);
      analogWrite(pinoLEDB, 100);
      if (anguloDaCoxa < menorAnguloCoxa)
      {
        menorAnguloCoxa = anguloDaCoxa;
      }
      return 0;
      break;
  }
}

/* -------------------------------------------------------------------
 * Function name: atualizaAngulo
 * Description: Função responsável por calcular o sentido de movimento 
 * e calcular o PID, assim como limitar eletronicamente o curso máximo 
 * e minimo da prótese
 * Inputs:
 * Outputs:
 * ---------------------------------------------------------------- */

void decideSentido(int PosicaoFinal) //PosicaoFinal é o valor que se deseja que a prótese angule, em graus
{
  const int pulsosMin = -3, pulsosMax = 40; // Limites de curso mínimo (-3*1.8 = -5.4º) e curso máximo (40*1.8=72º)
  int pulsosPosicaoFinal = round(PosicaoFinal / resolucao); // Cálculo da posição final a ser atingida

  if (pulsosPosicaoFinal > pulsosMax) // Caso o angulo desejado seja maior que o limite eletronico, a prótese angulará apenas até o limite
  {
    pulsosPosicaoFinal = pulsosMax; // Faz a angulação desejada ser o limite máximo
  }

  if (pulsosPosicaoFinal < pulsosMin) // Caso o angulo desejado seja menor que o limite eletronico, a prótese angulará apenas até o limite
  {
    pulsosPosicaoFinal = pulsosMin; // Faz a angulação desejada ser o limite mínimo
  }

  if ( pulsosPosicaoFinal >  pulsosDetectados + 1) // Perna está se afastando da posição mais proxima de 0; pulsos devem aumentar, ESC maior que 90
  {
    Setpoint = pulsosPosicaoFinal; // O ponto aonde se quer chegar é a angulação indicada.
    Input = pulsosDetectados;
    myPID.Compute();
    sentido = 1;
  }
  else if (pulsosPosicaoFinal < pulsosDetectados - 1) // Perna está retornando da posição mais proxima de 0; pulsos devem diminuir, ESC menor que 90
  {
    Setpoint = pulsosDetectados;
    Input = pulsosPosicaoFinal;
    myPID.Compute();
    sentido = 0;
  }
  else
  {
    Setpoint = 0;
    Input = 0;
    myPID.Compute();
  }
}

/* -------------------------------------------------------------------
 * Function name: atualizaAngulo
 * Description: Função responsável por dar a ordem de movimento ao ESC.
 * Inputs:
 * Outputs:
 * ---------------------------------------------------------------- */
 
void movimentaMotor(double valor)
{
  const int servoMin = 20, servoParadoMin = 83; // Valor minimo que o ESC aceita como entrada e valor minimo de entrada que não movimenta o motor
  const int servoParado = 88; // Valor ideal para parar o ESC
  const int servoParadoMax = 93, servoMax = 180; // Valor maximo de entrada que não movimenta o motor e valor maximo que o ESC aceita como entrada
  const float comparacao = 1.38, correcao = 0.35; // Linearização entre o range para trás (83-20) e para frente (180-93) e conversão do valor de 8 bits do PID para o ESC

  valor = round(valor * correcao); // Conversão do valor de 0-255(8 bits) para o valor de 8-140 do Servo

  if (sentido == 0 ) // Se a prótese esteja se movendo para trás (0) é enviado sinal menor que 90, Se a prótese esteja se movendo para frente (1) é enviado sinal maior que 90
  {
    if (valor > servoParadoMin - servoMin) // Limitação do valor a ser enviado ao ESC, para não ultrapassar seu limite minimo
    {
      ESC.write(servoMin);
    }
    else
    {
      ESC.write(round(servoParadoMin - valor + 5));
    }
  }
  else
  {
    if (valor > servoMax - servoParadoMax) // Limitação do valor a ser enviado ao ESC, para não ultrapassar seu limite maximo
    {
      ESC.write(servoMax);
    }
    else
    {
      ESC.write(round(servoParadoMax + valor * comparacao)); // Lineariza o movimento para a frente para estes terem a mesma proporção
    }
  }
}

//====================================================================
//                          SETUP AND LOOP
//====================================================================

void setup()
{
  pinMode(0, INPUT); //Definição necessária para a leitura de portas da interrupção
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(pinoLEDG, OUTPUT);
  pinMode(pinoLEDR, OUTPUT);
  pinMode(7, INPUT);
  pinMode(pinoLEDP, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(pinoHall1), pulsoDetectado, CHANGE); // Interrupção da leitura do Sensor de efeito Hall

  Wire.begin(); //Inicio da comunicação wire, responsavel por tratar do protoocolo I2C, para o MPU
  ESC.attach(pinoESC); // Inicio da comunicação com o ESC por meio do sistema de comunicação dos servos
  Serial.begin(115200); // Comunicação serial. Não esquecer de setar o rate certo
  myPID.SetMode(AUTOMATIC); // Inicio do controle PID a partir das variaveis Input, Setpoint e Output
  Input = pulsosDetectados; // Inicialização da variável de entrada do sensor
  Setpoint = 0; // Inicializacção da variável de posição desejada
  inicializaMPU(MPUAdress);
}

void loop()
{
  static int posicao = 0; // Posição que se deseja que a prótese chegue
  if (atualizaAngulo(MPUAdress)) // Caso o MPU esteja funcionando, o programa executa sua função normalmente
  {
    int velAngCoxa = round(gyroZrate - erroGyroZ); // Eixo do ângulo de movimento da coxa que depende da orinetação do sensor
    float anguloCoxa = kalAngleZ; // Eixo do ângulo de movimento da coxa que depende da orinetação do sensor
    selecionaAtividade(anguloCoxa, velAngCoxa);
    posicao = selecionaMovimento(estado, anguloCoxa); // Ângulo descrito pelo joelho do usuario dos MPU's
    decideSentido(posicao); // Decide o sentido da movimentação do joelho
    Serial.print(anguloCoxa);
    Serial.println(posicao);
    movimentaMotor(Output); // Envia a movimentação do joelho
    digitalWrite(pinoLEDP, LOW); // Desligar as luzes caso o fim de curso desative
  }
  else
  {
    ESC.write(90);
    Serial.print("\n Falha na comunicacao \n");
  }
}

//end
