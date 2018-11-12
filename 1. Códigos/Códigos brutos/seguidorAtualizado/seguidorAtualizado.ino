#include <Wire.h> // Comunicação com os MPU's por meio do I2C
#include "Kalman.h" // Filtro de dados para as leituras do MPU
#include <Servo.h>  // Comunicação com o ESC por meio da comunicação com servo
#include <PID_v1.h> // Controle do motor por meio do PID

#define TAM_FILTRO 150 // Definição da quanbtidade de medidas utilizadas para ser passado pelo filtro de Kalman 

const int palmDed = A2, palmMet = A3, palmCal = A1, pinoLed = 13;

const int pinoFDC = 2, pinoHall1 = 3, pinoHall2 = 4; // Pino de sinal do Fim de curso e pinos do sensor de efeito hall

/*Declaração das instancias de Kalman do MPU da coxa*/
Kalman kalmanZ1; //Cria a Instancia de Kalman para o eixo Z(Eixo de angulação da coxa)
Kalman kalmanY1; //Cria a Instancia de Kalman para o eixo Y(Eixo de angulação lateral da perna)
/*Declaração das instancias de Kalman do MPU da canela*/
Kalman kalmanZ2; //Cria a Instancia de Kalman para o eixo Z(Eixo de angulação da canela)
Kalman kalmanY2; //Cria a Instancia de Kalman para o eixo Y(Eixo de angulação lateral da canela)
/*Timers, endereço e buffer da comunicação I2C*/
uint32_t timerA, timerB, timer1; //timers da comunicação I2C
uint8_t MPUAdressA = 0x69, MPUAdressB = 0x68, i2cData[14]; // Endereços I2C do MPU e Buffer para os dados I2C
/*Dados brutos recebidos do sensor */
int16_t accX, gyroX; // Aceleração e vel angular do eixo X (não utilizado, rotação no plano)
int16_t accY, gyroY; // Aceleração e vel angular do eixo Y (reservado para uso futuro, inclinação lateral do corpo)
int16_t accZ, gyroZ; // Aceleração e vel angular do eixo Z (Utilizado, angulo da coxa em relação ao solo)
/*Angulos calculados a partir de cada sensor)*/
double accZangle, accYangle; // angulo calculado a partir dos dados do acelerometro
double gyroZangle, gyroYangle; // angulo calculado a partir dos dados do giroscópio
/*Angulos filtrados e prontos para uso*/
double kalAngleZ, kalAngleY; // Angulo calculado utilizando o filtro de Kalman, angulo final usado.
double gyroZrate, gyroYrate;// Velocidade angular filtrada, velocidade angular usada.
double erroGyroZ, erroGyroY; // Erro no giroscopio do eixo X, descoberto pelo filtro de Kalman
/*Constantes da conversão de ângulo para pulsos do encoder*/
const int anguloZero = 82;// Angulo de repouso da coxa
const float resolucao = 1.8; // Resolução do sensor de efeito Hall
/*variáveis da leitura de pulsos do sensor hall*/
bool sentido = 1; // sentido que a prótese está se movimentando.
int estado = 0, pulsosDetectados = 0; // Estado da Máquina de estados e Pulsos detectados pela interrupção
/*Variáveis do PID*/
const double Kp = 0.001, Ki = 0, Kd = 0; // Parâmetros de sintonia (configuração) do PID
double Setpoint, Input, Output; // Variáveis de entrada e saída do PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // Inicialização do PID utilizado para controlar o motor.
/*Configuração inicial do ESC*/
const int pinoESC = 9; // Pino em que o ESC está
Servo ESC; // Declaração da comunicação com o ESC utilizando a biblioteca Servo

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
  accZangle = (atan2(accY, accX) + PI) * RAD_TO_DEG; //Decompõe a aceleração resultante para saber qual o ângulo do plano Z em relação à gravidade
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG; //Decompõe a aceleração resultante para saber qual o ângulo do plano Y em relação à gravidade
  /*Configuração do filtro de Kalman*/
  if (MPU == 0x68)
  {
    kalmanZ1.setAngle(accZangle); // Seta o angulo inicial
    kalmanY1.setAngle(accYangle);
    timerA = micros();   //Zeramento do timer para futuros cálculos envolvendo a velocidade angular
  }
  else
  {
    kalmanZ2.setAngle(accZangle); // Seta o angulo inicial
    kalmanY2.setAngle(accYangle);
    timerB = micros();   //Zeramento do timer para futuros cálculos envolvendo a velocidade angular
  }
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

/*Função responsavel por realizar a leitura do MPU*/
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
    accZangle = (atan2(accY, accX) + PI) * RAD_TO_DEG; //Decompõe a aceleração resultante para saber qual o ângulo do plano Z em relação à gravidade
    accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG; //Decompõe a aceleração resultante para saber qual o ângulo do plano Y em relação à gravidade
    /*Conversão da velocidade angular lida*/
    gyroZrate = (double)gyroZ / 131.0;
    gyroYrate = -((double)gyroY / 131.0);
    /*Passagem de dados para o filtro de Kalman*/
    if (MPU == 0x68)
    {
      kalAngleZ = kalmanZ1.getAngle(accZangle, gyroZrate, (double)(micros() - timerA) / 1000000); // Calcula o angulo utilizando o filtro de Kalman.
      kalAngleY = kalmanY1.getAngle(accYangle, gyroYrate, (double)(micros() - timerA) / 1000000); // micros()-timer/10000000 é o dt, ou quanto tempo passou
      timerA = micros();  //Zeramento do timer para futuros cálculos envolvendo a velocidade angular
    }
    else
    {
      kalAngleZ = kalmanZ2.getAngle(accZangle, gyroZrate, (double)(micros() - timerB) / 1000000); // Calcula o angulo utilizando o filtro de Kalman.
      kalAngleY = kalmanY2.getAngle(accYangle, gyroYrate, (double)(micros() - timerB) / 1000000); // micros()-timer/10000000 é o dt, ou quanto tempo passou
      timerB = micros();  //Zeramento do timer para futuros cálculos envolvendo a velocidade angular
    }
    return true;
  }
  else // caso a função não tenha sucesso, é retornado um "falso"
  {
    return false;
  }
}

/*Função responsável por calcular o sentido de movimento e calcular o PID, assim como limitar eletronicamente o curso máximo e minimo da prótese*/
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

/*Função responsável por dar a ordem de movimento ao ESC*/
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
      ESC.write(round(servoParadoMin - (valor + 5)));
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

void setup()
{
  Serial.begin(115200);
  Wire.begin(); //Não esquecerrrrrrrrrrr
  attachInterrupt(digitalPinToInterrupt(pinoHall1), pulsoDetectado, CHANGE); // Interrupção da leitura do Sensor de efeito Hall
  inicializaMPU(MPUAdressA);
  inicializaMPU(MPUAdressB);
  ESC.attach(pinoESC); // Inicio da comunicação com o ESC por meio do sistema de comunicação dos servos
  Input = pulsosDetectados; // Inicialização da variável de entrada do sensor
  Setpoint = 0; // Inicializacção da variável de posição desejada
  myPID.SetMode(AUTOMATIC); // Inicio do controle PID a partir das variaveis Input, Setpoint e Output
}

void loop()
{
  bool flagMPUA = 0, flagMPUB = 0;
  float anguloMPU1 = 0, anguloMPU2 = 0, velAngMPU1 = 0, velAngMPU2 = 0;

  if (atualizaAngulo(MPUAdressA) == 1)
  {
    anguloMPU1 = kalAngleY;
    velAngMPU1 = gyroYrate;
    flagMPUA = 1;
  }

  if (atualizaAngulo(MPUAdressB) == 1)
  {
    anguloMPU2 = kalAngleY;
    velAngMPU2 = gyroYrate;
    flagMPUB = 1;
  }

  if (flagMPUA != 1 || flagMPUB != 1)
  {
    decideSentido(0);
    movimentaMotor(Output);
  }

  float anguloJoelho = anguloMPU1 - anguloMPU2;
  Serial.println(anguloJoelho);

// decideSentido (round(anguloJoelho));
//  movimentaMotor(Output);

ESC.write(anguloJoelho);

}
