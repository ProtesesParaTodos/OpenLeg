#include <Wire.h> // Comunicação com os MPU's por meio do I2C
#include "Kalman.h" // Filtro de dados para as leituras do MPU

#define TAM_FILTRO 150 // Definição da quanbtidade de medidas utilizadas para ser passado pelo filtro de Kalman 

const int palmDed = A1, palmMet = A2, palmCal = A3, pinoLed = 13;
bool flag = 0;

/*Declaração das instancias de Kalman do MPU da coxa*/
Kalman kalmanZ1; //Cria a Instancia de Kalman para o eixo Z(Eixo de angulação da coxa)
Kalman kalmanY1; //Cria a Instancia de Kalman para o eixo Y(Eixo de angulação lateral da perna)
/*Declaração das instancias de Kalman do MPU da canela*/
Kalman kalmanZ2; //Cria a Instancia de Kalman para o eixo Z(Eixo de angulação da canela)
Kalman kalmanY2; //Cria a Instancia de Kalman para o eixo Y(Eixo de angulação lateral da canela)
/*Timers, endereço e buffer da comunicação I2C*/
uint32_t timerA, timerB, timer1; //timers da comunicação I2C
uint8_t MPUAdressA = 0x68, MPUAdressB = 0x69, i2cData[14]; // Endereços I2C do MPU e Buffer para os dados I2C
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

void inicializaInterrupts()
{
  cli();//stop interrupts
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 124 ;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
}

ISR(TIMER1_COMPA_vect)
{ //change the 0 to 1 for timer1 and 2 for timer2
  flag = 1;
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

void printaDados(double anguloMPU1, double velAngMPU1, double anguloMPU2, double velAngMPU2, int palmilhaA, int palmilhaB, int palmilhaC, int indice)
{
  Serial.print(anguloMPU1);
  Serial.print(", ");
  Serial.print(velAngMPU1);
  Serial.print(", ");
  Serial.print(anguloMPU2);
  Serial.print(", ");
  Serial.print(velAngMPU2);
  Serial.print(", ");
  Serial.print(palmilhaA);
  Serial.print(", ");
  Serial.print(palmilhaB);
  Serial.print(", ");
  Serial.print(palmilhaC);
  Serial.print(", ");
  Serial.print(indice);
  Serial.println(";");
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(); //Não esquecerrrrrrrrrrr
  inicializaMPU(MPUAdressA);
  inicializaMPU(MPUAdressB);
  inicializaInterrupts();
}

void loop()
{
  static unsigned int amostra = 0;
  if (flag == 1)
  {
    float anguloMPU1, anguloMPU2, velAngMPU1, velAngMPU2;
    int palmilhaA = analogRead(palmDed);
    int palmilhaB = analogRead(palmMet);
    int palmilhaC = analogRead(palmCal);
    if (atualizaAngulo(MPUAdressA) == 1)
    {
      anguloMPU1 = kalAngleY;
      velAngMPU1 = gyroYrate;
    }
    else
    {
      anguloMPU1 = 0;
      velAngMPU1 = 0;
    }
    if (atualizaAngulo(MPUAdressB) == 1)
    {
      anguloMPU2 = kalAngleY;
      velAngMPU2 = gyroYrate;
    }
    else
    {
      anguloMPU2 = 0;
      velAngMPU2 = 0;
    }
    printaDados(anguloMPU1, velAngMPU1, anguloMPU2, velAngMPU2, palmilhaA, palmilhaB, palmilhaC, amostra);
    amostra++;
    flag = 0;
  }
}
