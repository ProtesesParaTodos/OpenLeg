#include <Wire.h> // Comunicação com os MPU's por meio do I2C
#include "Kalman.h" // Filtro de dados para as leituras do MPU

#define TAM_FILTRO 150 // Definição da quanbtidade de medidas utilizadas para ser passado pelo filtro de Kalman 

const int palmDed = A1, palmMet = A2, palmCal = A3;
bool flag = 0;

Kalman kalmanX; // Cria as Instancias de Kalman
Kalman kalmanY;

Kalman kalmanX1; // Cria as Instancias de Kalman
Kalman kalmanY1;

uint32_t timer, timer1; //timer do I2C
uint8_t i2cData[14], MPUA = 0x68, MPUB = 0x69; // Buffer para os dados I2C, edereços I2C dos MPU'S

int16_t accX, accY, accZ; //acelerometro
int16_t gyroX, gyroY, gyroZ;// giroscópio

double accXangle, accYangle; //angulo calculado pelo sensor da aceleração
double kalAngleX, kalAngleY; // Calculo do angulo utilizando o filtro de Kalman, Angulo final usado.

double gyroXrate, gyroYrate;//velocidade angular filtrada, e usada.
double erroGyroX, erroGyroY; // Só tiaginho salva

int16_t accX1, accY1, accZ1; //acelerometro
int16_t gyroX1, gyroY1, gyroZ1;// giroscópio

double accXangle1, accYangle1; //angulo calculado pelo sensor da aceleração
double kalAngleX1, kalAngleY1; // Calculo do angulo utilizando o filtro de Kalman, Angulo final usado.

double gyroXrate1, gyroYrate1;// velocidade angular usada.
double erroGyroX1, erroGyroY1; // Erro na leitura dos giroscópios em relação à mudança de ângulo deduzida do pelo acelerômetro

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




void inicializaMPU() {
  selectAdress(MPUA);

  Serial.println("Start:");

  i2cData[0] = 7; // Seta a rate de amostatragem para 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;
  i2cData[2] = 0x00; // Seta a range maxima do Giroscopio para ±250 graus/s
  i2cData[3] = 0x00; // Seta a range maxima do Acelerometro para ±2g
  while (i2cWrite(0x19, i2cData, 4, false));//VER COM LITTLE MINISTRO
  while (i2cWrite(0x6B, 0x01, true));//VER COM LITTLE MINISTRO
  while (i2cRead(0x75, i2cData, 1));//VER COM LITTLE MINISTRO
  Serial.println(i2cData[0]);
  if (i2cData[0] != 0x68) //Verifica se a resposta de qual o endereço o senesor tem está correta.
  {
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

  timer = micros();

  erroGyroX = 0;
  erroGyroY = 0;

  for (int i = 0; i < TAM_FILTRO; i = i + 1)
  {
    atualizaAngulo();
    erroGyroX = erroGyroX + gyroXrate;
    erroGyroY = erroGyroY + gyroYrate;
  }

  erroGyroX = erroGyroX / TAM_FILTRO;
  erroGyroY = erroGyroY / TAM_FILTRO;
}

void inicializaMPU1() {
  selectAdress(MPUB);

  Serial.println("Start:");

  i2cData[0] = 7; // Seta a rate de amostatragem para 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;
  i2cData[2] = 0x00; // Seta a range maxima do Giroscopio para ±250 graus/s
  i2cData[3] = 0x00; // Seta a range maxima do Acelerometro para ±2g
  while (i2cWrite(0x19, i2cData, 4, false));//VER COM LITTLE MINISTRO
  while (i2cWrite(0x6B, 0x01, true));//VER COM LITTLE MINISTRO
  while (i2cRead(0x75, i2cData, 1));//VER COM LITTLE MINISTRO
  Serial.println(i2cData[0]);
  if (i2cData[0] != 0x68) //Verifica se a resposta de qual o endereço o senesor tem está correta.
  {
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Aguarda o sensor se estabilizar

  // Seta o angulo inicial de Kalman e do Giroscopio
  while (i2cRead(0x3B, i2cData, 6));
  accX1 = ((i2cData[0] << 8) | i2cData[1]);
  accY1 = ((i2cData[2] << 8) | i2cData[3]);
  accZ1 = ((i2cData[4] << 8) | i2cData[5]);
  // Realiza as conversões necessárias para exibição das saídas em Graus
  accYangle1 = (atan2(accX1, accZ1) + PI) * RAD_TO_DEG;
  accXangle1 = (atan2(accY1, accZ1) + PI) * RAD_TO_DEG;

  kalmanX1.setAngle(accXangle1); // Seta o angulo inicial
  kalmanY1.setAngle(accYangle1);

  timer1 = micros();

  erroGyroX1 = 0;
  erroGyroY1 = 0;

  for (int i = 0; i < TAM_FILTRO; i = i + 1)
  {
    atualizaAngulo1();
    erroGyroX1 = erroGyroX1 + gyroXrate1;
    erroGyroY1 = erroGyroY1 + gyroYrate1;
  }

  erroGyroX1 = erroGyroX1 / TAM_FILTRO;
  erroGyroY1 = erroGyroY1 / TAM_FILTRO;
}

bool atualizaAngulo() //ve o angulo que o sensor está
{
  selectAdress(MPUA);
  /* Atualiza todos os valores */
  if (i2cRead(0x3B, i2cData, 14) == 0) //CHAMA O MINISTRO
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

bool atualizaAngulo1() //ve o angulo que o sensor está
{
  selectAdress(MPUB);
  /* Atualiza todos os valores */
  if (i2cRead(0x3B, i2cData, 14) == 0) //CHAMA O MINISTRO
  {
    accX1 = ((i2cData[0] << 8) | i2cData[1]);
    accY1 = ((i2cData[2] << 8) | i2cData[3]);
    accZ1 = ((i2cData[4] << 8) | i2cData[5]);
    gyroX1 = ((i2cData[8] << 8) | i2cData[9]);
    gyroY1 = ((i2cData[10] << 8) | i2cData[11]);
    gyroZ1 = ((i2cData[12] << 8) | i2cData[13]);

    // // Realiza as conversões necessárias para exibição das saídas em Graus
    accXangle1 = (atan2(accY1, accZ1) + PI) * RAD_TO_DEG;
    accYangle1 = (atan2(accX1, accZ1) + PI) * RAD_TO_DEG;

    gyroXrate1 = (double)gyroX1 / 131.0;
    gyroYrate1 = -((double)gyroY1 / 131.0);

    kalAngleX1 = kalmanX1.getAngle(accXangle1, gyroXrate1, (double)(micros() - timer1) / 1000000); // Calcula o angulo utilizando o filtro de Kalman.
    kalAngleY1 = kalmanY1.getAngle(accYangle1, gyroYrate1, (double)(micros() - timer1) / 1000000);
    timer1 = micros();
    return true;
  }
  else
  {
    return false;
  }
}




void printaDados(double anguloMPU1, double velAngMPU1, double anguloMPU2, double velAngMPU2, int palmilhaA, int palmilhaB, int palmilhaC, int indice)
{
  Serial.print(anguloMPU1);
  Serial.print(",");
  Serial.print(velAngMPU1);
  Serial.print(",");
  Serial.print(anguloMPU2);
  Serial.print(",");
  Serial.print(velAngMPU2);
  Serial.print(",");
  Serial.print(palmilhaA);
  Serial.print(",");
  Serial.print(palmilhaB);
  Serial.print(",");
  Serial.print(palmilhaC);
  Serial.print(",");
  Serial.print(indice);
  Serial.println(";");
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(); //Não esquecerrrrrrrrrrr
  inicializaMPU();
  inicializaMPU1();
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
    atualizaAngulo();
    anguloMPU1 = kalAngleY;
    velAngMPU1 = gyroYrate;
    atualizaAngulo1();
    anguloMPU2 = kalAngleY1;
    velAngMPU2 = gyroYrate1;
    printaDados(anguloMPU1, velAngMPU1, anguloMPU2, velAngMPU2, palmilhaA, palmilhaB, palmilhaC, amostra);
    amostra++;
    flag = 0;
  }
}
