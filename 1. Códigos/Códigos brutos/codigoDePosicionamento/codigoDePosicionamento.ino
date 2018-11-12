#include <Servo.h>

const int pinServo = 9, pinPulso = 3; 
float anguloDesejado;
byte pulsos=0;

Servo motor;

bool movePerna(float anguloDesejado)
{
  const int spanTras = 0, velStop = 90, spanFrente = 180; // spantras é a velStop-velMaxRe e spanfrente é velMaxFrente-velStop
  const float resolucao = 1.8, accel = 0.25;
  byte pulsoDesejado = 0, erro = 0, velocidade = 0;
  float anguloPercorr, anguloPrev = 45;
  anguloPercorr = anguloDesejado - anguloPrev;
  
  //Serial.println(anguloDesejado);
  if (anguloPercorr <= 0)
  {
    pulsoDesejado = round(-1 * anguloPercorr / resolucao);
    if (pulsos <= pulsoDesejado)
    {
      erro = (pulsoDesejado) - pulsos;
      //Serial.print("      ");
      //Serial.println(erro);
      velocidade = (round(erro * accel)*90)/(spanTras);
      if (velocidade>spanTras)
      {
        velocidade=spanTras;
      }
      motor.write(velStop - velocidade); // se velocidade negativa ele vai p tras, se positiva vai p frente
      return true;
    }

    else
    {
      pulsos = 0;
      anguloPrev = anguloDesejado;
      motor.write(velStop); //para o motor
      return false;
    }
  }
  else
  {
    pulsoDesejado = round(anguloPercorr / resolucao);
    //Serial.println(anguloDesejado);
    if (pulsos < pulsoDesejado)
    {
      erro = (pulsoDesejado) - pulsos;
      velocidade = (round(erro * accel)*90)/(spanFrente);
      if (velocidade>spanFrente)
      {
        velocidade=spanFrente;
      }
      // Serial.print("      ");
      //Serial.println(erro);
      motor.write(velStop + velocidade); // se velocidade negativa ele vai p tras, se positiva vai p frente
      return true;
    }
    else
    {
      pulsos = 0;
      anguloPrev = anguloDesejado;
      motor.write(velStop); //para o motor
      return false;
    }
  }
  digitalWrite(13, HIGH);
}

void contaPulso()
{
  pulsos = pulsos + 1;
}

void setup() {
  // put your setup code here, to run once:
  motor.attach(pinServo);
  pinMode(13, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(pinPulso), contaPulso, CHANGE);
  Serial.begin(9600);
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
}
void loop() {
  // put your main code here, to run repeatedly:
  while (movePerna(10) == true)
  {
  }
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
  while (movePerna(70) == true)
  {
  }

}
