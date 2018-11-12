//Finite State Machine - SmartLeg Beta

    //ADI
    if (p1 < 800 || p2 < 800 || p3 < 800) {

      PE = true;
      if (posiAng > THposi && acel > THacel && acel <= acelAnterior)
      {
        ADI = false;
        ADR = true;
      }
      //ADR
      else if (a2 == true && posiAng > THposi && acel < -THacel && acel <= acelAnterior)
      {
        ADR = false;
        MAP = true;
      }
      //MAP
      else if (a3 == true && posiAng > THposi && acel < -THacel && acel <= acelAnterior)
      {
        MAP = false;
        APT = true;
      }
      //APT
      else if (posiAng <= THposi && acel < -THacel && acel >= acelAnterior)
      {
        APT = false;
        PRB = true;
      }
      //PRB
      else if (a5 == true && posiAng <= THposi && acel < -THacel && acel >= acelAnterior)
      {
        PRB = false;
        BLI = true;
      }
    }
    else {

      PE = false;

      //BLI
      if ((b1 == true || a5 == true) && acel > THacel && acel >= acelAnterior)
      {
        BLI = false;
        PRB = false;
        BLM = true;
        //Perna Flexionada
        //Motor Parado
        Serial.println("Balanço Inicial - 90");
      }
      //BLM
      else if (b2 == true && posiAng > THposi && acel > THacel && acel <= acelAnterior)
      {
        BLM = false;
        BLT = true;
        //Perna Extendendo
        //Motor em Movimento
        moveperna(0);
      }
      //BLT
      else if (b3 == true && posiAng > THposi && acel > THacel && acel <= acelAnterior)
      {
        BLT = false;
        API = true;
        //Perna Extendida (máximo)
        //Motor Parado
        Serial.println("Balanço Terminal - 0");
      }
    }


    //*CICLO DA MARCHA*

    //*Fases de Apoio*

    if (PE == true) //ALGUM PÉ no CHÃO
    {
      //Apoio Duplo Inicial - Contato Inicial
      if (ADI == true) //&&(p1 em apoio (p1>p2 && p1>p3))
      {
        //Perna Extendida
        //Motor Parado
      }
      //Apoio Duplo Inicial - Resposta da Carga
      else if (APR == true) //&&(p1 em apoio (p1>p2 && p1>p3))
      {
        //Perna Levemente Extendida
        //Motor em Flexão até 15°
        moveperna(15);
     }
      //Apoio Simples - Médio Apoio
      else if (MAP == true) //&& (p1 e p2 em apoio(p1>=p2 && p1>p3 && p2>p3))
      {
        //Perna Levemente Extendida
        //Motor Parado
      }
      //Apoio Simples - Apoio Terminal
      else if (APT == true) //&& (p2 e p3 em apoio(p2>=p3 && p2>p1 && p3>p1))
      {
        //Perna Levemente Extendida
        //Motor Parado+

      }
      //Segundo Apoio Duplo - Pré-balanço
      else if (PRB == true) //&& (p3 em apoio(p3>p2 && p3>p1))
      {
        //Perna Flexionando
        //Motor em Movimento
        moveperna(90);
      }
    }
    //*Fases de Balanço*
    else//NENHUM PÉ NO CHÃO
    {
      //Oscilação Inicial - Balanço Inicial
      if (BLI == true)
      {
        //Perna Flexionada
        //Motor Parado
      }
      //Oscilação Intermediária - Balanço Médio
      else if (BLM == true)
      {
        //Perna Extendendo
        //Motor em Movimento
        moveperna(0);
      }
      //Oscilação Terminal - Balanço Terminal
      else if (BLT == true)
      {
        //Perna Extendida (máximo)
        //Motor Parado
      }
    }
