#include <Servo.h>
#include <NewPing.h>

enum statusEnum
{
  parado,
  parado_com_trava,
  andando_para_frente,
  andando_para_tras,
  medindo_distancias
};

//Configuracoes
#define MAX_DISTANCE 450       // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define TempoDebounce 50       //Tempo para eliminar o efeito Bounce EM MILISEGUNDOS
#define DistanciaMaximaChao 60 //Valor limite de distancia retornado pelos sensores infravermelhos pra impedir quedas
#define TEMPO_RAMPA 30         //Intervalo em milisegundos para as rampas de aceleracao e desaceleracao
#define velocidade_maxima 255  //Velocidade máxima que o motor vai atingir
#define minimaLuminosidade 200 //abaixo desse valor o farol entra no modo de máxima luminosidade se o modoFarol estiver no automatico
#define maximaLuminosidade 500 //acima desse valor o farol apaga se o modoFarol estiver no automatico

//Componentes
#define ledFarol 3               //alimentação do farol menor
#define motorA1 4                //é um motor apenas, pois para cada motor são 2 pinos mais o pino de velocidade
#define motorA2 5                //segundo pino do motor
#define motorASpeed 6            //controle de velocidade do motor
#define botaoModoFarol 8         //Botão que muda o modo do farol
#define sensorDistanciaGatilho 9 //Gatilho do sensor supersonico
#define sensorDistanciaEco 10    //Receptor de eco do sensor supersonico
#define pinoServoCabeca 11       //servo que movimenta o sensor supersonico
#define buzzer 12                //buzina
#define transistorSuperFarol 13  //porta que controla o transistor que liga o superFarol
#define medidorDeTensao A0       //medidor de tensão (não implementado)
#define sensorInfraFrente A1     //Usando portas analógicas porém está usando de forma digital
#define sensorInfraTras A2       //Usando portas analógicas porém está usando de forma digital
#define fotoResistor A5          //medição do fotoresitor

//Objetos globais
Servo servoCabeca;
NewPing sonar(sensorDistanciaGatilho, sensorDistanciaEco, MAX_DISTANCE);
static statusEnum status;

//controle do farol
bool estadoBotao;
static bool estadoBotaoAnt;
static int modoFarol = 3; //1 = automatico, 2 = ligado e 3 = desligado, ele começa com 3 mas na primeira passada ele vira 0
static unsigned long delayBotao = 0;
static int intensidadeFarol = 255; //valor de 0 até 255

//declaração das funções
void andarParaFrente();
void andarParaTras();
void virarParaDireita();
void virarParaEsquerda();
void pararMotores();
void pararMotoresComTrava();
int medirDistancias();
void mudarEstadoFarol();
void controlaLedsFarol();
int validarMovimentos(); //função pra evitar quedas de escadas ou mesas, retorna se ele pode ir para frente ou para trás

void setup()
{
  //declarações dos pinos
  Serial.begin(9600);
  pinMode(ledFarol, OUTPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorASpeed, OUTPUT);
  pinMode(botaoModoFarol, INPUT_PULLUP);
  servoCabeca.attach(pinoServoCabeca);
  servoCabeca.write(90);
  pinMode(buzzer, OUTPUT);
  pinMode(transistorSuperFarol, OUTPUT);
  pinMode(fotoResistor, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(sensorInfraFrente, INPUT);
  pinMode(sensorInfraTras, INPUT);

  //inicia o motor parado
  status = parado;
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  analogWrite(motorASpeed, 0);

  Serial.println("Sistemas iniciados");
}

void mudarStatus(statusEnum novoStatus)
{
  if (novoStatus != status)
  {
    status = novoStatus;
    if (novoStatus == andando_para_frente)
    {
      Serial.println("andando pra frente");
      return;
    }
    if (novoStatus == andando_para_tras)
    {
      Serial.println("andando pra tras");
      return;
    }
    if (novoStatus == parado)
    {
      Serial.println("parado");
      return;
    }
    if (novoStatus == parado_com_trava)
    {
      Serial.println("parado com trava");
      return;
    }
    if (novoStatus == medindo_distancias)
    {
      Serial.println("medindo distancias");
      return;
    }
    Serial.print("Status não catalogado: ");
    Serial.println(novoStatus);
  }
}

void andarParaFrente()
{
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  if (status != andando_para_frente)
  {
    for (int i = 0; i < velocidade_maxima + 1; i = i + 10)
    {
      analogWrite(motorASpeed, i);
      delay(TEMPO_RAMPA); //intervalo para incrementar a variavel i
    }
    mudarStatus(andando_para_frente);
  }
  else
  {
    analogWrite(motorASpeed, velocidade_maxima);
  }
}

void andarParaTras()
{
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  if (status != andando_para_tras)
  {
    for (int i = velocidade_maxima; i >= 0; i = i - 10)
    {
      analogWrite(motorASpeed, i);
      delay(TEMPO_RAMPA); //intervalo para incrementar a variavel i
    }
    mudarStatus(andando_para_tras);
  }
  else
  {
    analogWrite(motorASpeed, velocidade_maxima);
  }
}

//TODO: Colocar comandos do servo para virar as rodas
void virarParaDireita()
{
  Serial.println("Virando para direita");
  //colocar comando do servo aqui
}

void virarParaEsquerda()
{
  Serial.println("Virando para esquerda");
  //colocar comando do servo aqui
}

void pararMotores()
{
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  mudarStatus(parado);
}

void pararMotoresComTrava()
{
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, HIGH);
  mudarStatus(parado_com_trava);
}

//retorna a direção do grau com maior distancia
int medirDistancias()
{
  Serial.println("Medindo distância");
  int distancia[5];
  int grauDaMaiorDistancia = 0;
  for (int contador = 0, cont2 = 0; contador <= 180; contador = contador + 45, cont2++)
  {
    servoCabeca.write(contador);
    delay(500);
    distancia[cont2] = sonar.ping_cm();
    if (distancia[cont2] > distancia[grauDaMaiorDistancia])
    {
      grauDaMaiorDistancia = cont2;
    }
  }
  //movimento suave do servo
  //  for (int teste = 1000; teste < 2000; teste++) {
  //    servoCabeca.write(map(teste, 1000, 2000, 0, 180));
  //    delay(5);
  //  }
  servoCabeca.write(90);
  Serial.print("Maior distância encontrada: ");
  Serial.print(distancia[grauDaMaiorDistancia]);
  Serial.println(" cm");
  return grauDaMaiorDistancia;
}

void mudarEstadoFarol()
{
  if ((millis() - delayBotao) > TempoDebounce)
  {
    estadoBotao = digitalRead(botaoModoFarol);
    if (estadoBotao && (estadoBotao != estadoBotaoAnt))
    {
      modoFarol++;
      if (modoFarol == 4)
      {
        modoFarol = 1;
      }
      Serial.print("modoFarol: ");
      Serial.println(modoFarol);
      delayBotao = millis();
    }
    estadoBotaoAnt = estadoBotao;
  }
}

void controlaLedsFarol(int modo)
{
  if (modo == 2)
  {
    //sempre ligado
    digitalWrite(ledFarol, HIGH);
  }
  else if (modo == 3)
  {
    //sempre desligado
    digitalWrite(ledFarol, LOW);
  }
  else
  {
    //modo automatico
    int leituraLuminosidade;
    leituraLuminosidade = analogRead(fotoResistor);
    // Serial.print("Leitura_luminosidade: ");
    // Serial.println(leituraLuminosidade);

    if (leituraLuminosidade < minimaLuminosidade)
    {
      digitalWrite(ledFarol, HIGH);
    }
    else if (leituraLuminosidade > maximaLuminosidade)
    {
      digitalWrite(ledFarol, LOW);
    }
    else
    {
      int intensidade = map(leituraLuminosidade, maximaLuminosidade, minimaLuminosidade, 0, 255); // é invertido pois quanto mais luz, menos o led acende
      if (abs(intensidade - intensidadeFarol) > 50)
      {
        intensidadeFarol = intensidade;
      }
      analogWrite(ledFarol, intensidadeFarol);
    }
  }
}
//valida se ele pode ir para frente ou para trás
int validarMovimentos()
{
  //  Serial.println("Validando movimentos");
  //  Serial.println(analogRead(sensorInfraFrente));

  if (digitalRead(sensorInfraFrente) == LOW)
  {
    digitalWrite(LED_BUILTIN, HIGH); //acende o led no arduino
    return 1;
  }
  // if (analogRead(sensorInfraTras) > DistanciaMaximaChao)
  // {
  //   //    digitalWrite(LED_BUILTIN, HIGH); //acende o led no arduino
  //   return 2;
  // }
  return 0;
  //colocar outros sensores
}

void loop()
{
  if (validarMovimentos() == 0)
  {
    andarParaFrente();
  }
  else
  {
    pararMotoresComTrava();
  }
  //  if (sonar.ping_cm() > 15) {
  //    digitalWrite(LED_BUILTIN, LOW); //apaga o led no arduino
  //    andarParaFrente();
  //
  //  } else {
  //    digitalWrite(LED_BUILTIN, HIGH); //acende o led no arduino
  //    pararMotores();
  //    int menorGrauDistancia = medirDistancias();
  //
  //  }
  //  andarParaFrente();
  //  virarParaDireita();

  //  pararMotores();
  //  medirDistancias();
  //  virarParaEsquerda();

  //  andarParaTras();

  //valida os cliques do botão e controla o farol
  mudarEstadoFarol();
  controlaLedsFarol(modoFarol);
}
