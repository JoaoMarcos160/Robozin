#include <Servo.h>
#include <NewPing.h>

//Configuracoes
#define MAX_DISTANCE 450 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define TempoDebounce 50 //Tempo para eliminar o efeito Bounce EM MILISEGUNDOS
#define DistanciaMaximaChao 60 //Valor limite de distancia retornado pelos sensores infravermelhos pra impedir quedas

//Componentes
#define ledFarol  3
#define motorA1  4 //é um motor apenas, pois para cada motor são 2 pinos mais o pino de controle
#define motorA2  5
#define motorB1  6
#define motorB2  7
#define botaoModoFarol  8
#define sensorDistanciaGatilho 9
#define sensorDistanciaEco 10
#define pinoServoCabeca  11
#define ledParada  13
#define medidorDeTensao A0
#define sensorInfraFrente A1
#define sensorInfraDireita A2
#define sensorInfraTras A3
#define sensorInfraEsquerda A4
#define fotoResistor  A5

//Objetos globais
Servo servoCabeca;
NewPing sonar(sensorDistanciaGatilho, sensorDistanciaEco, MAX_DISTANCE);

//controle do farol
bool estadoBotao;
static bool estadoBotaoAnt;
static int modoFarol = 2; //0 = automatico, 1 = ligado e 2 = desligado, ele começa com 2 mas na primeira passada ele vira 0
static unsigned long delayBotao = 0;

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
  pinMode(ledParada, OUTPUT);
  pinMode(ledFarol, OUTPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(botaoModoFarol, INPUT_PULLUP);
  servoCabeca.attach(pinoServoCabeca);
  servoCabeca.write(90);
  pinMode(ledParada, OUTPUT);
  pinMode(fotoResistor, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(sensorInfraFrente, INPUT);
  pinMode(sensorInfraDireita, INPUT);
  pinMode(sensorInfraTras, INPUT);
  pinMode(sensorInfraEsquerda, INPUT);
  Serial.println("Sistemas iniciados");
}

void andarParaFrente() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
}

void andarParaTras() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
}

void virarParaDireita() {
  //colocar comando do servo aqui
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);

}

void virarParaEsquerda() {
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void pararMotores() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
}

void pararMotoresComTrava() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, HIGH);
}

//retorna a direção do grau com maior distancia
int medirDistancias() {
  int distancia [5];
  int grauDaMaiorDistancia = 0;
  for (int contador = 0, cont2 = 0; contador <= 180; contador = contador + 45, cont2++) {
    servoCabeca.write(contador);
    delay(500);
    distancia[cont2] = sonar.ping_cm();
    if (distancia[cont2] > distancia[grauDaMaiorDistancia]) {
      grauDaMaiorDistancia = cont2;
    }
  }
  //movimento suave do servo
  //  for (int teste = 1000; teste < 2000; teste++) {
  //    servoCabeca.write(map(teste, 1000, 2000, 0, 180));
  //    delay(5);
  //  }
  servoCabeca.write(90);
  Serial.print("Maior distancia encontrada: ");
  Serial.print(distancia[grauDaMaiorDistancia]);
  Serial.println(" cm");
  return grauDaMaiorDistancia;
}

void mudarEstadoFarol() {
  if ((millis() - delayBotao) > TempoDebounce ) {
    estadoBotao = digitalRead(botaoModoFarol);
    if ( estadoBotao && (estadoBotao != estadoBotaoAnt) ) {
      modoFarol++;
      if (modoFarol == 3) {
        modoFarol = 0;
      }
      delayBotao = millis();
    }
    estadoBotaoAnt = estadoBotao;
  }
}

void controlaLedsFarol() {
  switch (modoFarol) {
    case 0:
      //modo automatico
      int leituraLuminosidade;
      leituraLuminosidade = analogRead(fotoResistor);
      if (leituraLuminosidade < 150) {
        digitalWrite(ledFarol, HIGH);
      } else if (leituraLuminosidade > 300) {
        digitalWrite(ledFarol, LOW);
      } else {
        analogWrite(ledFarol, map(leituraLuminosidade, 300, 500, 255, 5));
      }
      break;

    case 1:
      //sempre ligado
      digitalWrite(ledFarol, HIGH);
      break;

    case 2:
      //sempre desligado
      digitalWrite(ledFarol, LOW);
      break;
  }
}
//valida se ele pode ir para frente ou para trás
int validarMovimentos() {
  //Serial.println(analogRead(sensorInfraFrente));
  if (analogRead(sensorInfraFrente) > DistanciaMaximaChao) {
    pararMotores();
    digitalWrite(LED_BUILTIN, HIGH); //acende o led no arduino
    return 1;
  }
  //colocar outros sensores
}

void loop()
{
  //  validarMovimentos();
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
  andarParaFrente();
  virarParaDireita();
  delay(5000);
  virarParaEsquerda();
  delay(5000);

  //valida os cliques do botão e controla o farol
  mudarEstadoFarol();
  controlaLedsFarol();
}
