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
#define motorASpeed 6
#define botaoModoFarol  8
#define sensorDistanciaGatilho 9
#define sensorDistanciaEco 10
#define pinoServoCabeca  11
#define buzzer 12 //buzina
#define ledParada  13
#define medidorDeTensao A0
#define sensorInfraFrente A1
#define sensorInfraTras A2
#define fotoResistor  A5

//Objetos globais
Servo servoCabeca;
NewPing sonar(sensorDistanciaGatilho, sensorDistanciaEco, MAX_DISTANCE);

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
  pinMode(ledParada, OUTPUT);
  pinMode(ledFarol, OUTPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorASpeed, OUTPUT);
  //  pinMode(motorB1, OUTPUT);
  //  pinMode(motorB2, OUTPUT);
  pinMode(botaoModoFarol, INPUT_PULLUP);
  servoCabeca.attach(pinoServoCabeca);
  servoCabeca.write(90);
  pinMode(buzzer, OUTPUT);
  pinMode(ledParada, OUTPUT);
  pinMode(fotoResistor, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(sensorInfraFrente, INPUT);
  pinMode(sensorInfraTras, INPUT);
  Serial.println("Sistemas iniciados");
}

void andarParaFrente() {
  Serial.println("Andando para frente");
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
}

void andarParaTras() {
  Serial.println("Andando para tras");
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
}

void virarParaDireita() {
  Serial.println("Virando para direita");
  //colocar comando do servo aqui
  //  digitalWrite(motorB1, HIGH);
  //  digitalWrite(motorB2, LOW);

}

void virarParaEsquerda() {
  Serial.println("Virando para esquerda");
  //  digitalWrite(motorB1, LOW);
  //  digitalWrite(motorB2, HIGH);
}

void pararMotores() {
  Serial.println("Parando motores");
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
}

void pararMotoresComTrava() {
  Serial.println("Parando motores com trava");
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, HIGH);
}

//retorna a direção do grau com maior distancia
int medirDistancias() {
  Serial.println("Medindo distância");
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
  Serial.print("Maior distância encontrada: ");
  Serial.print(distancia[grauDaMaiorDistancia]);
  Serial.println(" cm");
  return grauDaMaiorDistancia;
}

void mudarEstadoFarol() {
  if ((millis() - delayBotao) > TempoDebounce ) {
    estadoBotao = digitalRead(botaoModoFarol);
    if ( estadoBotao && (estadoBotao != estadoBotaoAnt) ) {
      modoFarol++;
      if (modoFarol == 4) {
        modoFarol = 1;
      }
      Serial.print("modoFarol: ");
      Serial.println(modoFarol);
      delayBotao = millis();
    }
    estadoBotaoAnt = estadoBotao;
  }
}

void controlaLedsFarol(int modo) {
  if (modo == 2)  {
    //sempre ligado
    digitalWrite(ledFarol, HIGH);
  } else if (modo == 3) {
    //sempre desligado
    digitalWrite(ledFarol, LOW);
  } else {
    //modo automatico
    int leituraLuminosidade;
    leituraLuminosidade = analogRead(fotoResistor);
    // Serial.print("Leitura_luminosidade: ");
    // Serial.println(leituraLuminosidade);
    int minimo = 200, maximo = 500;
    if (leituraLuminosidade < minimo) {
      digitalWrite(ledFarol, HIGH);
    } else if (leituraLuminosidade > maximo) {
      digitalWrite(ledFarol, LOW);
    } else {
      int intensidade = map(leituraLuminosidade, maximo, minimo, 0, 255);// é invertido pois quanto mais luz, menos o led acende
      if (abs(intensidade - intensidadeFarol) > 50) {
        intensidadeFarol = intensidade;
      }
      analogWrite(ledFarol, intensidadeFarol);
    }
  }
}
//valida se ele pode ir para frente ou para trás
int validarMovimentos() {
  //  Serial.println("Validando movimentos");
  //  Serial.println(analogRead(sensorInfraFrente));

  if (analogRead(sensorInfraFrente) > DistanciaMaximaChao) {
    digitalWrite(LED_BUILTIN, HIGH); //acende o led no arduino
    return 1;
  }
  if (analogRead(sensorInfraTras) > DistanciaMaximaChao) {
    //    digitalWrite(LED_BUILTIN, HIGH); //acende o led no arduino
    return 2;
  }
  return 0;
  //colocar outros sensores
}

void loop()
{
  if (validarMovimentos() == 0) {
    andarParaFrente();
  } else {
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
