//S0 is set to VCC and S1 to GND
#define TCS_OUT1 9
#define TCS_OUT2 10
#define TCS_S2 11
#define TCS_S3 12

//teste com LED
#define LED 4

#define MEDIANA 7
#define ALPHA 0.15
#define DP_BRANCO 1.25

double cores[3];
int medidas[MEDIANA], branco[2];

void setup(){
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT1, INPUT);
  pinMode(TCS_OUT2, INPUT);
  Serial.begin(9600);
  //teste com LED
  pinMode(LED, OUTPUT);
}
//loop para teste, a função a ser chamada é cor().
void loop(){
  delay(500);
  Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n");
  digitalWrite(LED, cor());
  
}

int cor() {
  le_sensor(TCS_OUT1);
  //Início teste (mostrar na tela valores lidos
  Serial.print("\n\nR1 = ");
  Serial.print(cores[0]);
  Serial.print("G1 = ");
  Serial.println(cores[1]);
  Serial.print("B1 = ");
  Serial.println(cores[2]);
  //Fim teste pt.1  
  branco[0] = media_dp(0);
  le_sensor(TCS_OUT2);
  //Continuação teste
  Serial.print("\nR2 = ");
  Serial.print(cores[0]);
  Serial.print("G2 = ");
  Serial.println(cores[1]);
  Serial.print("B2 = ");
  Serial.println(cores[2]);
  //Fim teste.
  branco[1] = media_dp(1);
  return branco[0] && branco[1];
}

int mediana(int *medidas) {
  float maximo = 0;
  int indice;
  for (int i = 0; i < 5 ; ++i)  {
    for (int j = 0; j < MEDIANA - i ; ++j) {
      if(medidas[j] > maximo) {
        maximo = medidas[j];
        indice = j;
      }
    }
    if(indice != MEDIANA - 1) {
      maximo = medidas[MEDIANA - 1];
      medidas[MEDIANA - i] = medidas[indice];
      medidas[indice] = maximo;
    }
    maximo = 0;
  }

  return medidas[3];

}

void le_sensor(int TCS_OUT) {
  //Vermelho
  digitalWrite(TCS_S2,LOW);
  digitalWrite(TCS_S3,LOW);
  for(int i=1; i<MEDIANA; i++){
    medidas[i] = pulseIn (TCS_OUT, digitalRead(TCS_OUT) == HIGH ? LOW : HIGH);
  }
  cores[0] = (ALPHA)*cores[0] + (1-ALPHA)*mediana(medidas);

  //Azul
  digitalWrite(TCS_S3,HIGH);
  for (int i = 0; i < MEDIANA; ++i) {
    medidas[i] = pulseIn(TCS_OUT,digitalRead(TCS_OUT) == HIGH ? LOW : HIGH);
  }
  cores[2] = (ALPHA)*cores[2] + (1-ALPHA)*mediana(medidas);

  //Verde
  digitalWrite(TCS_S2,HIGH);
  for (int i = 0; i < MEDIANA ; ++i) {
    medidas[i] = pulseIn(TCS_OUT,digitalRead(TCS_OUT) == HIGH ? LOW : HIGH);
  }
  cores[1] = (ALPHA)*cores[1] + (1-ALPHA)*mediana(medidas);

}

int media_dp(int i) {
  double m = (cores[0] + cores[1] + cores[2])/3;
  double dp = (sqrt(pow(cores[0] - m, 2) + pow (cores[1] - m, 2) + pow(cores[2] - m, 2)))/3;
  //Início teste
  Serial.print("\t m = ");
  Serial.print(m);
  Serial.print("\t dp = ");
  Serial.println(dp);
  //Fim teste  
  return(dp < DP_BRANCO);
}
