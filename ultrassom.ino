#include <Ultrasonic.h>
#include <stdio.h>
#include <string.h>
 
#define pino_trigger 4
#define pino_echo 5
Ultrasonic ultrasonic(pino_trigger, pino_echo);

#define TAMANHO_VETOR 11 //ímpar
#define ALFA 0.3
 
double distancia;
double distancia1;
double distancia2;
double distancia3;
long int cont1=0,cont2=0;
double vetor_media[TAMANHO_VETOR]; 
double vetor_mediana[TAMANHO_VETOR]; 
double mediana, media;
double valor;


void setup(){
  Serial.begin(9600);
  valor = ultrasonic.convert(ultrasonic.timing(), Ultrasonic::CM);
}

 
void filtro_media(){

  double soma=0.0; 
  int i;
  
  vetor_media[cont1] = distancia;

  if(cont2 >= TAMANHO_VETOR - 1){ 

    for(i=0;i<TAMANHO_VETOR;i++)
      soma+=vetor_media[i];
    
    media = soma/(TAMANHO_VETOR*1.0);
    distancia1 = media;
  }
}

void ordena(double v[]) {
   int i,j,aux;

   for(i=0;i<TAMANHO_VETOR-1;i++){

      for(j=0;j<TAMANHO_VETOR-1;j++){
         if(v[j]>v[j+1]){
            aux=v[j];
            v[j]=v[j+1];
            v[j+1]=aux;  
         }
      }
   }
}

void filtro_mediana(){

  int i;
  double vetor_ordenado[TAMANHO_VETOR];

  vetor_mediana[cont1] = distancia;

  if(cont2 >= TAMANHO_VETOR - 1){

    for(i=0;i<TAMANHO_VETOR;i++)
      vetor_ordenado[i] = vetor_mediana[i];
    
    ordena(vetor_ordenado);
    mediana = vetor_ordenado[(TAMANHO_VETOR-1)/2];
    distancia2 = mediana;
  }
}

void filtro_alfa(){ //Ignora grandes variações
  
  valor = ALFA * valor + (1-ALFA) * distancia;
  distancia3 = valor;

}

void printa(){

  Serial.print(distancia); //leitura
  Serial.print(" ");
  Serial.print(distancia1); //filtro media
  Serial.print(" ");
  Serial.print(distancia2); //filtro mediana
  Serial.print(" ");
  Serial.println(distancia3); // filtro alfa
  
}

void loop(){

  //printa();

  delay(100);
  long microsec = ultrasonic.timing();
  distancia = ultrasonic.convert(microsec, Ultrasonic::CM); //leitura

  //filtro_media();
  //filtro_mediana();
  filtro_alfa(); //esse foi melhor nos testes.
  
  cont1++;
  if(cont1 == TAMANHO_VETOR) cont1=0; 
  
  if(cont2 < TAMANHO_VETOR*2) cont2++;
  
}
