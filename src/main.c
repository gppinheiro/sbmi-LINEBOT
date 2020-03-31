// Projeto 17, ROBOT SEGUIDOR DE LINHA, criado por: Guilherme Pinheiro (up201703867) e Tomás Araújo (up201704738).
// --- Bibliotecas AVR ---
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

// --- Bibliotecas criadas por terceiros --
#include "rc5.h"

// --- Nossas bibliotecas ---
#include "biblio.h"
#include "lcd_gpta.h"

// --- Variáveis Defenidas ---
// MOTORA
#define INA1 5 //PC5 - A5
#define INA2 3 //PD3 - 3
#define PWMA 5 //PD5 - 5
// MOTORB
#define INB1 4 //PD4 - 4
#define INB2 7 //PD7 - 7
#define PWMB 6 //PD6 - 6
// Tracker Sensor
#define IR1 0  //PC0 - A0
#define IR2 1  //PC1 - A1
#define IR3 2  //PC2 - A2
#define IR4 3  //PC3 - A3
#define IR5 4  //PC4 - A4
// PID
#define KP 12
#define KD 2
#define KI 0
// Desmultiplexador
#define A 0    //PD0 - 0
#define B 1    //PD1 - 1

// --- Variáveis Globais ---
uint8_t EEMEM interval;
uint8_t volta, n_voltas,direcao,velocidade;
int16_t maximo[5],minimo[5];
int32_t ler_sensor[5],vetor_normalizado[5];
int tensao,val,val_anterior;
double proporcional=0,derivada=0,integral=0,antigo_proporcional=0;

// --- Funções criadas por nós ---
// LCD -> Quase todas as funções estão na biblioteca
void LCD_Print (int32_t tracker_position, int n_voltas, uint8_t dir, uint8_t vel) {
  LCD_Goto(0,0);
  LCD_Message("Posicao:");
  LCD_Integer(tracker_position);
  LCD_Goto(14,0);
  LCD_Integer(n_voltas);
  LCD_Goto(0,1);
  LCD_Message("Modo:");
  if (dir==0) { LCD_Message("esquerda"); }
  if (dir==1) { LCD_Message("direita"); }
  LCD_Goto(14,1);
  if (vel == 50) { LCD_Message("N"); }
  if (vel == 100) { LCD_Message("F"); }
  LCD_Goto(0,2);
}

// Desmultiplex
void ligaSaida (int x) {
  switch(x) {
    case 0:
    resetIO('D',A);
    resetIO('D',B);
    break;
    case 1:
    setIO('D',A);
    resetIO('D',B);
    break;
    case 2:
    resetIO('D',A);
    setIO('D',B);
    break;
    case 3:
    setIO('D',A);
    setIO('D',B);
    break;
  }
} 

// PWM
// - Inicio PWM 
void set_PWM(void) {
  TCCR0B=0;
  TIFR0 |= (7<<TOV0);
  TCCR0A=0b10100011;
  TIMSK0 = 0;
  TCCR0B = 0b00000010;
}
// - Duty-cycle PWM 
void set_duty (char c, int bit, float value) {
  int duty;
  duty=value*255/100;
  if (bit==6 && c=='D') {
    OCR0B=duty; //OCR0B é para o pino 3 do arduino
  }
  else if (bit==5 && c=='D') {
    OCR0A=duty; //OCR0A é para o pino 11 do arduino
  }
}

// Motores
// - Frente
void forward (void) {
  setIO('C',INA1);
  resetIO('D',INA2);
  setIO('D',INB1);
  resetIO('D',INB2);
}
// - Para trás
void reverse (void) {
  resetIO('C',INA1);
  setIO('D',INA2);
  resetIO('D',INB1);
  setIO('D',INB2);
}
// - Parar
void stop (void) {
  resetIO('C',INA1);
  resetIO('D',INA2);
  resetIO('D',INB1);
  resetIO('D',INB2);
}

// Analog Read
// - Inicialização ADC (Analog Digital Conversion)
void init_ADC (void) {
	// Definir Vref=AVcc
  ADMUX = ADMUX | (1<<REFS0);
  // Desativar buffer digital em PC0
  DIDR0 = DIDR0 | (1<<PC0) | (1<<PC1)| (1<<PC2) | (1<<PC3) | (1<<PC4);
  // Pré-divisor em 128 e ativar ADC
  ADCSRA = ADCSRA | (7<<ADPS0)|(1<<ADEN);
}
// - Função para ler ADC
uint16_t read_ADC (uint8_t ADC_channel) {
  ADMUX |= (1<<REFS0);
  ADMUX = (ADMUX & 0xF0) | (ADC_channel & 0x0F);
  ADCSRA |= (1<<ADSC);
  while(ADCSRA & (1<<ADSC));
  return ADC;
}

// Bateria
/* int bateria(void) {
  val = read_ADC(5);
  if(val != val_anterior){
      tensao=val*5/1024;
      val_anterior=val;
  }
  return tensao*100/5;
} */

// Tracker Sensor
// - Iniciar
void init_TrackerSensor(void) {
  // Inicia os INPUTS do sensor
  initIO('C',IR1,0);
  initIO('C',IR2,0);
  initIO('C',IR3,0);
  initIO('C',IR4,0);
  initIO('C',IR5,0);
  // Para a comparação não dar erro, colocar Min=Max(1023) e Max=Min(0)
  for(int i=0;i<5;i++){
		minimo[i] = 1023;
		maximo[i] = 0;
	}
}
// - Calibração
void Calibracao (void) {
  for(int j=0;j<400;j++) {
    for (int i=0;i<5;i++) {
      ler_sensor[i]=read_ADC(i);
      if (ler_sensor[i]>maximo[i]) maximo[i]=ler_sensor[i];
      if (ler_sensor[i]<minimo[i]) minimo[i]=ler_sensor[i];
      _delay_ms(2);
    }
  }
}
// - Lê sensor
void LerSensor (void) {
  for(int i=0;i<5;i++) {
    ler_sensor[i]=read_ADC(i);
    vetor_normalizado[i]= (ler_sensor[i]-minimo[i])*1000/(maximo[i]-minimo[i]);
  }
}
// - Posicao dos 3 sensores, os outros 2 só servem para aumentar o numero de voltas
int32_t PosicaoSensor (void) {
  int32_t denominador=0,numerador=0,posicao=0;

  for(int i=1;i<4;i++) {
    numerador+=((i-1)*1000)*vetor_normalizado[i];
    denominador+=vetor_normalizado[i];
  }

  posicao=numerador/(denominador*20);

  return posicao;
}
// - PID
int32_t PID(int32_t posicao) {
  proporcional = posicao - 50;
  derivada = proporcional - antigo_proporcional;
  integral += proporcional;
  antigo_proporcional = proporcional;
  return (proporcional*KP + derivada*KD + integral*KI);
}
// - Ler Linha
void LerLinha(void) {
  int32_t posicao,pid,dutyA,dutyB;

  if(vetor_normalizado[0]<150 && vetor_normalizado[4]<150 && vetor_normalizado[2]<150 && volta==0) { volta=1; n_voltas++; eeprom_update_byte(&interval,n_voltas); } 
  else if (vetor_normalizado[0]>900 && vetor_normalizado[4]>900 && volta==1) { volta=0; }

  if(vetor_normalizado[1]<150 && vetor_normalizado[3]<150 && direcao==0) { vetor_normalizado[3]=1000; } 
  else if(vetor_normalizado[1]<150 && vetor_normalizado[3]<150 && direcao==1) { vetor_normalizado[1]=1000; }

  posicao=PosicaoSensor();
  
  LCD_Print(posicao,n_voltas,direcao,velocidade);

  pid=PID(posicao);
  dutyA=velocidade-pid; 
  dutyB=velocidade+pid;

  if(dutyA < 100 && dutyA>0) set_duty('D',5,dutyA);
  if(dutyB < 100 && dutyB>0) set_duty('D',6,dutyB);
  if(dutyA >= 100) set_duty('D',5,100);
  if(dutyB >= 100) set_duty('D',6,100);
  if(dutyA <= 0) set_duty('D',5,0);
  if(dutyB <= 0) set_duty('D',6,0);
}

// Inicialização de tudo
void init_ALL (void) {
  ligaSaida(1);
  n_voltas=0;
  volta=0;
  //INPUTS
  set_PWM();
  init_ADC();
  SetupPorts();
  LCD_Clear();
  LCD_Init();
  init_TrackerSensor();
  Calibracao();

  //OUTPUTS
  initIO('C',INA1,1);
  initIO('D',INA2,1);
  initIO('D',PWMA,1);
  initIO('D',INB1,1);
  initIO('D',INB2,1);
  initIO('D',PWMB,1);
  initIO('D',A,1);
  initIO('D',B,1);
}
//////////////// End Functions //////////////////

// --- PRINCIPAL ---
int main (void) {
  uint16_t command;
  uint8_t ligar=0,estado=0,modo_fast=0,cmdnum,n_voltas_total = eeprom_read_byte(&interval);

  sei();
  RC5_Init();
  init_ALL();

  while(1) {
    LerSensor();

    if(RC5_NewCommandReceived(&command)) {
      RC5_Reset();

      if(RC5_GetStartBits(command) != 3) { /* ERROR */ }

      cmdnum = RC5_GetCommandBits(command);
      
      if(cmdnum == 9){
        direcao=0; //escolhe_esquerda
      }
      if(cmdnum == 11){
        direcao=1; //escolhe_direita
      }
      if(cmdnum == 3){
        ligar=1;
      }
      if (cmdnum == 5){
        ligar=0;
      }
      if (cmdnum == 15) {
        modo_fast=1;
      }
      if (cmdnum == 1) {
        modo_fast=0;
      }
    }

    if(ligar) {
      if (vetor_normalizado[0]>900 && vetor_normalizado[1]>900 && vetor_normalizado[2]>900 && vetor_normalizado[3]>900 && vetor_normalizado[4]>900) estado=0;
      else estado=1;

      switch(estado) {
        case 0:
          stop();
          ligaSaida(2);
          break;
        case 1:
          forward();
          ligaSaida(0);
          LerLinha();
          break;
      }
      if(modo_fast) { velocidade=60; }
      else { velocidade=50; }
    }
    else { stop(); ligaSaida(2); }
  }
}