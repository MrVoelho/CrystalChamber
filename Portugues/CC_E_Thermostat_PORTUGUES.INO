/*
    Crystal Chamber - Um sketch para controle da temperatura e umidade relativa em ambiente fechado para crescimento de cristais salinos por evaporação
    Copyright (C) 2024  Voelho
	
	Este programa é um software livre: você pode redistribuí-lo e/ou modificá-lo
	de acordo com os termos da Licença Pública Geral GNU, conforme publicada pela
	Free Software Foundation, seja na versão 3 da Licença, ou em qualquer versão posterior.
	
	Este programa é distribuído com a esperança de que seja útil,
	mas SEM QUAISQUER GARANTIAS; nem mesmo a garantia implícita de
	COMERCIABILIDADE ou ADEQUAÇÃO A UM PROPÓSITO ESPECÍFICO. Veja a
	Licença Pública Geral GNU para mais detalhes.

	Você deve ter recebido uma cópia da Licença Pública Geral GNU
	junto com este programa. Se não, veja <https://www.gnu.org/licenses/>.
*/

/*
  Crystal Chamber - Calibração do termostato - Versão E 
  Uma rotina para clibração do ângulo do termostato em relação com a voltagem da fonte
  
  13 outubro 2024
  Por Voelho
  
  [Comentado em Português]
  
  Este sketch é parte do Projeto Crystal Chamber v1.1.G
  Para mais detalhes do algorítmo e esquemáticos, leia o arquivo de Documentação
  Para informação sobre as bibliotecas e licenças, leia o arquivo LICENCA
   
*/

//============================Bibliotecas

//Servo
#include <VarSpeedServo.h>
VarSpeedServo servoVar; 

//Display
#include <Arduino.h>
#include <U8x8lib.h>
U8X8_SSD1306_128X64_NONAME_SW_I2C display(/* clock=*/SCL, /* data=*/SDA, /* reset=*/U8X8_PIN_NONE); // Altere de acordo com seu display

//Botões
#include <PinButton.h>

//============================Definições
//_________________.Pinos
#define BT_mid A2       //Botão 1
#define BT_up A1        //Botão 2
#define PTec 4       //TEC
#define calPin A3     //Servo termostato

//_________________.Botões
PinButton BtUp(BT_up);
PinButton BtMid(BT_mid);

//_________________.Pinos
int posicao = 0; //Inicia em zero

void setup() {
  //Declarações
  pinMode(BT_mid, INPUT); 
  pinMode(BT_up, INPUT);  
  pinMode(PTec, OUTPUT);  
  digitalWrite(PTec, HIGH); 

  //Tela inicial
  displayInit();
  display.println("Calibrar");

  //Inicia o servo
  servoVar.attach(calPin);
  servoVar.stop();
  servoVar.write(posicao, 100, true);
  delay(2000);
  DispReset();
  
}

void loop() {
  //verifica os botões
  BtUp.update();
  BtMid.update();
  
  
  /*Interface
  Simples Up: +10 graus
  Duplo Up: reseta para zero
  Simples Mid: envia o ângulo e movimenta o servo
  
  Duplo Mid: chama a movimentação automática (função autocal)
  */
  if (BtUp.isSingleClick()) {
    posicao = posicao + 10;
    display.print("> ");
    display.println(posicao);
  }
  if (BtUp.isDoubleClick()) {
    posicao = 0;
    display.print("> ");
    display.println(posicao);
  }
  if (BtMid.isSingleClick()) {
    servoVar.write(posicao, 70, true);
    DispReset();
    display.print(posicao);
  }
  if (BtMid.isDoubleClick()) {
    DispReset();
    autocal();
  }
}
void autocal(){
//Move o servo pelas posições mais relevantes para a calibração com um delay para ler e anotar a voltagem
//Sequência: 0 60 120 180 20 80 140 40 100 160
  display.print("AutoCal");
  posicao = 0;
  autocalStep();
  posicao = 20;
  autocalStep();
  posicao = 40;
  autocalStep();
  posicao = 180;
  servoVar.write(posicao, 70, true);
  DispReset();
  display.print(posicao);
}
void autocalStep(){
  for (int i = 1; i <= 3; i++) { //3 incrementos
      //move e espera
      servoVar.write(posicao, 70, true);
      delay(4000); //2 segundos para ler e anotar
      posicao = posicao + 60;
      if(posicao > 180){ //teto
        posicao = 180;
      }
  }
}

//_________________.Display
void DispReset() {
  display.clearDisplay();  //limpa o buffer
  display.setCursor(0, 0);
}

void displayInit() {
  display.begin();
  display.setFont(u8x8_font_5x7_f);  //Fonte
  display.print(F(">"));
}
