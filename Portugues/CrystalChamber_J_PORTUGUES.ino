/*
    Crystal Chamber - Um sketch para controle da temperatura e umidade relativa em ambiente fechado para crescimento de cristais salinos por evaporação
    Copyright (C) 2025  Voelho
	
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
	
  Crystal Chamber
  Versão 1.2.J 
  17 junho 2025
  Por Voelho
  
  [Comentado em português]
  
  A versão é lida como x.y.z (estrutura, eletrônicos, sketch) - O sketch G é o primeiro publicado (12/2024)
  Para mais detalhes do algorítmo e esquemáticos, leia o arquivo de Documentação
  Para informação sobre as bibliotecas e licenças, leia o arquivo "LICENCA"
   
*/

//============================Bibliotecas
//Serial
#include <SPI.h>

//SD
#include <SD.h>
File myFile;
#pragma execution_character_set("utf-8")

//DHT
#include <DHT.h>

//Servo
#include <VarSpeedServo.h>
VarSpeedServo servo;       //cria objeto servo da umidade
VarSpeedServo elPescador;  //cria objeto servo do termostato

//Display
#include <Arduino.h>
#include <U8x8lib.h>
U8X8_SSD1306_128X64_NONAME_SW_I2C display(/* clock=*/SCL, /* data=*/SDA, /* reset=*/U8X8_PIN_NONE);  // Altere de acordo com seu display

//Botões
#include <PinButton.h>


//============================Definições
//_________________.Pinos
#define tachoPin 3   	//Tacômetro da ventoinha externa
#define PTec 4          //TEC (placa peltier)
#define buzzerPin 7  	//Buzzer
#define PServo 9        //Servo regulador de umidade
#define PVentilador 10  //Ventilador interno
#define Pvolt A0		//Pino de leitura do divisor de voltagem
#define BT_up A1        //Botão 1
#define BT_mid A2       //Botão 2
#define Ptermo A3  		//Servo termostato 

//_________________.Botões
PinButton BtUp(BT_up);
PinButton BtMid(BT_mid);

//_________________.DHT
DHT dht[] = {
  { 5, DHT22 }, //No pote da sílica gel
  { 6, DHT22 }, //Na câmara de exposição
  { 2, DHT22 },  //No ambiente externo
};

int humidity[3];     // Array de Umidade
int temperature[3];  // Array de Temperatura

//_________________.Variáveis de controle
// Operação
unsigned char OPE;          //Variável de controle de estado (0: Manutenção) (1: TU Operação, controle de temperatura) (2: TU Operação, controle de umidade) (3: T_ calibração da umidade sem servo - requer tampa fechada) (4: __ calibração sem controle térmico) (5: _U Controle da umidade em temperatura ambiente - Requer tampa aberta)
bool MAN = false;           //Flag que vai sinalizar se o sistema estava em manutenção, para chamar o retomaOp() ao retornar à operação.
bool vSD = false;           //Variável que vai acompanhar a presença do cartão SD true = tem false = nao tem
unsigned char visor = 0;    //Variável para acompanhar status do visor (0 - desligado) (1 - operacao) (2 - menu de MODO)
char intSel = 1;            //Variável que vai operar a seleção da interface (pode ser negativa)
unsigned char intLim = 0;   //Variável que guarda a quantidade de opções a serem selecionadas
unsigned char Uticks = 0;   //Variável que vai atrasar a resposta do controle da umidade para aguardar responsta do ambiente
unsigned char SDticks = 0;  //Variável que vai atrasar a escrita no cartão SD ( 0 = escreve )

//Média móvel
// arrays das médias: 5 valores e a soma
int mTin[6] = { 0, 0, 0, 0, 0, 0 };
int mUin[6] = { 0, 0, 0, 0, 0, 0 };
int mTex[6] = { 0, 0, 0, 0, 0, 0 };
// índices
unsigned char pTin = 0;
unsigned char pUin = 0;
unsigned char pTex = 0;
// Validadores
unsigned char vTin = 0;
unsigned char vUin = 0;
unsigned char vTex = 0;

// Acompanhamento
bool OPT = false;    //Variável de acompanhamento do status da TEC
bool Useco = false;  //Variável para registrar se umidade secou demais durante o ciclo de controle
bool intW = false;   //Variável que vai segurar o display durante as interações, para evitar sobrescrever/resetar toda hora
bool tampa = true;   //Detecção da tampa aberta ou fechada. Por padrão, considerar tampa fechada - evita que o ambiente aqueça em eventos de interrupção
bool HB = false; 	 //Variável que sinaliza a ativação do algorítmo HeatBooster

volatile unsigned long counter = 0;  //Contador para o RPM

int rpm = 5000;        //Counter do RPM - Inicia alto para permitir start da programação
char TS[2];            //Status da temperatura (0 - sentido: 1 = subindo -1 = descendo) (1 - ocorrências abaixo do mínimo)
int US[3];             //Status da umidade (0 - sentido) (1 - Posição do servo trapdoor) (2 - Variação do servo trapdoor)
char MoSt = 1;         //Status do monitor (0 - normal) (1 - Retomada Manutenção) (2 - Baixa Potencia) (3 - Baixa secagem) (4 - Alta potencia) (5 - Falha da ventoinha) (6 - Temperatura acima do limite)
int varM;              // Multiplicador da variação
int varV = 9000;       // Variação da média móvel de temperatura na última hora- inicialmente, considera a primeira constante para manter o fluxo e os 4 dígitos
int varVE = 9000;      // Variação da média móvel de temperatura externa na última hora
int varVU = 9000;      // Variação da média móvel de umidade na última hora
int adcRead = 0;           //Leitura do ADC
int Vtec = 0;              // Variável da média móvel de voltagem de saída
int Valvo = 300;           //Variável que mantém o alvo da voltagem, inicializada em 3 volts
unsigned int termSec = 0;  //Tempo de delay do termostato

int conT[3];  //Variável de acompanhamento da temperatura (0: Temperatura "anterior" - para comparativo da variação) (1: Média móvel de 5 leituras) (2:registro a 15 minutos atrás)
int conU[4];  //Variável de acompanhamento da variação de umidade em módulo (0: Umidade "anterior" - para comparativo da variação) (1: Média móvel de 6 leituras) (2: módulo da última variação) (3:registro a 15 minutos atrás)
int conE[3];  //Variável de acompanhamento da temperatura externa (0: registro a 15 minutos atrás) (1: Média móvel de 5 leituras) (2: Temperatura externa, referência para o Booster)

//Intervalos
unsigned long SDLT = 0;                       //Horário da última leitura
const unsigned long SDInterval = 20000;       //Intervalo de registro padrão de 20 segundos (Modos com RPM counter tem delay de 1 segundo)
unsigned long TECon = 0;                      //Horário quando ligou a TEC
unsigned long ULT = 0;                        //Horário que começou acompanhamento da umidade
unsigned long RPMLT = 0;                      //Horário que a ventoinha apresentou rpm abaixo do limite
unsigned long varLT = 0;                      //Horário de registro da última variação da média
const unsigned long varInterval = 900000;     //Registro da variação da média a cada 15 minutos
unsigned long intLT = 0;                      //Horário da última ação na interface
const unsigned long intInterval = 5500;       //Timeout para cada ação na interface de menu, ou sistema cancela
unsigned long termLT = 0;                     //Horário da última leitura do termostato
const unsigned long termInterval = 180000;    //Intervalo para funções do termostato (3 minutos)
unsigned long buzzerLT = 0;                   //Horário do último alerta
const unsigned long buzzerInterval = 600000;  //10 minutos para repetir alertas
unsigned long boostLT = 0;                    //Horário do último uso do HeatBooster

//Parâmetros

//Ângulos do servo de umidade
const unsigned char sClose = 52;  //Fechamento máximo do servo
const unsigned char sOpen = 0;    //Abertura máxima do servo

//Faixa de temperatura. Usa lógica de inteiros (25.0°C = 250)
const int Tsup = 235;
const int Talvo = 230;
const int Tinf = 220;
int Tb = Talvo;  // Buffer de religada da TEC quando reascender de uma queda abaixo de Tinf
//Faixa de umidade
const int Usup = 757;
const int Ualvo = 754;
const int Uinf = 750;
int dUmax = 10;  // variação de umidade máxima inicial
// Variáveis globais para o controle térmico
int DTc;  // Variável que vai compensar o DT
float DT = 0.0;  // Diferença de temperatura externa em relação ao alvo

//============================ARDUINO
void setup() {
  //Declarações
  pinMode(PFan, OUTPUT);   
  pinMode(PTec, OUTPUT);   
  pinMode(BT_mid, INPUT);  
  pinMode(BT_up, INPUT);   
  pinMode(PVentilador, OUTPUT); 

  //Inicializar variáveis
  OPE = 1;    // Modo de operação inicial
  TS[0] = 0;  //Sem variação de temperatura
  US[0] = 0;  //Sem variação de umidade
  US[2] = 1;  // Variação inicial do servo em 1 grau

  sTEC(0);
  digitalWrite(PVentilador, HIGH);  // Ventilador interno

  //Inicializar tela
  displayInit();
  display.println(F("CrystalChamber"));
  display.println("G");
  buzzerIntro();

  //Verificação da tampa
  sFan(1);  //Liga ventoinha da tampa
  delay(500);
  tacometro();  //Verifica resposta da ventoinha para inferir se tampa está aberta ou fechada
  display.println(rpm);
  if (rpm < 100) {   //menos que 100, pois talvez flutue maior que zero
    tampa = false;   //sem leitura no tacometro, está sem a tampa
  }
  sFan(0);  //desliga ventoinha da tampa

  //Inicializar DHT's
  for (auto &sensor : dht) {
    sensor.begin();
  }

  // Inicializar SD
  OperaSD();

  //Inicializar o servo com uma dica visual
  // O servo de umidade é fundamental para a operação do sistema, então uma movimentação rápida é feita 
  // para indicar que está funcionando - procedimento útil para detectar conexões em mal contato, soltas ou eventual defeito
  
  servo.attach(PServo); //Conecta o servo
  dbgServo(); // Check-up visual
  opServo(9);  // Começa fechado

  //Inicializar Termostato
  termoServo.attach(Ptermo);  // Servo contínuo do termostato
  termoServo.write(90, 255);  // Passa comando de parada

  delay(1000);   // A guarda para leitura inicial
  TempReg();     // Primeiro registro e leitura da voltagem
  delay(1000);   // Conforma
  Termostato();  // Primeiro ajuste
}

void loop() {
  //checa botões
  BtUp.update();
  BtMid.update();

  //Checa coleta de dados
  if ((millis() - SDLT) >= SDInterval) {
    TempReg();          //Controle e registro
    if (vSD == true) {  // Se tiver cartão inserido
      //Delay de escrita no SD - 2 minutos = 5 intervalos adicionais (ticks)
      if (SDticks < 5) {
        SDticks = SDticks + 1;
      } else {
        SDticks = 0;
      }
    } else {
      SDticks = 1;  // Se não tiver cartão, fixa SDticks diferente de zero
    }
    //Operação
    if (OPE == 0) {  // Condição de manutenção
      opServo(9);    // Fecha o servo
      //Desliga o conjunto
      sTEC(0);
      sVentilador(0);
    } else if (OPE == 4) {  // Desativa controle higrotérmico mas mantém ventilador interno ligado
      sTEC(0);
    } else {                   //Permitido controle de temperatura ou umidade
      if (OPE != 5) {          // Se for 5, sobrepõe controle de temperatura
        if (conT[1] > Tsup) {  // Acima da faixa alvo
          if (TS[0] == 1) {    // Se veio subindo da faixa
            Tb = Tb - 1;       //Reduz o ponto de religada
          }
          if (OPT == false) {  //Se TEC estiver desligada
            //Liga TEC (sempre junto aos ventiladores)
            sTEC(1);
            sVentilador(1);
          }
        } else if (conT[1] < Tinf) {  // Abaixo da faixa alvo
          //Desliga TEC mas mantém as ventoinhas.
          sTEC(0);
          if (TS[0] == -1) {    // Se veio descendo
            TS[1] = TS[1] + 1;  // Registra evento de queda de temperatura
          }
        } else {                 //Dentro da faixa alvo - Realizar o controle
          if (TS[0] == 1) {      //Subindo
            if (OPT == false) {  //TEC desligada
              if ((conT[1] >= Tb) && (conT[1] >= (Tinf + 2))) {  //Compara a temperatura atual com os pontos buffer ou Tinf + margem
                sTEC(1);
              }
            }
          }
          if ((OPE != 3) && (OPE != 2)) {  //Restrição de controle de umidade no modo calibração
            OPE = 2;                       //Libera controle de umidade
          }
        }
      }
      if ((OPE == 2) || (OPE == 5)) {  // Estado de controle da umidade
        //Desativar TEC quando entra no OPE = 5
        if (OPT == true && OPE == 5) {
          sTEC(0);
        }
        if (conU[1] > Usup) {                //Umidade acima do limite superior, então rotina de secagem
          if ((US[0] == 1) && (ULT == 0)) {  //subindo e sem registro prévio
            ULT = millis();                  //atualiza para momento que passou do limite superior
          }
          if (US[1] >= sClose) {  // Servo fechado, "condição inicial"
            opServo(-1);          //Abertura inicial
          } else {
            ajServo();  //Ajusta abertura e opera
          }
        } else if (humidity[1] < Uinf) {                       //Umidade média abaixo do superior, mas instantânea acima do limite inferior, rotina de evaporação
          if (US[0] == -1) {                                   //Umidade descendo
            opServo(9);                                        //Fechar o servo até voltar no alvo
            Useco = true;                                      //registra condição de controle
            US[2] = US[2] / 2;                                 //Reduz variação pela metade
            if (US[2] < 1) { US[2] = 1; }                      //garante variação mínima de 1
          } else if ((US[1] >= sClose) && (Useco == false)) {  //Condição inicial ou vindo de um reset
            opServo(-1);                                       //Abrir
          } else {
            ajServo();  //Ajusta abertura e opera
          }
        } else {           // Umidade dentro da faixa, operar e manter
          if (ULT != 0) {  //Se houver registro de controle temporal
            ULT = 0;       //Zera
          }
          if ((US[0] == 1) && (conU[0] > Ualvo)) {  //subindo, acima do alvo
            if (Useco == true) {                    //veio de secagem excessiva
              opServo(-1);                          //Abre
              Useco = false;                        //Desativa status de secagem
            } else {                                //Abre mínimo
              US[2] = 1;
              opServo(-1);
            }
          } else if ((US[0] == -1) && (conU[0] < Ualvo)) {  //descendo, abaixo do alvo
            US[2] = 1;
            opServo(1);  //fecha mínimo
          }
        }
      }
    }

    //Verifica RPM
    if (OPT == true) {       //Verifica apenas se a TEC estiver ligada
      tacometro();           //Verifica o RPM
      if (tampa == false) {  //Se o indicativo for que a tampa está desconectada
        if (rpm > 100) {     //Verifica o rpm
          tampa = true;      //Sinaliza que a tampa está conectada
        }
      }
    }

    monitor();  //Verificação a cada SDInterval (para não travar a interface com a tela ligada)
  }

  //Interface
  //Up duplo - Display reset - Ação de reset fora dos condicionantes de tela
  if (BtUp.isDoubleClick()) {
    displayInit();
  }
  //Controles da interface
  switch (visor) {
    case 1:  // TELA 1: ligada em operação:
      //Mid duplo - Alert Override
      if (BtMid.isDoubleClick()) {
        retomaOp();  // Estado de retorno da manutenção, para forçar o reset do Monitor()
      }
      //Mid simples - Menu
      if (BtMid.isSingleClick()) {
        //Abre o menu
        visor = 2;
        intLT = millis();  // registra interação
        buzzerClic();
        DispReset();  //Reset
      }
      //Up simples - Display off
      if (BtUp.isSingleClick()) {
        displayOff();
        buzzerLow();
      }
      break;
    case 2:  // TELA 2: Menu aberto
      intLim = 5;
      if ((millis() - intLT) <= intInterval) {  // Verifica timeout da interface
        if (intW == false) {                    //Turno do sistema, não está aguardando resposta
          //limpa linhas de opções
          opClear();
          //Coloca ponteiro
          display.setCursor(0, 0);
          display.print(F(" Modo "));
          display.setCursor(0, (intSel + 2));
          display.setInverseFont(1);
          display.print(">");
          display.setInverseFont(0);
          //Mostrar opções (Obs: Usar set cursor + print consome menos memória que printString)
          display.setCursor(2, 2);
          display.print(F("TU"));
          display.setCursor(2, 3);
          display.print(F("Man."));
          display.setCursor(2, 4);
          display.print(F("T_"));
          display.setCursor(2, 5);
          display.print(F("__"));
          display.setCursor(2, 6);
          display.print(F("_U"));
		  display.setCursor(2, 7);
          display.print(F("CL"));
          intW = true;  //passa o turno - aguardar resposta
        }
        // Botões
        // Up simples - Subir
        if (BtUp.isSingleClick()) {
          intSel = intSel - 1;
          passe();
          buzzerClic();
        }
        // Mid simples - Confirmar
        if (BtMid.isSingleClick()) {
          //confirma
          buzzerHigh();
          switch (intSel) {
            case 0:  // reiniciar operação
              OPE = 1;
              break;
            case 1:
              OPE = 0;
              break;
            case 2:
              OPE = 3;
              break;
            case 3:
              OPE = 4;
              break;
            case 4:
              OPE = 5;
              break;
			case 5: 
              MoSt = 0; 
              break;
          }
		  opClear();
          passe();
          visor = 1;  //Tela retorna
        }
        //Mid duplo - Opera o SD
        if (BtMid.isDoubleClick()) {
          OperaSD(); 
        }
      } else {
        visor = 1;  //Timeout do menu, retorna a operação normal
        passe();
      }
      break;
    default:  //TELA 0: desligada, aguarda ordem de ligar
              //Up simples - ligar tela
      if (BtUp.isSingleClick()) {
        displayOn();
        DispReset();
        display.println(F("-Load-"));
        buzzerHigh();
      }
      monitor();  //Verifica com tela desligada a cada ciclo
      break;
  }
}
//============================Funções operacionais
//_________________.Funções de alerta/monitoramento
void monitor() {
  //Controla período de tolerâncias após manutenção
  if (MoSt == 1) {                         //Períoto de tolerância pós-manutenção
    if ((millis() - TECon) >= 240000UL) {  //Aguarda 4 min (rotina retomaOp reseta o timer TECon, mesmo com a TEC desligada)
      MoSt = 0;                            //Finaliza o período de tolerância
    }
  }

  if (OPE == 0) {
    display.setCursor(0, 1);
    display.print(F("MAN"));
    MAN = true;
  } else {
    if (MAN == true) {  // Se estava em manutenção, chama reset geral
      retomaOp();
    }
    MAN = false;  //desativa estado de manutenção


    // Condição de segurança: TEC ligada com Ventoinha funcionando bem
    if (OPT == true) {
      if (RPMLT == 0) {    //Ventoinha sempre ok
        if (rpm < 1900) {  // Ventoinha falhou na rotação, sinaliza para desligar - Usual: 2100 RPM de leitura (!) verifique o RPM típico da sua ventoinha e ajuste
          RPMLT = millis();
        }
      } else {
        if ((millis() - RPMLT) > 60000UL) {  //Ventoinha 60 segundos abaixo do esperado, verificar condição de falha
          if (rpm > 2000) {                  // possível falso positivo, se a FAN voltou a rodar como esperado, cancela
            RPMLT = 0;
          } else {  //não voltou, desliga e dispara manutenção
            MoSt = 5;
            OPE = 0;
          }
        }
      }
    }
    //Condições operacionais
	if ((OPE >= 1) && (OPE <= 3)) {  //Apenas para os modos operacionais com controle térmico
      //Falha de resfriamento (Ex: caso de problema na ventoinha interna da tampa)
      if (MoSt != 1) {                // Pula verificação no período de tolerância, para garantir start em condições aquecidas e retorno de manutenção
        if (conT[1] > (Tsup + 20)) {  //Temperatura acima de 2 graus acima do limite superior
          if (TS[0] == 1) {           //Temperatura subindo
            OPE = 0;                  //Interrompe o funcionamento por segurança, pode ser problema de algum componente
            MoSt = 6;                 //Flag
          }
        }
      }
    }
    // Baixa capacidade de secagem
    if ((OPE == 2) && (ULT != 0)) {
      if (((millis() - ULT) / 3600000UL) > 2.0) {  //2 horas após início do registro de umidade superior
        if ((conU[1] > Usup)) {                    //Acima da faixa
          MoSt = 3;                                // Baixa secagem
        }
      }
    }
    // Muitas quedas abaixo do mínimo de temperatura
    if (TS[1] > 4) {
      if (MoSt == 7) {  // Registro limite de temperatura abaixo da mínima somado a um caso de falha do DHT
        OPE = 0;        // Interrompe operação
      } else {
        MoSt = 4;  // alta potencia
      }
    }
  }

  if (MoSt != 0) {  //Há um alerta, reporta
    display.setCursor(4, 1);  //posiciona
    if (MoSt == 1) {
      display.print(F("start"));
    } else {
      //Aviso buzzer
      if ((millis() - buzzerLT) >= buzzerInterval) {  //Verificação temporal
        buzzerAlert();                                //Toca melodia de alerta
        buzzerLT = millis();                          //Sinaliza que tocou
      }
      if (visor == 0) {  //Se não for visor 0, tela já está ligada
        displayOn();
      }
    }
    switch (MoSt) {  // Printa o erro
      case 3: display.print(F("BSec")); break;
      case 4: display.print(F("APot")); break;
      case 5: display.print(F("FAN")); break;
      case 6: display.print(F("TIn")); break;
      case 7: display.print(F("DHT")); break;
    }
  }
}
//_________________.Retomada da manutenção
void retomaOp() {
  //Função que reseta valores e retoma padrões para retorno da operação após manutenção
  TS[0] = -1;  // supõe queda para evitar loop com condição MoSt = 6
  TS[1] = 0;
  US[0] = 1;
  dbgServo();
  opServo(9);  //Fecha
  conT[0] = 0;
  conU[0] = 0;
  conU[2] = 0;
  Tb = Talvo;
  ULT = 0;
  RPMLT = 0;
  rpm = 5000;
  MoSt = 1;  // Reseta status de alerta e sinaliza tolerância para verificação da temperatura interna
  conE[1] = 0;  //Reset imediato da leitura externa
  display.setCursor(0, 0);
  display.println(F("-ReLoad-"));
  varV = 9000;       //Reseta indicador de variação de temperatura
  varVU = 9000;      //Reseta indicador de variação de umidade
  varVE = 9000;		 //Reseta indicador de variação da temperatura externa
  varLT = millis();  //reseta counter de variação
  TECon = millis();  //reseta counter da potencia
  Useco = false;
  tampa = false;
  sVentilador(1);  //reinicia ventilação
  memset(mTin, 0, sizeof(mTin));
  memset(mUin, 0, sizeof(mUin));
  memset(mTex, 0, sizeof(mTex));
  pTin = 0;
  pUin = 0;
  pTex = 0;
  vTin = 0;
  vUin = 0;
  vTex = 0;
  DTc = 0;
  TempReg(); 
  delay(250);
  Termostato();  //Reinicia o termostato
}

//_________________.DHT + SD
void leTMP() {
  if (temperature[1] == 0) {  //mal contato ou desconexão do sensor
    conT[1] = conT[0];        //Mantém a leitura anterior
    displayOn();
    MoSt = 7;  //avisa falha do DHT
  } else {
    //Cálculo da nova média
    mediaAdd(mTin, pTin, vTin, temperature[1]);
    conT[1] = mediaCal(mTin, vTin);
    if (MoSt == 7) {  // Reseta a flag do DHT caso ele volte a funcionar
      MoSt = 0;
    }
  }

  // Verifica variação e atualiza referência
  if (conT[1] > conT[0]) {  //Subindo
    TS[0] = 1;
  } else if (conT[1] < conT[0]) {
    TS[0] = -1;
  } else {
    TS[0] = 0;
  }
  conT[0] = conT[1];  //Atualiza referência para próxima leitura

  //Histórico da variação
  if (varLT == 0) {
    conT[2] = conT[0];
    conU[3] = conU[0];
    varLT = millis();
  }

  //Verifica se é hora de atualizar
  if ((millis() - varLT) >= varInterval) {
    //Atualiza histórico de temperatura
    if (conT[0] > conT[2]) {
      varM = 7000;
      //Aumento de temperatura, verifica se existe DTC para quente e ajusta
      if (DTc < 0) { TermoAjuste(); }
    } else if (conT[0] < conT[2]) {
      varM = 2000;
      //Queda de temperatura, verifica se existe um DTC para o frio e ajusta
      if (DTc > 0) { TermoAjuste(); }
    } else {
      varM = 1000;
      //Temperatura estabilizada - Ajusta DTc para compensar a curva do Termostato
      TermoAjuste();
    }
    conT[2] = conT[0];      //Atualiza histórico de <varInterval> minutos atrás
    varV = int(varV / 10);  //Avança o ponteiro 1 casa
    varV = varV + varM;     //Adiciona o novo status no início

    //Atualiza histórico de umidade
    if (conU[0] > conU[3]) {
      varM = 7000;
    } else if (conU[0] < conU[3]) {
      varM = 2000;
    } else {
      varM = 1000;
    }
    conU[3] = conU[0];        //Atualiza histórico de <varInterval> minutos atrás
    varVU = int(varVU / 10);  // Avança o ponteiro 1 casa
    varVU = varVU + varM;     //Adiciona o novo status no início
	
	//Atualiza histórico da temperatura externa
	if (conE[1] > conE[0]) {
      varM = 7000;
    } else if (conE[1] < conE[0]) {
      varM = 2000;
    } else {
      varM = 1000;
    }
	conE[0] = conE[1];        //Atualiza histórico de <varInterval> minutos atrás
    varVE = int(varVE / 10);  //Avança o ponteiro 1 casa
    varVE = varVE + varM;     //Adiciona o novo status no início

    varLT = millis();  //Atualiza horário do último registro
	
	//HeatBooster - Verificação para desligar
    if (HB == true) {  //HeatBooster ligado, checa estabilidade da Temp. Externa
      calc = (int)(varVE / 100);
      switch (calc) {  //ignora casos 17 e 19
        case 11:
        case 12:
        case 21:
        case 22:
          HB = false;   //desliga
          conE[2] = 0;  //zera referência
          break;
      }
    }
  }

  if ((millis() - termLT) >= termInterval) {  //Avaliação da temperatura externa
    //Atualiza média nova
    mediaAdd(mTex, pTex, vTex, temperature[2]);
    conE[1] = mediaCal(mTex, vTex);
	
	//Algorítmo HeatBooster - Acréscimo de potência que antecipa a resposta de aumento da temperatura externa
    //Gatilho : temperatura externa média subindo
    if (conE[1] > conE[0]) {  //Média atual maior que o registro
      if (conE[2] == 0) {     //Sem ponto de referência para observar
        conE[2] = conE[1];    //Registra a temperatura média do momento
        boostLT = millis();   //Registra o momento
      }
    } else if (conE[1] == conE[0]) {  //Caso constante
      if (conE[2] != 0) {             //Se houver um registro
        boostLT = millis();           //Atualiza o momento, para manter monitoramento em caso de aquecimento ser retardado no tempo
      }
    } else {        //Média caindo
      conE[2] = 0;  //Zera
      boostLT = 0;  //Zera
    }

    //Verifica o booster
    if ((OPE >= 1) && (OPE <= 3)) {  //Modos de controle térmico
      if ((conE[2] != 0) && (HB == false)) {//Há um registro e o booster está desligado
        if ((millis() - boostLT) > 6000) { // Verificação de afastamento de 6 segundos para evitar erro de divisão por zero
          if (iArr((100 * (conE[1] - conE[2])) / ((millis() - boostLT) / 6000)) > 4) {  //Taxa de aquecimento observada maior que [0,245°C / 60 min] (!) Verificar a taxa usual nas suas condições para que o algoritmo seja efetivo
            DTc = DTc + 6; //Acréscimo de 0.6 °C no DTC (!) determinado experimentalmente, convém experimentar para ajustar corretamente a seu arranjo físico
            HB = true;
            // Reinicia o acompanhamento da média externa
            conE[0] = conE[1];
            varVE = 9000;
          }
        }
      }
    }
	
    Termostato();       //Chama controle
    termLT = millis();  //atualiza horário da última verificação
  }
}

void leUR() {
  //Chamada para atualizar a média móvel de umidade ou produzir média parcial
  mediaAdd(mUin, pUin, vUin, humidity[1]);
  conU[1] = mediaCal(mUin, vUin);

  // Verifica variação e atualiza referência
  if (Uticks == 3) {                   //Número de ciclos até realizar avaliação da umidade
    conU[2] = abs(conU[1] - conU[0]);  //Registra o módulo da variação atual em relação à referência

    if (conU[1] > conU[0]) {  //Subindo
      US[0] = 1;
    } else if (conU[1] < conU[0]) {  //Descendo
      US[0] = -1;
    } else {  //constante
      US[0] = 0;
    }
    conU[0] = conU[1];  //Atualiza referência para próxima leitura
    Uticks = 1;         //reinicia contador
  } else {
    Uticks = Uticks + 1;
  }
}
//________________.Leituras e escritas
void TempReg() {
  if (OPE != 0) {  //não lê em manutenção
    //Leitura é em float, função iArr() passa para integer arredondado (ex. 23,56 é armazenado como 236)
    //(!) atualize as funções de calibração do DHT de acordo com seus sensores
    temperature[0] = iArr(((dht[0].readTemperature()) * 0.9927) - 0.2018);  // DHT I
    temperature[1] = iArr(((dht[1].readTemperature()) * 0.9669) + 0.6177);  // DHT III
    temperature[2] = iArr(((dht[2].readTemperature()) * 0.9621) + 1.0308);  // DHT II - Externo

    //Determina qual das curvas de calibração de umidade deve ser seguida (Os sensores DHT sofrem variações de acordo com a faixa de umidade, recomendo a calibração em pelo menos duas temperaturas)
    if (conT[1] < 260) {
      humidity[0] = iArr(((dht[0].readHumidity()) * 0.9675) - 0.9138);  // DHT I
      humidity[1] = iArr(((dht[1].readHumidity()) * 0.9037) + 1.0347);  // DHT III
      humidity[2] = iArr(((dht[2].readHumidity()) * 0.9469) - 1.8105);  //DHT II
    } else {
      humidity[0] = iArr(((dht[0].readHumidity()) * 1.0858) - 10.828);  // DHT I
      humidity[1] = iArr(((dht[1].readHumidity()) * 1.0105) - 8.203);   // DHT III
      humidity[2] = iArr(((dht[2].readHumidity()) * 1.0581) - 11.362);  //DHT II
    }
    //Faz as leituras, temperatura por último por atualizar o histórico de 15 min de ambas
    leUR();   // Passa a umidade para a média móvel e controle
    leTMP();  // Passa temperaturas para a média móvel e controle
  }
  
  //Verificação da tensão da TEC
  if (OPT == 1) {  //TEC ligada
	  Vtec = tensaoTEC(); //Lê tensão
  } else {
    Vtec = 0;  //Registra como zero pois a TEC está desligada
  }
  
  if (SDticks == 0) {
    myFile.print(millis());
    quebra();
  }

  //Varre os sensores
  if (visor == 1) {
    DispReset();
  }
  for (int i = 0; i < 3; i++) {
    //Escreve no SD
    if (SDticks == 0) {
      myFile.print(temperature[i] / 10.0, 1);  //Faz a conversão em ponto flutuante para registrar o decimal
      quebra();
      myFile.print(humidity[i] / 10.0, 1);
      quebra();
    }
    //Verifica se a tela está ligada, no modo de operação
    if (visor == 1) {
      //Printa informação no display
      display.print(i + 1);
      display.print(F(" T:"));
      display.print(temperature[i] / 10.0, 1);
      display.print(F(" U:"));
      display.println(humidity[i] / 10.0, 1);
    }
  }
  if (visor == 1) {
    //Médias móveis
    display.setCursor(11, 1);
    display.print(varVE);

    display.setCursor(0, 5);
    display.print(rpm);
    display.print(F("RPM"));
    display.print(OPT);

    display.setCursor(9, 5);
    display.print(US[1]);  //Posição do servo
    display.print(":");
    display.print(US[2]);  //Variação do servo
    //Monitor de variação
    display.setCursor(0, 6);
    //Mostra variação registrada de temperatura
    display.print("T");
    display.print(varV);
    display.print("|");
    //Mostra número de erros de queda
    display.print(TS[1], DEC);
    display.print("| ");
    //display sobre termostato
    display.print(Vtec / 100.0, 1);  //Mostra a voltagem atual em décimos
    display.print(" ");
    display.println(DTc);  //Mostra a compensação DTc
	//Mostra variação registrada de umidade
    display.print("U");
    display.print(varVU);
    display.print("|");
    //Mostra minutos até próxima leitura
    display.print((int)((varLT + varInterval - millis()) / 60000UL));

    //Média externa conE
    display.setCursor(10, 7);
    display.print("E");
    display.print(conE[1] / 10.0, 1);
  }
  if (SDticks == 0) {
	myFile.print(DTc);  //Variável DTc
	quebra();
	myFile.print(US[1]);  //Abertura do servo de umidade
    quebra();
    myFile.print(Vtec / 100.0, 2);  //Tensão da TEC
    quebra();
    myFile.print(HB); //Status do HeatBooster
    myFile.println("");  //quebra de linha
    myFile.flush();      // Registra
  }
  SDLT = millis();  //atualiza último horário
}
void quebra() {  //Poupa memória repetindo função de escrever o delimitador do .txt
  myFile.print(F(";"));
}
//_________________.Funções de liga/desliga
void sTEC(int x) {
  //Controle x (0: desligar, 1: ligar)
  switch (x) {
    case 0:
      digitalWrite(PTec, LOW);
      OPT = false;
      break;
    case 1:
      digitalWrite(PTec, HIGH);
      TECon = millis();  //Registra quando ligou a TEC
      OPT = true;
      break;
  }
}

void sVentilador(int x) {  // Ventilador da câmara
  //Controle x (0: desligar, 1: ligar)
  switch (x) {
    case 0:
      digitalWrite(PVentilador, LOW);
      break;
    case 1:
      // Modos 4 e 5 requerem tampa aberta
      if (OPE == 4 || OPE == 5) {
        if (tampa == false) {  //permite ligar ventilador da câmara somente sem a tampa
          digitalWrite(PVentilador, HIGH);
        }
      } else {
        digitalWrite(PVentilador, HIGH);
      }
      break;
  }
}
//_________________.Ventoinhas
void tacometro() {
  counter = 0;                                                       // reseta counter RPM
  attachInterrupt(digitalPinToInterrupt(tachoPin), pulsos, RISING);  //Evento de leitura
  delay(1000);                                                       //espera e conta
  rpm = counter * 30;                                                // converte em RPM
  detachInterrupt(digitalPinToInterrupt(tachoPin));                  //Desconecta pelos próximos segundos
}
//_________________.Display
void DispReset() {
  display.clearDisplay();  //clear buffer
  if (vSD == false) {
    display.setCursor(9, 0);
    display.print(F("SD"));  //Sinaliza falta do SD
  } 
  if (HB == true) {
	display.setCursor(11, 0);
    display.print(F("+"));  //Sinaliza uso do HeatBooster
  }
  
  display.setCursor(0, 0);
  switch (OPE) {
    case 0:
      display.setInverseFont(1);
      display.print(F("STOP"));
      display.setInverseFont(0);
      break;
    case 1:
      display.print(F("T-mode"));
      break;
    case 2:
      display.print(F("U-mode"));
      break;
    case 3:
      display.print(F("T_"));
      break;
    case 4:
      display.print(F("__"));
      break;
    case 5:
      display.print(F("_U"));
      break;
  }
  display.setCursor(0, 2);
}

//_________________.Cartão SD
void OperaSD() {
  display.setCursor(0, 1);
  display.print(F("SD... "));
  if (vSD == true) {  //Permite a remoção
    myFile.close();
    vSD = false;
    delay(250);  //Aguardar o fechamento geral
    display.println(F("Off"));
    buzzerLow();
  } else if (vSD == false) {  //Iniciar ou reiniciar o cartão após inserir
    delay(250);               //Delay para garantir que sistema está operando bem
    if (SD.begin(8)) {
      myFile = SD.open("datalog.txt", FILE_WRITE);
      if (myFile) {
        display.println(F("Ok"));
        buzzerClic();
        vSD = true;   //registra que o cartão está presente
        SDticks = 0;  // reinicia SDTicks
      }
    } else {
      display.println(F("Ausente"));
      buzzerAlert();
    }
  }
}

//_________________.Servo
void opServo(int sentido) {
  int servoPos;    // Variável local: Posição a escrever
  int solicitado;  //Declaração local
  
  // Sentido negativo = abrir, positivo = fechar 

  if (sentido == 9) {  //Caso inicial, fechar a porta e inicializar posição
    servoPos = sClose;
  } else {
    solicitado = US[1] + (sentido * US[2]);
    //Processa nova abertura
    if (solicitado <= sOpen) {  //Limite de abertura máxima
      servoPos = sOpen;
    } else if (solicitado >= sClose) {  //Limite de fechamento máximo
      servoPos = sClose;
    } else {
      servoPos = solicitado;
    }
  }
  servo.write(servoPos, 80);  //Executa movimento
  US[1] = servoPos;           //Registra posição da porta
  delay(500);                 //Conformar
}
void ajServo() {
  // Função de ajuste da variação da porta com base na variação de umidade
  
  // Sistema de freio da variação de umidade em função do alvo
  if (abs(conU[1] - Ualvo) > 50) {
    dUmax = 10;
  } else if (abs(conU[1] - Ualvo) > 20) {
    dUmax = 5;
  } else {
    dUmax = 2;  //para garantir proximidade com o ponto de equilíbrio de forma suave, porém evitando a margem de erro do sensor
  }
  if (US[0] != 0) {                 //Ajustar a variação somente quando houver variação
    if (conU[2] < dUmax) {          // Variação abaixo da máxima permitida
      US[2] = int(US[2] + 1);       //Aumenta o passo
      US[2] = min(US[2], 5);        //Limita a variação máxima em 5, para evitar descontrole e rebote
    } else if (conU[2] >= dUmax) {  //Variação acima ou igual à máxima permitida
      US[2] = int(US[2] / 2);       //Reduz o passo
      US[2] = max(US[2], 1);        //Limita a variação mínima em 1, para evitar estagnação.
    }
  }
  //Instrução de ajuste de acordo com estado e variação
  if (conU[1] > Usup) {        // Acima
    if (US[0] >= 0) {          //Acima, subindo ou estacionado
      opServo(-1);             //Abrir
    } else if (US[0] == -1) {  //Acima, descendo
      if (conU[2] >= dUmax) {  // Variação maior
        opServo(1);            //Fechar
      } else {                 // Variação menor
        opServo(-1);           //Abrir
      }
    }
  }
}
void dbgServo() {
  //Função que movimenta o servo no start e override para debug visual em caso de mal contato
  servo.write(25, 80, true);  //Abre
  delay(2000);
  servo.write(50, 80, true);  //Fecha
}
//_________________.Termostato
void Termostato() {
  //verifica modos de operação com controle de temperatura
  if ((OPE >= 1) && (OPE <= 3)) {
    float DTfun = 0.0;  // Variável para realizar a função
	DT = 0.0;     // Diferença de temperatura externa em relação ao alvo (pode ser negativa)


    //só trabalha se não houver mal contato no DHT
    if (temperature[2] != 0) {

      //Calcula variação e converte em posição do servo
      if (conE[1] == 0) {
        DT = (temperature[2] - Talvo) / 10.0;  //Início da operação, sem média ainda (ou após reset) - Usa temperatura instantânea
      } else {
        DT = (conE[1] - Talvo + DTc) / 10.0;  // Com média
      }

      // Função quadrática Graus(voltagem)
      DTfun = (2.49 * DT * DT) + (32.45 * DT) + (118.48);  //DT para Voltagem (dV)
      // Limitação de domínio (!) Ajuste conforme seu regulador de voltagem
      if (DTfun < 160) { //mínimo de 1.6 volts
        DTfun = 160;
      } else if (DTfun >= 1120) { //máximo de 11.2 volts
        DTfun = 1120;
      }
      Valvo = (int)DTfun;  //Truncado como integer
	  
	  //Verifica necessidade de ajuste
      if (OPT == true) {  //Confirma que a TEC está ligada, dispensa o processamento caso não esteja
        //Registro dos dados de temperatura em equilíbrio para refino das curvas de calibração
        if ((int)(varV / 10) == 111) {  //3 turnos de temperatura constante
          if (vSD == true) {            //checa presença do cartão
            myFile.print(millis());
            quebra();
            myFile.print("A");  //Tag para separação do registro (útil para filtragem no excel)
            quebra();
            myFile.print(conE[1] / 10.0, 1);  //Média temp. externa
            quebra();
            myFile.print(conT[2] / 10.0, 1);  //Média da temperatura nos <varInterval> minutos anteriores
            quebra();
            myFile.print(Vtec / 100.0, 2);  //Leitura de voltagem média
            myFile.println("");             //quebra de linha
            myFile.flush();                 // Registra
          }
        }

		//Verificação da tolerância para ajuste (!) Varia de acordo com o arranjo físico, colete dados para um ajuste adequado
		//Atual: Amplitude (0.3v) Desvio padrão (0.08v) Amplitude + (0.16) Amplitude - (0.08)
        if (abs(Vtec - Valvo) >= 16) { //Ajustar apenas se a diferença for maior que 0.16v 
          //Loop de até 4 interações de ajuste
          for (int i = 0; i <= 3; i++) {
            ajustaVolt();                   //Faz a chamada do ajuste
            delay(100);                     //Aguarda para fazer a leitura
            Vtec = tensaoTEC();             //Faz leitura para checagem do ajuste
            if (abs(Vtec - Valvo) <= 10) {  //Chegou na voltagem alvo, sai do loop
              break;
            }
          }
        }
      }
    }
  }
}
void TermoAjuste() {
  //Função para ajuste de DT
  //DT > 0 indica necessidade de aumentar a potência
  
  if (HB == true) { //Se o booster foi ativado
    calc = (conT[0] - Talvo) / 3; //Reduzir a intensidade do ajuste em um terço (evita que o ganho de potência pelo booster seja desfeito rapidamente)
    if (calc < 0) {  // Garante ajuste mínimo
      calc = min(calc, -1);
    } else if (calc > 0) {
      calc = max(calc, 1);
    } 
  } else {  //Booster desligado, ajuste padrão
    calc = (conT[0] - Talvo);
  }
  DTc = DTc + calc;  //Faz o ajuste
  //Verifica condições limite
  if (conT[0] < Talvo) {            //Caso de redução do DTc
    if ((conE[1] - Talvo) <= 15) {  //Está numa situação de potência mínima -> A voltagem mínima do regulador manteve 1.5 °C de diferença
      if (HB == true) {             //Se o booster foi ativado
        DTc = max(6, DTc);          //manter pelo menos o valor do booster
      } else {
        DTc = max(0, DTc);  //nulo ou positivo, por ser potência mínima
      }
    }
  } 

  Termostato();  //Chama termostato para fazer o ajuste
}
int tensaoTEC() {
  //Função que realiza a leitura da voltagem da fonte
  adcRead = 0;  //zera a variável
  for (int j = 0; j <= 14; j++) { //15 amostras
    adcRead = adcRead + analogRead(Pvolt);  //Leitura de sinal bruta 0 a 1023 na soma acumulada
    delay(100);                             //aguarda 100 ms entre leituras
  }
  //Cálculos do divisor de voltagem:
  //R1 = 20000 R2 = 10000 Amplitude = 5 v
  //Com 15 amostras, (15/(1024*15)) = (1/1024) em volts
  adcRead = (int)(adcRead / 10.24);  //Faz cálculo em cV
  return adcRead;                    //Retorno
}

void ajustaVolt() {
  //Função que faz o controle do termoServo de rotação contínua
  //(!) Verificar a velocidade de rotação de acordo com os valores da função .write() para permitir o cálculo  
  //Calcula a duração do movimento
  termSec = (int)(18.18 * (abs(Vtec - Valvo)));  // 1000 [ms] / 55 [cV/s] = 18.18 em [ms]
  //Verifica o sentido de ajuste
  if (Vtec > Valvo) {  // Reduzir a voltagem
    termoServo.write(84, 255);  //Inicia rotação
  } else if (Vtec < Valvo) {    //Aumentar a voltagem
    termoServo.write(96, 255);  //Inicia rotação
  }
  delay(termSec);             //Aguarda o tempo do ajuste
  termoServo.write(90, 255);  //Envia o sinal de parada
}
//_________________.Operações matemáticas
int iArr(float value) {
  return (int)(value * 10.0); 
}
int mediaAdd(int *buffer, unsigned char &pbuffer, unsigned char &vbuffer, int valor) {  // Função para atualização dos buffers de média móvel
  if (valor == 0) {
    return;
  }
  if (vbuffer == 5) {              //se o buffer já estiver cheio
    buffer[5] -= buffer[pbuffer];  //remove da soma constante o valor mais antigo
  } else {
    vbuffer++;
  }
  buffer[pbuffer] = valor;      //substitui o índice mais antigo
  buffer[5] += valor;           //adiciona o valor na soma
  pbuffer = (pbuffer + 1) % 5;  //move o índice, garantindo que volte a zero após a posição 4
}
int mediaCal(int *buffer, unsigned char &vbuffer) {
  if (vbuffer > 0) {
    return (buffer[5] / vbuffer);  //Faz a média com a soma constante e o número de índices
  } else {
    return 0;
  }
}

void pulsos() {
  //Contador de pulsos da ventoinha externa
  counter++;
}

//_________________.Operações de interface
void passe() {
  //Momento em que há resposta do usuário, verifica se o ponteiro passou do limite
  if (intSel > intLim) {
    intSel = 0;  // Passou descendo, muda para primeira opção - útil quando se tinha 3 botões
  } else if (intSel < 0) {
    intSel = intLim;  // Passou subindo, muda para última opção
  }
  intLT = millis();  //reinicia contagem do timeOut
  intW = false;
}
void opClear() {
  for (int i = 0; i <= intLim; i++) {
    display.clearLine(i + 2);
  }
}
void displayInit() {
  display.begin();
  visor = 1;                         //Tela ligada
  display.setFont(u8x8_font_5x7_f);  //Fonte
  display.print(F(">"));
}
void displayOff() {
  display.setPowerSave(1);  //Desliga a tela
  visor = 0;
}
void displayOn() {
  display.setPowerSave(0);  //Liga a tela
  if (visor == 0) {         // Troca para primeira tela se estiver desligado
    visor = 1;
  }
}
//_________________.Buzzer
void buzzerIntro() {
  //Buzzer Intro
  tone(buzzerPin, 440, 200);  //A4
  delay(200);
  tone(buzzerPin, 554, 250);  //C#5
  delay(250);
  tone(buzzerPin, 330, 200);  //E4
  delay(200);
  tone(buzzerPin, 660, 500);  //E5
}
void buzzerClic() {
  //Buzzer de interação
  tone(buzzerPin, 1108, 200);  //C#6
}
void buzzerAlert() {
  //Buzzer de erro disparado
  tone(buzzerPin, 554, 150);  //C#5
  delay(150);
  tone(buzzerPin, 466, 250);  //A#4
  delay(250);
  tone(buzzerPin, 554, 150);  //C#5
  delay(150);
  tone(buzzerPin, 466, 500);  //A#4
}
void buzzerLow() { //Tom baixo
  tone(buzzerPin, 494, 100);  //B4
  delay(100);
  tone(buzzerPin, 440, 150);  //A4
}
void buzzerHigh() { //Tom alto
  tone(buzzerPin, 494, 100);  //B4
  delay(100);
  tone(buzzerPin, 660, 250);  //E5
}