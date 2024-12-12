/*
    Crystal Chamber - A sketch for controlling temperature and relative humidity in crystal growing by evaporation method
    Copyright (C) 2024  Voelho

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/*
	
  Crystal Chamber - Thermostat calibration - Version E 
  A routine for calibration of thermostat angle and supply voltage 
  
  13 october 2024
  By Voelho
  
  [English commented]
  
  This sketch is part of the Crystal Chamber Project v1.1.G
  For detailed information about the algorithm and chamber schematics, check the "Crystal Chamber Documentation" file
  For information about libraries and licensing, refer to the "LICENSE" file
   
*/

//============================Libraries

//Servo
#include <VarSpeedServo.h>
VarSpeedServo servoVar; 

//Display
#include <Arduino.h>
#include <U8x8lib.h>
U8X8_SSD1306_128X64_NONAME_SW_I2C display(/* clock=*/SCL, /* data=*/SDA, /* reset=*/U8X8_PIN_NONE);  // Change accordingly with your display

//Buttons
#include <PinButton.h>

//============================Definitions
//_________________.Pins
#define BT_mid A2       //Button 1
#define BT_up A1        //Button 2
#define PTec 4       //TEC
#define calPin A3     //Thermostat servo

//_________________.Buttons
PinButton BtUp(BT_up);
PinButton BtMid(BT_mid);

//_________________.Pins
int posicao = 0; //Start at zero

void setup() {
  //pinMode Declarations
  pinMode(BT_mid, INPUT); 
  pinMode(BT_up, INPUT);  
  pinMode(PTec, OUTPUT);  
  digitalWrite(PTec, HIGH); 

  //Screen start
  displayInit();
  display.println("Calibration");

  //Start servo
  servoVar.attach(calPin);
  servoVar.stop();
  servoVar.write(posicao, 100, true);
  delay(2000);
  DispReset();
  
}

void loop() {
  //Check buttons
  BtUp.update();
  BtMid.update();
  
  
  /*Interface
  Single Up: add 10 degrees
  Double Up: resets to zero
  Single Mid: sends the angle to move the servo 
  
  Double Mid: call the automatic mode (see autocal function)
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
//Moves the servo through most relevant angles for calibration with delay to read voltage
//Sequence: 0 60 120 180 20 80 140 40 100 160
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
  for (int i = 1; i <= 3; i++) { //3 increments
      //move and wait
      servoVar.write(posicao, 70, true);
      delay(4000); //2 seconds to read and take note
      posicao = posicao + 60;
      if(posicao > 180){ //ceiling
        posicao = 180;
      }
  }
}

//_________________.Display
void DispReset() {
  display.clearDisplay();  //clear buffer
  display.setCursor(0, 0);
}

void displayInit() {
  display.begin();
  display.setFont(u8x8_font_5x7_f);  //Font
  display.print(F(">"));
}
