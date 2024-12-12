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
	
  Crystal Chamber
  Version 1.1.G 
  (04/12/2024)
  By Voelho
  
  [English commented]
  
  Version is readed as x.y.z (structure, electronics, sketch) - Sketch G is the first that was published (12/2024)
  For detailed information about the algorithm and chamber schematics, check the "Crystal Chamber Documentation" file
  For information about libraries and licensing, refer to the "LICENSE" file
   
*/

//============================ Libraries
//Serial
#include <SPI.h>

//SD card
#include <SD.h>
File myFile;
#pragma execution_character_set("utf-8")

//DHT
#include <DHT.h>

//Servo
#include <VarSpeedServo.h>
VarSpeedServo servo;       //Servo entity for humidity control 
VarSpeedServo elPescador;  //Servo entity for thermostat control

//Display
#include <Arduino.h>
#include <U8x8lib.h>
U8X8_SSD1306_128X64_NONAME_SW_I2C display(/* clock=*/SCL, /* data=*/SDA, /* reset=*/U8X8_PIN_NONE);  // Change accordingly with your display

//Buttons
#include <PinButton.h>


//============================ Definitions
//_________________.Pins
#define PFan 10         //External fan
#define tachoPin 3   //External fan tachometer
#define PTec 4          //TEC
#define PVentilador A0  //Internal fan
#define PServo 9        //Humidity servo
#define BT_mid A2       //Button 1
#define BT_up A1        //Button 2
#define anzolPin A3  //Thermostat servo
#define buzzerPin 7  //Buzzer

//_________________.Buttons
PinButton BtUp(BT_up);
PinButton BtMid(BT_mid);

//_________________.DHT
DHT dht[] = {
  { 5, DHT22 }, //Silica gel DHT
  { 6, DHT22 }, //Chamber DHT
  { 2, DHT22 }, //External DHT
};

int humidity[3];     // Humidity array
int temperature[3];  // Temperature array 

//_________________.Control variables
// Operation
unsigned char OPE;          // State control variable
bool MAN = false;           // Flag that will signal if the system was in maintenance, to call for restart
bool vSD = false;           // Variable that will wait for the presence of the SD card (true = present, false = not present)
unsigned char visor = 0;    // Variable to track the display status (0 - off) (1 - operation) (2 - mode select menu)
char intSel = 0;            // Variable that will operate the interface selection (may be negative)
unsigned char intLim = 0;   // Variable that stores the number of options to be selected
unsigned char Uticks = 0;   // Variable that will delay the humidity control response to wait environmental response
unsigned char SDticks = 0;  // Variable that will delay the writing to the SD card (0 = write)

// Moving Average
int mTin[6] = { 0, 0, 0, 0, 0, 0 };
int mUin[6] = { 0, 0, 0, 0, 0, 0 };
int mTex[6] = { 0, 0, 0, 0, 0, 0 };
// Indexes
unsigned char pTin = 0;
unsigned char pUin = 0;
unsigned char pTex = 0;
// Validators
unsigned char vTin = 0;
unsigned char vUin = 0;
unsigned char vTex = 0;

// Acompanhamento
bool OPT = false;    // Variable to track the status of the TEC
bool Useco = false;  // Variable to register if the humidity dried too much during the control cycle
bool intW = false;   // Variable that will hold the display during interactions, to prevent it from being overwritten/reset every time
bool tampa = true;   // Detection of the lid being open or closed. By default, consider the lid closed - prevents the environment from heating during interruption events

volatile unsigned long counter = 0;  //RPM counter

int rpm = 5000;        // RPM value - Starts high to allow program initialization
char TS[2];            // Temperature status 
int US[3];             // Humidity status
char MoSt = 1;         // Monitor status
int varM;              // Variation multiplier
int varV = 9000;       // Variation of the moving average of temperature in the last hour
int varVU = 9000;      // Variation of the moving average of humidity in the last hour
unsigned char sGraus;  // Temperature servo position

int conT[3];  // Temperature tracking array (0: "Previous temp." to determine the variation) (1: 5-reading moving average) (2: Record from 15 minutes ago)
int conU[4];  // Umidity tracking array (0: Previous humidity - for variation comparison) (1: 6-reading moving average) (2: Absolute value of the last variation) (3: Record from 15 minutes ago)
int conE;     // Tracking external temperature moving average.

//Time intervals
unsigned long SDLT = 0;                       // Time of the last reading
const unsigned long SDInterval = 20000;       // Standard register interval of 20 seconds (Modes with RPM counter have a +1 second delay)
unsigned long TECon = 0;                      // Time when the TEC was turned on
unsigned long ULT = 0;                        // Time when humidity tracking started
unsigned long RPMLT = 0;                      // Time when the fan showed RPM below the limit
unsigned long varLT = 0;                      // Time of the last moving average variation record
const unsigned long varInterval = 900000;     // Record of the moving average variation every 15 minutes (1/4 hour)
unsigned long intLT = 0;                      // Time of the last action on the interface
const unsigned long intInterval = 5500;       // Timeout for each action in the menu interface
unsigned long termLT = 0;                     // Time of the last thermostat reading
const unsigned long termInterval = 180000;    // Interval for thermostat functions (3 minutes)
unsigned long buzzerLT = 0;                   // Time of the last alert
const unsigned long buzzerInterval = 600000;  // 10 minutes to repeat buzzer alerts
unsigned long tempResLT = 0;                  // Time of the last "high temperature reset"

// Parameters

//Humidity servo angles
const unsigned char sClose = 52;  //Closed position
const unsigned char sOpen = 0;    //Maximum opened position

//Temperature range (use integer arithmetic, so 23.5 °C is represented as  235)
const int Tsup = 235;
const int Talvo = 230;
const int Tinf = 220;
int Tb = Talvo;  // TEC power-up buffer when it restarts after dropping below Tinf
// Humidity range (also integer arithmetic)
const int Usup = 757;
const int Ualvo = 754;
const int Uinf = 750;
int dUmax = 10;  // initial maximum humidity variation

int DTc;  // Delta Tc, variable to compensate for DT

//============================ARDUINO
void setup() {

  //pinMode Declarations
  pinMode(PFan, OUTPUT);   
  pinMode(PTec, OUTPUT);   
  pinMode(BT_mid, INPUT);  
  pinMode(BT_up, INPUT);   
  pinMode(PVentilador, OUTPUT);  

  //Variable startup
  OPE = 1;    // Initial state
  TS[0] = 0;  // No temperature variation
  US[0] = 0;  // No humidity variation
  US[2] = 1;  // Humidity servo variation in 1 degree

  // TEC off
  sTEC(0);
  digitalWrite(PVentilador, HIGH);  // Internal fan

  //Screen start
  displayInit();
  display.println(F("CrystalChamber"));
  display.println("G");
  buzzerIntro();

  //Lid verification
  sFan(1);  
  delay(500);
  tacometro();  //Check RPM
  display.println(rpm);
  if (rpm < 100) {   
    tampa = false;   //No lid
    sVentilador(0);  
  }
  sFan(0);

  //DHT initialization
  for (auto &sensor : dht) {
    sensor.begin();
  }

  //SD card initialization
  OperaSD();

  //Servo initialization with a visual clue 
  //	The humidity servo is crucial for the system's operation, so a quick movement is made 
  //	to visually indicate that it is working - useful for detecting loose connections on the pins
  
  servo.attach(PServo);         
  elPescador.attach(anzolPin);  
  dbgServo();
  opServo(9);  //Close the desiccant trapdoor

  delay(2000);   
  TempReg();     //First register
  Termostato();  //Thermostat ajust
}

void loop() {
  //Check button input
  BtUp.update();
  BtMid.update();

  //Check for reading time
  if ((millis() - SDLT) >= SDInterval) {
    TempReg();          
	
    if (vSD == true) {  // Only if SD card is present
      //Delay to avoid collecting too many data points in short time
      if (SDticks < 5) {
        SDticks = SDticks + 1;
      } else {
        SDticks = 0;
      }
    } else {
      SDticks = 1;  // In case SD card is not present
    }
	
    //Operation
    if (OPE == 0) {  // Maintenance condition
      opServo(9);    // Close the trapdoor
	  //shutdown
      sFan(0);
      sTEC(0);
      sVentilador(0);
    } else if (OPE == 4) {  // Disables hygrometric control but keeps the fan on
      sTEC(0);
      sFan(0);
    } else {                   // Allows temperature or humidity control
      if (OPE != 5) {          // If it's 5, overrides temperature control
        if (conT[1] > Tsup) {  // Above the target range
          if (TS[0] == 1) {    // If rising
            Tb = Tb - 1;       //Reduces the restart point
          }
          if (OPT == false) {  //If the TEC is off
            //Turn it on (always with the fans)
            sFan(1);
            sTEC(1);
            sVentilador(1);
          }
        } else if (conT[1] < Tinf) {  // Below target range
          //Turn TEC off, but keep the fans
          sTEC(0);
          if (TS[0] == -1) {    // If falling
            TS[1] = TS[1] + 1;  // Registers the temperature drop event
          }
        } else {                 //Inside the range - starts controlling
          if (TS[0] == 1) {      //If rising...
            if (OPT == false) {  //... and TEC is off
              if ((conT[1] >= Tb) && (conT[1] >= (Tinf + 2))) {  //Check for restart point or minimum value to turn it back on
                sFan(1);
                sTEC(1);
              }
            }
          }
          if ((OPE != 3) && (OPE != 2)) {  //State 3 has humidity control restriction
            OPE = 2;                       //If not, allows for humidity control state
          }
        }
      }
      if ((OPE == 2) || (OPE == 5)) {  // humidity control states
        if (OPT == true && OPE == 5) { // State 5 has no thermal control
          sTEC(0);
          sFan(0);
        }
        if (conU[1] > Usup) {                // Humidity above target range, dry routine. 
          if ((US[0] == 1) && (ULT == 0)) {  // If rising without previous register
            ULT = millis();                  // Logs the moment
          }
          if (US[1] >= sClose) {  // Closed servo (typical initial condition)
            opServo(-1);          // Opens a minimum
          } else {
            ajServo();  //Adjust the variation and operates
          }
        } else if (humidity[1] < Uinf) {                       // Humidity below the upper limit, but instantaneously above the lower limit, evaporation routine
          if (US[0] == -1) {                                   // Humidity decreasing
            opServo(9);                                        // Close the servo until it returns to the target
            Useco = true;                                      // Registers control condition
            US[2] = US[2] / 2;                                 // Reduces variation by half
            if (US[2] < 1) { US[2] = 1; }                      // Ensures a minimum variation of 1
          } else if ((US[1] >= sClose) && (Useco == false)) {  // Initial condition or coming from a reset
            opServo(-1);                                       // Open
          } else {
            ajServo();  //Adjust the variation and operates
          }
        } else {           // Humidity within the range
          if (ULT != 0) {  // If there is a log of higher than target... 
            ULT = 0;       // ...not anymore
          }
          if ((US[0] == 1) && (conU[0] > Ualvo)) {  // Rising, above the target
            if (Useco == true) {                    // Came from excessive drying
              opServo(-1);                          // Open
              Useco = false;                        // Deactivates drying status
            } else {                                // Open slightly
              US[2] = 1;
              opServo(-1);
            }
          } else if ((US[0] == -1) && (conU[0] < Ualvo)) {  //Decreasing, below the target
            US[2] = 1;
            opServo(1);  //closes a minimum
          }
        }
      }
    }

	//RPM counter
    if (OPT == true) {       // Checks only if the TEC is on
      tacometro();           // Checks the RPM
      if (tampa == false) {  // If the indication is that the lid is disconnected
        if (rpm > 100) {     // Checks the RPM
          tampa = true;      // Indicates that the lid is connected
        }
      }
    }

    monitor();  // Check security conditions each SDInterval (to avoid freezing the interface)
  }

  //Interface - Check CrystalChamber Documentation for button mapping
  if (BtUp.isDoubleClick()) {
    displayInit();
  }
  //Interface control
  switch (visor) {
    case 1:  // Screen 1: Watching
      //Alert Override
      if (BtMid.isDoubleClick()) {
        retomaOp(); 
      }
      //Menu
      if (BtMid.isSingleClick()) {
        visor = 2;
        intLT = millis();  // registers an interaction
        buzzerClic();
        DispReset();  //Reset the screen
      }
      //Display off
      if (BtUp.isSingleClick()) {
        displayOff();
        buzzerLow();
      }
      break;
    case 2:  // Screen 2: Menu
      intLim = 4;
      if ((millis() - intLT) <= intInterval) {  // Check interface timeout
        if (intW == false) {                    //System turn, not waiting for input
          opClear(); //Clear option lines
          //Pointer positioning
          display.setCursor(0, 0);
          display.print(F(" Modo "));
          display.setCursor(0, (intSel + 2));
          display.setInverseFont(1);
          display.print(">");
          display.setInverseFont(0);
          //Show options (usage of set cursor + print uses less memory than printString)
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
          intW = true;  //Awaits for user input
        }
        // Buttons
        // Go up
        if (BtUp.isSingleClick()) {
          intSel = intSel - 1;
          passe();
          buzzerClic(); //Buzzer 
        }
        // Confirm
        if (BtMid.isSingleClick()) {
          buzzerHigh(); //Buzzer
          switch (intSel) { //Evaluates the input
            case 0:
              OPE = 1;
              opClear();
              break;
            case 1:
              OPE = 0;
              opClear();
              break;
            case 2:
              OPE = 3;
              opClear();
              break;
            case 3:
              OPE = 4;
              opClear();
              break;
            case 4:
              OPE = 5;
              opClear();
              break;
          }
          passe();
          visor = 1;  // Change to Screen 1
        }
        //Remove/insert SD card
        if (BtMid.isDoubleClick()) {
          OperaSD(); 
        }
      } else {
        visor = 1;  //Menu timeout, return to Screen 1
        passe();
      }
      break;
    default:  // Screen 0: Power saving mode of the display
      //Display turn on
      if (BtUp.isSingleClick()) {
        displayOn();
        DispReset();
        display.println(F("-Load-"));
        buzzerHigh();
      }
      monitor();  //If the screen is off, check for security conditions every loop
      break;
  }
}

//============================ Operational Functions
//_________________.Alert and monitoring
void monitor() {
// Controls the tolerance period after maintenance
  if (MoSt == 1) {                         //Post-maintenance tolerance period
    if ((millis() - TECon) >= 240000UL) {  //Waits for 4 minutes (the retomaOp routine resets the TECon timer, even with the TEC off)
      MoSt = 0;                            //Ends the tolerance period
    }
  }

  if (OPE == 0) {
    display.setCursor(0, 1);
    display.print(F("MAN")); //Show maintenance status
    MAN = true;
  } else {

    if (MAN == true) {  // If came from a maintenance state, resets
      retomaOp();
    }
    MAN = false;  //Not on maintenance anymore


    // Security conditions
	// TEC is on and external Fan is working well
    if (OPT == true) {
      if (RPMLT == 0) {    //If fan is always working ok...
        if (rpm < 2600) {  // Check for failure in rotation (my fan usual RPM reading is between 2880 and 2910 RPM, so anything below 2600 is probably a problem)
          RPMLT = millis();
        }
      } else { //Sometime the fan failed...
        if ((millis() - RPMLT) > 60000UL) {  // ...more than 60 seconds ago?
          if (rpm > 2800) {                  // Check for a false-positive, if it's back in the range, no problem 
            RPMLT = 0;
          } else {  // if not, abort the operation
            MoSt = 5;
            OPE = 0; //Maintenance state
          }
        }
      }
    }
    
	//Operational Conditions

    // Low cooling power
    // [A] Check if it’s a rapid heating event (like at dawn) - Relative to the instantaneous reading.

    if (tempResLT == 0) {                                     // Default, no reset signal
      if ((temperature[1] >= (Talvo + 3)) && (TS[0] == 1)) {  // Instantaneous temperature 0.3 degrees above the range and rising
        //Resets the moving average to force a thermostat adjust
        conE = 0;
        memset(mTin, 0, sizeof(mTin));
        pTin = 0;
        vTin = 0;
        DTc = 0;
        Termostato();
        //Records the attempt
        tempResLT = millis();
      }
    } else {                                               //There is a previous attempt...
      if (temperature[1] < (Talvo + 3)) {                  //...it resolved, temperature is no longer higher
        tempResLT = 0;                                     //resets the record
      } else {                                             //...it did not
        if (((millis() - tempResLT) / 1800000UL) > 1.0) {  //and it was more than half an hour ago - enough time for equilibrium given termInterval and the moving average size
          MoSt = 2; //Sends a "low power" alert
        }
      }
    }
 //#STEP
    // [B] Check relative to the average
    if (((millis() - TECon) / 3600000UL) > 4.0) {  //TEC is on for more than 4 hours
      if (conT[1] > Tsup) {                        //Average is above the range
        MoSt = 2;                                  //Sends the alert
      }
    }
    
	// Cooling fail (example: in case the cold fan fails)
    if (MoSt != 1) {                //Bypass verification during the tolerance period to ensure the start and maintenance return conditions
      if (conT[1] > (Tsup + 20)) {  //Temp. 2°C above upper limit...
        if (TS[0] == 1) {           //... and rising
          OPE = 0;                  //Interrupt operation for safety, could be an issue with a component
          MoSt = 6;                 //Flag
        }
      }
    }
    // Low drying capacity 
    if ((OPE == 2) && (ULT != 0)) {
      if (((millis() - ULT) / 3600000UL) > 2.0) {  //2 hours after the register
        if ((conU[1] > Usup)) {                    //still above the range
          MoSt = 3;                                //Flag
        }
      }
    }
    // High cooling power 
    if (TS[1] > 4) { //More than 4 decreases below the minimum 
      if (MoSt == 7) {  // If it is related to a fail on the DHT sensor 
        OPE = 0;        // Stops the operation
      } else { //If not sensor related
        MoSt = 4;  // Flags too much cooling power
      }
    }
  }

  if (MoSt != 0) {  //If there is an alert
    display.setCursor(4, 1); 
    if (MoSt == 1) {
      display.print(F("start")); //Message for tolerance period
    } else {
      //Buzzer alerts
      if ((millis() - buzzerLT) >= buzzerInterval) {  //Interval
        buzzerAlert();                                //Rings
        buzzerLT = millis();                          //Update last time
      }
      if (visor == 0) {  //Ensures that screen is on
        displayOn();
      }
    }
    switch (MoSt) {  // Print the error message
      case 2: display.print(F("BPot")); break;
      case 3: display.print(F("BSec")); break;
      case 4: display.print(F("APot")); break;
      case 5: display.print(F("FAN")); break;
      case 6: display.print(F("TIn")); break;
      case 7: display.print(F("DHT")); break;
    }
  }
}
//_________________.Reset function
void retomaOp() {
  TS[0] = -1;  // suposes falling to avoid loop with MoSt = 6
  TS[1] = 0;
  US[0] = 1;
  dbgServo(); //Servo visual cue
  opServo(9);
  conT[0] = 0;
  conU[0] = 0;
  conU[2] = 0;
  Tb = Talvo;
  ULT = 0;
  RPMLT = 0;
  rpm = 5000;
  MoSt = 1;  
  conE = 0;
  display.setCursor(0, 0);
  display.println(F("-ReLoad-"));
  varV = 9000;       
  varVU = 9000;      
  varLT = millis();  
  TECon = millis();  
  Useco = false;
  tampa = false;
  sVentilador(1); 
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
  Termostato(); 
}

//_________________.DHT + SD
void leTMP() {
  //Chamada para atualizar a média móvel de temperatura ou produzir média parcial
  if (temperature[1] == 0) {  //Verify a bad connection with the sensor
    conT[1] = conT[0];        //keeps last reading
    displayOn(); //Alert
    MoSt = 7;
  } else {
    //Updates the moving average
    mediaAdd(mTin, pTin, vTin, temperature[1]);
    conT[1] = mediaCal(mTin, vTin);
    if (MoSt == 7) {  //In case the sensor is back, remove the alert
      MoSt = 0;
    }
  }

  //Variation verification
  if (conT[1] > conT[0]) { //Rising
    TS[0] = 1;
  } else if (conT[1] < conT[0]) { //Falling
    TS[0] = -1;
  } else { //Constant
    TS[0] = 0;
  }
  conT[0] = conT[1];  //Updates reference for next reading

  //Variation history
  //First log
  if (varLT == 0) {
    conT[2] = conT[0];
    conU[3] = conU[0];
    varLT = millis();
  }

  //Check interval
  if ((millis() - varLT) >= varInterval) {
    //Updates temperature history 
    if (conT[0] > conT[2]) {
      varM = 7000;
      //Temperature increase, check for DTC for high temperature and adjust
      if (DTc < 0) { TermoAjuste(); }
    } else if (conT[0] < conT[2]) {
      varM = 2000;
      //Temperature decrease, check for DTC for cool temperature and adjust
      if (DTc > 0) { TermoAjuste(); }
    } else {
      varM = 1000;
      //Temperature stable, ajusts DTC to compensate for the inaccuracy of the thermostat curve 
      TermoAjuste();
    }
    conT[2] = conT[0];      //Updates history from <varInterval> minutes ago
    varV = int(varV / 10);  //Moves the pointer 1 position foward
    varV = varV + varM;     //Add the new status

    //Updates humidity history 
    if (conU[0] > conU[3]) {
      varM = 7000;
    } else if (conU[0] < conU[3]) {
      varM = 2000;
    } else {
      varM = 1000;
    }
    conU[3] = conU[0];      
    varVU = int(varVU / 10); 
    varVU = varVU + varM;    

    varLT = millis();  //Update the last register time
  }

  //Thermostat functions
  if ((millis() - termLT) >= termInterval) {  //Check for interval

	// Updates moving average
    mediaAdd(mTex, pTex, vTex, temperature[2]);
    conE = mediaCal(mTex, vTex);
    Termostato();       //Control 
    termLT = millis();  //updates last time
  }
}

void leUR() {
  // Updates moving average
  mediaAdd(mUin, pUin, vUin, humidity[1]);
  conU[1] = mediaCal(mUin, vUin);

  //Variation verification
  if (Uticks == 3) {                   //Number of cicles for humidity evaluation
    conU[2] = abs(conU[1] - conU[0]);  //Registers the absolute variation value

    if (conU[1] > conU[0]) {  //Rising
      US[0] = 1;
    } else if (conU[1] < conU[0]) {  //Falling
      US[0] = -1;
    } else {  //Stable
      US[0] = 0;
    }
    conU[0] = conU[1];  
    Uticks = 1;         //restart counter
  } else {
    Uticks = Uticks + 1; //increase counter
  }
}
//________________.Readings and writings
void TempReg() {
  if (OPE != 0) {  //bypass in maintenance state
  
  	// (!) ATENTION - Change the coefficients accordingly to your DHT sensor's calibration curves	
	
    //the readings return float values, function iArr() converts it to a rounded integer (ex. 23.56 becomes 236)
    temperature[0] = iArr(((dht[0].readTemperature()) * 0.9927) - 0.2018);  
    temperature[1] = iArr(((dht[1].readTemperature()) * 0.9669) + 0.6177);  
    temperature[2] = iArr(((dht[2].readTemperature()) * 0.9621) + 1.0308);  

    //Selection of the correct humidity calibration curve (DHT sensors behave differently according to the temperature)
    if (conT[1] < 260) {
      //for temperatures below 26°C
      humidity[0] = iArr(((dht[0].readHumidity()) * 0.9675) - 0.9138);  
      humidity[1] = iArr(((dht[1].readHumidity()) * 0.9037) + 1.0347);  
      humidity[2] = iArr(((dht[2].readHumidity()) * 0.9469) - 1.8105);  
    } else {
      humidity[0] = iArr(((dht[0].readHumidity()) * 1.0858) - 10.828);  
      humidity[1] = iArr(((dht[1].readHumidity()) * 1.0105) - 8.203);   
      humidity[2] = iArr(((dht[2].readHumidity()) * 1.0581) - 11.362);  
    }
    leUR();   // Humidity control
    leTMP();  // Temperature control
  }
  if (SDticks == 0) { //Check SD card
    myFile.print(millis()); //Print the timestamp
    quebra();
  }

  if (visor == 1) { //Check screen for data display
    DispReset();
  }
  for (int i = 0; i < 3; i++) {
    if (SDticks == 0) { //Check if time for SD writing
      myFile.print(temperature[i] / 10.0, 1); 
      quebra();
      myFile.print(humidity[i] / 10.0, 1);
      quebra();
    }
    if (visor == 1) {
      //Readings
      display.print(i + 1);
      display.print(F(" T:"));
      display.print(temperature[i] / 10.0, 1);
      display.print(F(" U:"));
      display.println(humidity[i] / 10.0, 1);
    }
  }
  if (visor == 1) {
    //Moving averages
    display.setCursor(11, 0);
    display.print((conT[1] / 10.0), 1);
    display.setCursor(11, 1);
    display.print((conU[1] / 10.0), 1);
	
	//RPM and TEC
    display.setCursor(0, 5);
    display.print(rpm);
    display.print(F("RPM"));
    display.print(OPT);

    //Servo
    display.setCursor(9, 5);
    display.print(US[1]);  
    display.print(":");
    display.print(US[2]);  

	//Variation history
	display.setCursor(0, 6);
    display.print("T");
    display.print(varV);
    display.print("|");
    display.print(TS[1], DEC);
    display.print("| ");
    
	//Thermostat
    display.print(sGraus);
    display.print(" ");
    display.println(DTc);

    display.print("U");
    display.print(varVU);
    display.print("|");
    //Time remaining for next variation history reading
    display.print((int)((varLT + varInterval - millis()) / 60000UL));

    //External temperature average
    display.setCursor(10, 7);
    display.print("E");
    display.print(conE / 10.0, 1);
  }
  if (SDticks == 0) { //SD information
    myFile.print(OPT);  // TEC
    quebra();
    myFile.print(US[1]);  //Servo angle
    quebra();
    myFile.print(sGraus);  //Thermostat angle
    myFile.println("");  //Line break
    myFile.flush();      // Registers
  }
  SDLT = millis();  //update last time
}
void quebra() {   //SD data delimiter function, to save memory
  myFile.print(F(";"));
}
//_________________.Switch functions
void sFan(int x) { //Fans
  //Control x (0: turn off, 1: turn on)
  switch (x) {
    case 0:
      digitalWrite(PFan, LOW);
      break;
    case 1:
      digitalWrite(PFan, HIGH);
      break;
  }
}
void sTEC(int x) { //TEC
  switch (x) {
    case 0:
      digitalWrite(PTec, LOW);
      OPT = false;
      break;
    case 1:
      digitalWrite(PTec, HIGH);
      TECon = millis();  //Registers the time TEC was turned on
      OPT = true;
      break;
  }
}

void sVentilador(int x) {  // Chamber fan
  switch (x) {
    case 0:
      digitalWrite(PVentilador, LOW);
      break;
    case 1:
      // States 4 and 5 require open lid
      if (OPE == 4 || OPE == 5) {
        if (tampa == false) { 
          digitalWrite(PVentilador, HIGH);
        }
      } else {
        digitalWrite(PVentilador, HIGH);
      }
      break;
  }
}
//_________________. Tachometer
void tacometro() {
  counter = 0;                                                       //Reset RPM counter
  attachInterrupt(digitalPinToInterrupt(tachoPin), pulsos, RISING);  //Read
  delay(1000);                                                       //Wait
  rpm = counter * 30;                                                //Convert to RPM
  detachInterrupt(digitalPinToInterrupt(tachoPin));                  //Detach
}
//_________________.Display
void DispReset() {
  display.clearDisplay();  //clear buffer
  if (vSD == false) {
    display.setCursor(9, 0);
    display.print(F("SD"));  //If SD is missing
  }                          
  display.setCursor(0, 0);
  switch (OPE) { //State titles
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

//_________________.SD card
void OperaSD() {
  display.setCursor(0, 1);
  display.print(F("SD... "));
  if (vSD == true) {  //If it is connected
    myFile.close(); //Close
    vSD = false;
    delay(250);
    display.println(F("Off"));
    buzzerLow();
  } else if (vSD == false) {  // If it isn't connected
    delay(250);             
    if (SD.begin(8)) { //Start
      myFile = SD.open("datalog.txt", FILE_WRITE); //Creates datalog file
      if (myFile) {
        display.println(F("Ok"));
        buzzerClic();
        vSD = true;   
        SDticks = 0;  
      }
    } else {
      display.println(F("Missing"));
      buzzerAlert();
    }
  }
}

//_________________.Servo
void opServo(int sentido) {
  // Humidity servo positioning function
  int servoPos;    
  int solicitado;  

  if (sentido == 9) {  //Close call
    servoPos = sClose;
  } else { //Position call
    solicitado = US[1] + (sentido * US[2]);

	//Check for limits
    if (solicitado <= sOpen) {  
      servoPos = sOpen;
    } else if (solicitado >= sClose) {  
      servoPos = sClose;
    } else {
      servoPos = solicitado;
    }
  }
  servo.write(servoPos, 80);  //Moves
  US[1] = servoPos;           //Updates position register
  delay(500);                 //wait a little
}
void ajServo() {
  // Function to adjust the positioning accordingly to humidity variation 
  
  // Damping the variation based on humidity target
  if (abs(conU[1] - Ualvo) > 50) {
    dUmax = 10;
  } else if (abs(conU[1] - Ualvo) > 20) {
    dUmax = 5;
  } else {
    dUmax = 2;  //sensor minimum error range
  }
  if (US[0] != 0) {                 // Adjust the variation only when a change occurs
    if (conU[2] < dUmax) {          // Variation below the maximum allowed
      US[2] = int(US[2] + 1);       // Increase the step
      US[2] = min(US[2], 5);        // Limit the maximum variation to 5 to prevent instability
    } else if (conU[2] >= dUmax) {  // Variation above or equal to the maximum allowed
      US[2] = int(US[2] / 2);       // Reduce the step
      US[2] = max(US[2], 1);        // Limit the minimum variation to 1 to avoid stagnation
    }
  }
  //Adjust instruction according to condition and variation 
  if (conU[1] > Usup) {        //Above target...
    if (US[0] >= 0) {          //... rising or stable 
      opServo(-1);             // Open
    } else if (US[0] == -1) {  //... falling ...
      if (conU[2] >= dUmax) {  // 			...More than the limit
        opServo(1);            // Close
      } else {                 // 			...Less than the limit
        opServo(-1);           // Open
      }
    }
  }
}
void dbgServo() {
  //Visual cue for servo correctly working 
  servo.write(25, 80, true);  
  delay(2000);
  servo.write(50, 80, true);  
}
//_________________.Thermostat
void Termostato() {
  //Check for states with temperature control
  if ((OPE >= 1) && (OPE <= 3)) {
    float DT = 0.0;     // Local float variable for DT
    float DTfun = 0.0;  // Local float variable for function calculation

    if (temperature[2] != 0) {//Bypass in case the DHT is faulty
      //calculates the deviation from target
      if (conE == 0) { //If there is no average yet
        DT = (temperature[2] - Talvo) / 10.0;  
      } else {
        DT = (conE - Talvo + DTc) / 10.0;  
      }

      DTfun = (-19.1 * DT * DT) + (310.3 * DT) + (-983.35);  //DT to Servo position function
      
	  // Domain restriction
      if (DTfun < 0.0) {
        DTfun = 0.0;
      } else if (DTfun >= 180.0) {
        DTfun = 180.0;
      }
      sGraus = (unsigned char)DTfun;  // Truncate as byte variable
      elPescador.write(sGraus, 70); //Moves
    }
  }
}
void TermoAjuste() { 
  //Function to limit the DT adjust, avoiding a infinite sum when the Thermostat servo is already at one limit during stable periods
  if ((conT[2] > Talvo) && (sGraus < 180)) {  //To increase DT, before the maximum power
    ajDTC();
  } else if ((conT[2] < Talvo) && (sGraus > 0)) {  //To decrease DT, before the minimum power
    ajDTC();
  }
}
void ajDTC() {
  //Call for DTC adjust, to save memory
  DTc = DTc + (conT[2] - Talvo);
}
//_________________.Math functions
int iArr(float value) {
  return (int)(value * 10.0); 
}
int mediaAdd(int *buffer, unsigned char &pbuffer, unsigned char &vbuffer, int valor) {  //Moving average buffers
  if (valor == 0) {
    return;
  }
  if (vbuffer == 5) {              //When buffer is full
    buffer[5] -= buffer[pbuffer];  //remove the oldest value from the constant sum
  } else {
    vbuffer++;
  }
  buffer[pbuffer] = valor;      //overwrites the oldest index
  buffer[5] += valor;           //add value in the sum
  pbuffer = (pbuffer + 1) % 5;  //moves the circular index
}
int mediaCal(int *buffer, unsigned char &vbuffer) {
  if (vbuffer > 0) {
    return (buffer[5] / vbuffer);  //Calculates the moving average (with partials)
  } else {
    return 0;
  }
}

void pulsos() {
  //Counter of tachometer pulses
  counter++;
}

//_________________.Interface functions
void passe() {
  //After user input, check if pointer went around
  if (intSel > intLim) {
    intSel = 0;  //Passed the last, go to the first - Useful when there are 3 buttons
  } else if (intSel < 0) {
    intSel = intLim;  //Passed the first, go to the last
  }
  intLT = millis();  //restart timeout countdown
  intW = false;
}
void opClear() {
  for (int i = 0; i <= intLim; i++) {
    display.clearLine(i + 2);
  }
}
void displayInit() {
  display.begin();
  visor = 1;                         //Screen is on
  display.setFont(u8x8_font_5x7_f);  //Font
  display.print(F(">"));
}
void displayOff() {
  display.setPowerSave(1);  //Turn screen off
  visor = 0;
}
void displayOn() {
  display.setPowerSave(0);  //Turn screen on
  if (visor == 0) {        
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
  //Buzzer interaction
  tone(buzzerPin, 1108, 200);  //C#6
}
void buzzerAlert() {
  //Buzzer alert
  tone(buzzerPin, 554, 150);  //C#5
  delay(150);
  tone(buzzerPin, 466, 250);  //A#4
  delay(250);
  tone(buzzerPin, 554, 150);  //C#5
  delay(150);
  tone(buzzerPin, 466, 500);  //A#4
}
void buzzerLow() { //Low tone
  tone(buzzerPin, 494, 100);  //B4
  delay(100);
  tone(buzzerPin, 440, 150);  //A4
}
void buzzerHigh() { //High tone
  tone(buzzerPin, 494, 100);  //B4
  delay(100);
  tone(buzzerPin, 660, 250);  //E5
}
