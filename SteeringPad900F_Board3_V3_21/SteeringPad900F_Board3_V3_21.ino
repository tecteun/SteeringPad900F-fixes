/*
 * spoofing the USB VID/PID to a real logitech steering wheel, fixes detection in, for example, snowrunner.
 * lsusb Bus 001 Device 002: ID 046d:c24f Logitech, Inc. G29 Driving Force Racing Wheel [PS3] 
 * lsusb Bus 001 Device 002: ID VID:PID Vendor-name, Product-name
 * 
 * this needs to be done by updating boards.txt and flashing with the "Arduino Micro, modded" board from the boards list
 * 
 * (because this is not merged in mainline yet https://github.com/arduino/ArduinoCore-avr/pull/387)
 * 
 * howto: updating boards.txt for VID/PID descriptor
 * add below board to boards.txt
 * on windows this is found at
 * C:\Users\%USERNAME%\AppData\Local\Arduino15\packages\arduino\hardware\avr\boards.txt
 * 

##############################################################

mmicro.name=Arduino Micro, modded

mmicro.vid.0=0x2341
mmicro.pid.0=0x0037
mmicro.vid.1=0x2341
mmicro.pid.1=0x8037
mmicro.vid.2=0x2A03
mmicro.pid.2=0x0037
mmicro.vid.3=0x2A03
mmicro.pid.3=0x8037
mmicro.vid.4=0x2341
mmicro.pid.4=0x0237
mmicro.vid.5=0x2341
mmicro.pid.5=0x8237
mmicro.upload_port.0.vid=0x2341
mmicro.upload_port.0.pid=0x0037
mmicro.upload_port.1.vid=0x2341
mmicro.upload_port.1.pid=0x8037
mmicro.upload_port.2.vid=0x2A03
mmicro.upload_port.2.pid=0x0037
mmicro.upload_port.3.vid=0x2A03
mmicro.upload_port.3.pid=0x8037
mmicro.upload_port.4.vid=0x2341
mmicro.upload_port.4.pid=0x0237
mmicro.upload_port.5.vid=0x2341
mmicro.upload_port.5.pid=0x8237
mmicro.upload_port.6.board=micro

mmicro.upload.tool=avrdude
mmicro.upload.tool.default=avrdude
mmicro.upload.tool.network=arduino_ota
mmicro.upload.protocol=avr109
mmicro.upload.maximum_size=28672
mmicro.upload.maximum_data_size=2560
mmicro.upload.speed=57600
mmicro.upload.disable_flushing=true
mmicro.upload.use_1200bps_touch=true
mmicro.upload.wait_for_upload_port=true

mmicro.bootloader.tool=avrdude
mmicro.bootloader.tool.default=avrdude
mmicro.bootloader.low_fuses=0xff
mmicro.bootloader.high_fuses=0xd8
mmicro.bootloader.extended_fuses=0xcb
mmicro.bootloader.file=caterina/Caterina-Micro.hex
mmicro.bootloader.unlock_bits=0x3F
mmicro.bootloader.lock_bits=0x2F

mmicro.build.mcu=atmega32u4
mmicro.build.f_cpu=16000000L
mmicro.build.vid=0x046d
mmicro.build.pid=0x0000
mmicro.build.usb_product="Steering Pad 900-F"
mmicro.build.usb_manufacturer="Logitech, Inc"
mmicro.build.board=AVR_MICRO
mmicro.build.core=arduino
mmicro.build.variant=micro
mmicro.build.extra_flags={build.usb_flags}


##############################################################
 
 * 
 * 
 */

/*
  LICENSE

  Project Name: Steering PAD 900-F v3.2
  Author: Rui Caldas (HomeGameCoder)
  License: GNU General Public License v3.0

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <https://www.gnu.org/licenses/>.


  LEGAL DISCLAIMER

  The code provided herein is for informational and educational purposes only and is distributed
  "as is" without warranties of any kind, either express or implied, including but not limited to
  warranties of merchantability or fitness for a particular purpose.
  The author disclaims any liability for damages, losses, or other consequences arising from the
  use or misuse of this code.
  By using this code, you acknowledge and agree to these terms.
  Use entirely at your own risk.
*/

//  ███████ ███████  █████  ████████ ██    ██ ██████  ███████ ███████ 
//  ██      ██      ██   ██    ██    ██    ██ ██   ██ ██      ██      
//  █████   █████   ███████    ██    ██    ██ ██████  █████   ███████ 
//  ██      ██      ██   ██    ██    ██    ██ ██   ██ ██           ██ 
//  ██      ███████ ██   ██    ██     ██████  ██   ██ ███████ ███████ 
//                 

// Set this to true the first time you upload the script to the ProMicro.
#define isFirstTimeUploading false 
// This will ensure the data in the EEPROM is what the script is
// expecting when booting.
// It needs to be true once at the FIRST upload.
// If you leave it true, all calibrations are reseted to default
// every time you plug in the controller.
// After uploading the script once to the arduino with this true, set it to false and re-upload.
// It is not necessary to do this again!


//  ██ ███    ██  ██████ ██      ██    ██ ███████ ██  ██████  ███    ██ ███████ 
//  ██ ████   ██ ██      ██      ██    ██ ██      ██ ██    ██ ████   ██ ██      
//  ██ ██ ██  ██ ██      ██      ██    ██ ███████ ██ ██    ██ ██ ██  ██ ███████ 
//  ██ ██  ██ ██ ██      ██      ██    ██      ██ ██ ██    ██ ██  ██ ██      ██ 
//  ██ ██   ████  ██████ ███████  ██████  ███████ ██  ██████  ██   ████ ███████ 

// ADC
#include <DigitalWriteFast.h>
#include "ADS1X15.h"
ADS1115 ADS(0x48);
/* 27726
// DISPLAY
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiAvrI2c oled;
*/

 
// DISPLAY
//#include <Wire.h> // TwoWire, Wire
#include <AceWire.h>
using ace_wire::TwoWireInterface;
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAceWire.h"
#define I2C_ADDRESS 0x3C

using WireInterface = TwoWireInterface<TwoWire>;
WireInterface wireInterface(Wire);

SSD1306AsciiAceWire<WireInterface > oled(wireInterface);

 

// JOYSTICK
#include <Joystick.h>

Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,
  15,     // Buttons (Max 32)
  0,      // Hats (Max 2)
  true,   // X Axis
  true,   // Y Axis
  true,   // Z Axis
  false,  // X Rotation
  false,  // Y Rotation
  false,  // Z Rotation
  false,  // Rudder
  false,  // Throttle
  false,  // Accelerator
  false,  // Break
  false   // Steering (no force feedback here!)
);

//   ██████  ██████  ███    ██ ███████ ██  ██████  ██    ██ ██████   █████  ██████  ██      ███████ 
//  ██      ██    ██ ████   ██ ██      ██ ██       ██    ██ ██   ██ ██   ██ ██   ██ ██      ██      
//  ██      ██    ██ ██ ██  ██ █████   ██ ██   ███ ██    ██ ██████  ███████ ██████  ██      █████   
//  ██      ██    ██ ██  ██ ██ ██      ██ ██    ██ ██    ██ ██   ██ ██   ██ ██   ██ ██      ██      
//   ██████  ██████  ██   ████ ██      ██  ██████   ██████  ██   ██ ██   ██ ██████  ███████ ███████ 
//                                                                                                  
//                                                                                                  
//  ██    ██  █████  ██████  ██  █████  ██████  ██      ███████ ███████                             
//  ██    ██ ██   ██ ██   ██ ██ ██   ██ ██   ██ ██      ██      ██                                  
//  ██    ██ ███████ ██████  ██ ███████ ██████  ██      █████   ███████                             
//   ██  ██  ██   ██ ██   ██ ██ ██   ██ ██   ██ ██      ██           ██                             
//    ████   ██   ██ ██   ██ ██ ██   ██ ██████  ███████ ███████ ███████                             
//

// JOYSTICK
// Steering Wheel positions for linearity correction
int16_t steeringSensorMapLUT[11] = {
  8723,   // 0º   LEFT (+ 20º)
  9882,   // 90º
  11011,  // 180º
  12092,  // 270º
  13122,  // 360º
  14077,  // 450º CENTER
  15151,  // 540º
  16459,  // 630º
  17820,  // 720º
  19118,  // 810º
  19654   // 900º RIGHT (- 20º)
};

// STEERING LUT
const int16_t outputLUT[11] = {
  -32768, -26214, -19660, -13106, -6552,
  0, 6552, 13106, 19660, 26214, 32767
};

// Define easing modes
enum EasingMode {
  EASEIN,   // Slow start: quadratic curve (norm^2)
  LINEAR,   // Linear response: no modification
  EASEOUT   // Fast start: square root curve (sqrt(norm))
};

// Break Pedal settings (Default)
EasingMode brakeEase = EASEIN;
int16_t brakeSensorMin = 11107;
int16_t brakeSensorMax = 14793;


// Accelerator Pedal settings (Default)
EasingMode acceleratorEase = LINEAR;
int16_t acceleratorSensorMin = 10557;
int16_t acceleratorSensorMax = 14718;


// FORCE FEEDBACK
int forceGain = 100;
bool forceActive = true;


//   ██████  ██████  ███    ██ ███████ ████████  █████  ███    ██ ████████ 
//  ██      ██    ██ ████   ██ ██         ██    ██   ██ ████   ██    ██    
//  ██      ██    ██ ██ ██  ██ ███████    ██    ███████ ██ ██  ██    ██    
//  ██      ██    ██ ██  ██ ██      ██    ██    ██   ██ ██  ██ ██    ██    
//   ██████  ██████  ██   ████ ███████    ██    ██   ██ ██   ████    ██    
//                                                                         
//                                                                         
//  ██    ██  █████  ██████  ██  █████  ██████  ██      ███████ ███████    
//  ██    ██ ██   ██ ██   ██ ██ ██   ██ ██   ██ ██      ██      ██         
//  ██    ██ ███████ ██████  ██ ███████ ██████  ██      █████   ███████    
//   ██  ██  ██   ██ ██   ██ ██ ██   ██ ██   ██ ██      ██           ██    
//    ████   ██   ██ ██   ██ ██ ██   ██ ██████  ███████ ███████ ███████    
//


// BUTTONS
const int errorMargin = 7; // Adjustable error margin for reading the buttons

const int thresholds[7] = 
{
  220, // button 3
  355, // button 5
  477, // button 6
  161, // buttons 3 & 5
  180, // buttons 3 & 6
  258, // buttons 5 & 6
  140  // buttons 3 & 5 & 6
}; 

const unsigned long LONG_PRESS_THRESHOLD = 1000;  // 1 second

// DISPLAY
const int DISPLAY_WIDTH = 16;

// EEPROM

// JOYSTICK
const int16_t joystickMin = -32767;
const int16_t joystickMax = 32767;

// FORCE FEEDBACK
  Gains gains[1];
  EffectParams params[1];
  int16_t lastPosition = 0;

  #define motorPinDirection 8
  #define motorPinPWM 9
  #define motorPinEnable 10

// MENU
byte menuLength = 7;



//  ██████  ██████   ██████   ██████ ███████ ███████ ███████            
//  ██   ██ ██   ██ ██    ██ ██      ██      ██      ██                 
//  ██████  ██████  ██    ██ ██      █████   ███████ ███████            
//  ██      ██   ██ ██    ██ ██      ██           ██      ██            
//  ██      ██   ██  ██████   ██████ ███████ ███████ ███████            
//                                                                      
//                                                                      
//  ██    ██  █████  ██████  ██  █████  ██████  ██      ███████ ███████ 
//  ██    ██ ██   ██ ██   ██ ██ ██   ██ ██   ██ ██      ██      ██      
//  ██    ██ ███████ ██████  ██ ███████ ██████  ██      █████   ███████ 
//   ██  ██  ██   ██ ██   ██ ██ ██   ██ ██   ██ ██      ██           ██ 
//    ████   ██   ██ ██   ██ ██ ██   ██ ██████  ███████ ███████ ███████ 
//   

// BUTTONS
int sensorValues[4];
bool buttonsMux[15] = {false};

bool button[16];
bool buttonMenu;

bool oldButton[16];
bool oldButtonMenu;

bool button3WasPressed;
bool longPressTriggered;
long button3StartTime = 0;

bool button13AsFunction;
bool button13OnHold;

// FORCE FEEDBACK
int32_t forces[1];

// MENU
byte operationMode = 0;
byte oldOperationMode = 0;
byte menuLevel = 0;
byte oldMenuLevel = 0;
byte lastMenuLevel = 1;
bool reopenLevel = false;

//PEDALS
byte brakePedalCalibrationStep = 0;
byte acceleratorPedalCalibrationStep = 0;

// STEERING
int16_t steeringSensor;
int16_t steeringPosition;
int16_t brakeSensor;
int16_t acceleratorSensor;
byte steeringCalibrationStep = 0;

// SYSTEM
// Every for loop variable
int i;
int iBlock;

//  ███████ ███████ ████████ ██    ██ ██████  
//  ██      ██         ██    ██    ██ ██   ██ 
//  ███████ █████      ██    ██    ██ ██████  
//       ██ ██         ██    ██    ██ ██      
//  ███████ ███████    ██     ██████  ██      
//
void setup()
{
  //Serial.begin(9600);
  #if isFirstTimeUploading
    StoreData(); //This will reset all calibrations - See FEATURES section above!
  #endif

  // SET BUTTON INPUTS
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);

  pinMode(4, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  // START ADC ///////////////////////////////////////////////////
  ADS.setMode(0);      // 0:continuous 1:single
  ADS.setGain(1);      // 0:6.144V  1:4.096V 2:2.048V 4:1.024V 8:0.512V 16:0.256V
  ADS.setDataRate(7);  // 0:slowest 4:default 7:fastest
  ADS.begin();

  // START DISPLAY ///////////////////////////////////////////////
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont(Stang5x7);
  DisplayMainScreen();

  // START JOYSTICK //////////////////////////////////////////////
  Joystick.begin(true);
  Joystick.setXAxisRange(joystickMin, joystickMax);
  Joystick.setYAxisRange(joystickMin, joystickMax);
  Joystick.setZAxisRange(joystickMin, joystickMax);
  gains[0].totalGain = forceGain;
  //gains[0].constantGain      = 100;
	//gains[0].rampGain          = 0;
	//gains[0].squareGain        = 0;
	//gains[0].sineGain          = 0;
	//gains[0].triangleGain      = 0;
	//gains[0].sawtoothdownGain  = 0;
	//gains[0].sawtoothupGain    = 0;
	//gains[0].springGain        = 0;
	//gains[0].damperGain        = 100;
	//gains[0].inertiaGain       = 100;
	//gains[0].frictionGain      = 100;
	//gains[0].customGain        = 0;
  
  Joystick.setGains(gains);
  
  //Joystick.setEffectParams(params);

  // START FORCE FEEDBACK /////////////////////////////////////////
  // Timer3 for FFB
  cli();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 399;
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31);
  TIMSK3 |= (1 << OCIE3A);
  sei();
  pinMode(motorPinDirection, OUTPUT);
  pinMode(motorPinPWM, OUTPUT);
  pinMode(motorPinEnable, OUTPUT);

  // LOAD DATA FROM EEPROM
  ReadData();
  digitalWrite(10, forceActive);
  menuLevel = 0;
  operationMode = 0;

  delay(100);
}


//  ██       ██████   ██████  ██████  
//  ██      ██    ██ ██    ██ ██   ██ 
//  ██      ██    ██ ██    ██ ██████  
//  ██      ██    ██ ██    ██ ██      
//  ███████  ██████   ██████  ██      
//
void loop() {

  // OPERATION MODES
  // 0 - In Game
  // 1 - In Menu
  // 2 - In Confirmation
  unsigned long currentMillis = millis();
  ReadButtons(currentMillis);
  ReadAnalogSensors();

  // IN GAME MODE
  if (operationMode == 0)  
  {
    processAccelleratorPedal();
    processBreakPedal();
    ProcessDataAndApply(currentMillis);
    if (buttonMenu) {
      operationMode = 1;  // Enable MENU
      oled.clear();
      menuLevel = lastMenuLevel; // Set MENU to first setting
    }
    showSensors();
  }

  if (operationMode == 1) MenuOperations();  // IN MENU MODE

  UpdateOldButtons();
  oldOperationMode = operationMode;

}


//  ███    ███ ███████ ███    ██ ██    ██ 
//  ████  ████ ██      ████   ██ ██    ██ 
//  ██ ████ ██ █████   ██ ██  ██ ██    ██ 
//  ██  ██  ██ ██      ██  ██ ██ ██    ██ 
//  ██      ██ ███████ ██   ████  ██████  
//
void MenuOperations()
{

  if (button[3] && !oldButton[3] && menuLevel != 0)  // NEXT OPTION
  {
    menuLevel += 1;
    if (menuLevel > menuLength) menuLevel = 1;
  }
  
  // GO TO SAVE CONFIRMATION
  if (button[4] && !oldButton[4] && menuLevel != 0) 
  {
    DisplayConfirmationScreen();
    lastMenuLevel = menuLevel;
    menuLevel = 0;

  } else if (button[4] && !oldButton[4] && menuLevel == 0)  // GO TO SAVE CONFIRMATION
  {
    ReadData();
    DisplayMainScreen();
    operationMode = 0;
  } else if (button[5] && !oldButton[5] && menuLevel == 0)  // GO TO SAVE CONFIRMATION
  {
    StoreData();
    ReadData();

    // APPLY NEW DATA RIGHT AWAY
    gains[0].totalGain = forceGain;
    Joystick.setGains(gains);

    digitalWrite(10, forceActive);

    DisplayMainScreen();
    operationMode = 0;
  }

  //    ______                              _   _           
  //   |  ____|                   /\       | | (_)          
  //   | |__ ___  _ __ ___ ___   /  \   ___| |_ ___   _____ 
  //   |  __/ _ \| '__/ __/ _ \ / /\ \ / __| __| \ \ / / _ \
  //   | | | (_) | | | (_|  __// ____ \ (__| |_| |\ V /  __/
  //   |_|  \___/|_|  \___\___/_/    \_\___|\__|_| \_/ \___|
  // 
  if (menuLevel == 1)
  {
    if (menuLevel != oldMenuLevel)
    {
      oled.clear();
    }
    DisplayTitleForceFeedback();
    oled.print(F("Active: "));
    if (!forceActive) { oled.print(F("No")); }
    else { oled.print(F("Yes")); }
    CleanLine();
    DisplayMenuChange();
    if (button[5] && !oldButton[5])  // SET NEW VALUE
    {
      forceActive = !forceActive;
      digitalWrite(10, forceActive);
    }
  }

  //    ______                  _____       _       
  //   |  ____|                / ____|     (_)      
  //   | |__ ___  _ __ ___ ___| |  __  __ _ _ _ __  
  //   |  __/ _ \| '__/ __/ _ \ | |_ |/ _` | | '_ \ 
  //   | | | (_) | | | (_|  __/ |__| | (_| | | | | |
  //   |_|  \___/|_|  \___\___|\_____|\__,_|_|_| |_|
  //
  if (menuLevel == 2)
  {
    if (menuLevel != oldMenuLevel)
    {
      oled.clear();
    }
    DisplayTitleForceFeedback();
    oled.print(F("Current Gain: "));
    oled.print(forceGain);
    CleanLineln();
    //oled.setRow(0); oled.setCol(0);
    oled.print(F("New value: "));
    int newGain = map(steeringSensor, steeringSensorMapLUT[0], steeringSensorMapLUT[10], 0, 200);
    if (newGain < 1) newGain = 1;
    if (newGain > 200) newGain = 200;
    oled.print(newGain);
    CleanLine();
    DisplayMenuSet();

    if (button[5] && !oldButton[5])  // SET NEW VALUE
    {
      forceGain = newGain;
    }
 
    DisplayMenuSet();
  }

//    ____                 _      _____         _       _ 
//   |  _ \               | |    |  __ \       | |     | |
//   | |_) |_ __ ___  __ _| | __ | |__) |__  __| | __ _| |
//   |  _ <| '__/ _ \/ _` | |/ / |  ___/ _ \/ _` |/ _` | |
//   | |_) | | |  __/ (_| |   <  | |  |  __/ (_| | (_| | |
//   |____/|_|  \___|\__,_|_|\_\ |_|   \___|\__,_|\__,_|_|
//    / ____|                                             
//   | |    _   _ _ ____   _____                          
//   | |   | | | | '__\ \ / / _ \                         
//   | |___| |_| | |   \ V /  __/                         
//    \_____\__,_|_|    \_/ \___|                         
// 
  if (menuLevel == 3)
  {
    if (menuLevel != oldMenuLevel)
    {
      oled.clear();
    }
    DisplayTitlePedalCurve();
    oled.println(F("Break pedal"));
    switch (brakeEase)
    {
      case EASEIN:
      oled.print("Ease In");
      break;
      case LINEAR:
      oled.print("Linear");
      break;
      case EASEOUT:
      oled.print("Ease Out");
      break;
    }
    CleanLine();

    if (button[5] && !oldButton[5])
    {
      brakeEase = (EasingMode)(((int)brakeEase + 1) % 3);
    }
 
    DisplayMenuChange();
  }

  //                         _                _               _____         _       _ 
  //       /\               | |              | |             |  __ \       | |     | |
  //      /  \   ___ ___ ___| | ___ _ __ __ _| |_ ___  _ __  | |__) |__  __| | __ _| |
  //     / /\ \ / __/ __/ _ \ |/ _ \ '__/ _` | __/ _ \| '__| |  ___/ _ \/ _` |/ _` | |
  //    / ____ \ (_| (_|  __/ |  __/ | | (_| | || (_) | |    | |  |  __/ (_| | (_| | |
  //   /_/____\_\___\___\___|_|\___|_|  \__,_|\__\___/|_|    |_|   \___|\__,_|\__,_|_|
  //    / ____|                                                                       
  //   | |    _   _ _ ____   _____                                                    
  //   | |   | | | | '__\ \ / / _ \                                                   
  //   | |___| |_| | |   \ V /  __/                                                   
  //    \_____\__,_|_|    \_/ \___|                                                   
  //   
  if (menuLevel == 4)
  {
    if (menuLevel != oldMenuLevel)
    {
      oled.clear();
    }
    DisplayTitlePedalCurve();
    oled.println(F("Accelerator pedal"));
    switch (acceleratorEase)
    {
      case EASEIN:
      oled.print("Ease In");
      break;
      case LINEAR:
      oled.print("Linear");
      break;
      case EASEOUT:
      oled.print("Ease Out");
      break;
    }
    CleanLine();

    if (button[5] && !oldButton[5])
    {
      acceleratorEase = (EasingMode)(((int)acceleratorEase + 1) % 3);
    }
 
    DisplayMenuChange();
  }

  //     _____      _ _ _               _   _              _____ _                 _             
  //    / ____|    | (_) |             | | (_)            / ____| |               (_)            
  //   | |     __ _| |_| |__  _ __ __ _| |_ _  ___  _ __ | (___ | |_ ___  ___ _ __ _ _ __   __ _ 
  //   | |    / _` | | | '_ \| '__/ _` | __| |/ _ \| '_ \ \___ \| __/ _ \/ _ \ '__| | '_ \ / _` |
  //   | |___| (_| | | | |_) | | | (_| | |_| | (_) | | | |____) | ||  __/  __/ |  | | | | | (_| |
  //    \_____\__,_|_|_|_.__/|_|  \__,_|\__|_|\___/|_| |_|_____/ \__\___|\___|_|  |_|_| |_|\__, |
  //                                                                                        __/ |
  //                                                                                       |___/ 

  if (menuLevel == 5)
  {
    if (menuLevel != oldMenuLevel)
    {
      steeringCalibrationStep = 0;
      oled.clear();
      DisplayTitleCalibration();
      oled.print(F("Steering wheel"));
      DisplayMenuStart();
    }

    if (button[5] && button[5] != oldButton[5])
    {
      switch (steeringCalibrationStep)
      {
        case 0:
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        oled.println(F("Rotate full left"));
        oled.print(F("  0 deg"));
        DisplayValueText();
        break;
        
        case 1:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F(" 90 deg"));
        DisplayValueText();
        break;

        case 2:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F("180 deg"));
        DisplayValueText();
        break;
        
        case 3:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F("270 deg"));
        DisplayValueText();
        break;
        
        case 4:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F("360 deg"));
        DisplayValueText();
        break;
        
        case 5:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        oled.println(F("Center point    "));
        oled.print(F("450 deg"));
        DisplayValueText();
        break;
        
        case 6:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        //CleanLine();
        oled.print(F("540 deg"));
        DisplayValueText();
        break;
        
        case 7:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F("630 deg"));
        DisplayValueText();
        break;
        
        case 8:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F("720 deg"));
        DisplayValueText();
        break;
        
        case 9:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F("810 deg"));
        DisplayValueText();
        break;
        
        case 10:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep++;
        DisplayTitleCalibrate();
        DisplayRotateToAngle();
        oled.print(F("900 deg"));
        DisplayValueText();
        break;

        case 11:
        steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
        steeringCalibrationStep = 0;
        reopenLevel = true;
        break;
      }
    }

    if (steeringCalibrationStep > 0 && steeringCalibrationStep < 12)
    {
      oled.setRow(2); oled.setCol(94);
      oled.print(steeringSensor);
      CleanLine();
      DisplayMenuSet();
    }
  }

  //     _____      _ _ _               _   _               
  //    / ____|    | (_) |             | | (_)              
  //   | |     __ _| |_| |__  _ __ __ _| |_ _  ___  _ __    
  //   | |    / _` | | | '_ \| '__/ _` | __| |/ _ \| '_ \   
  //   | |___| (_| | | | |_) | | | (_| | |_| | (_) | | | |  
  //    \_____\__,_|_|_|_.__/|_|  \__,_|\__|_|\___/|_| |_|  
  //    ____                 _      _____         _       _ 
  //   |  _ \               | |    |  __ \       | |     | |
  //   | |_) |_ __ ___  __ _| | __ | |__) |__  __| | __ _| |
  //   |  _ <| '__/ _ \/ _` | |/ / |  ___/ _ \/ _` |/ _` | |
  //   | |_) | | |  __/ (_| |   <  | |  |  __/ (_| | (_| | |
  //   |____/|_|  \___|\__,_|_|\_\ |_|   \___|\__,_|\__,_|_|
  // 
  if (menuLevel == 6)
  {
    if (menuLevel != oldMenuLevel)
    {
      brakePedalCalibrationStep = 0;
      oled.clear();
      DisplayTitleCalibration();
      oled.print(F("Break pedal"));
      DisplayMenuStart();
    }

    if (button[5] && button[5] != oldButton[5])
    {
      switch (brakePedalCalibrationStep)
      {
        case 0:
        brakePedalCalibrationStep++;
        DisplayTitleCalibrate();
        oled.println(F("Press the pedal 1mm"));
        oled.print(F("Minimum"));
        DisplayValueText();
        break;
        
        case 1:
        brakeSensorMin = brakeSensor;
        brakePedalCalibrationStep++;
        DisplayTitleCalibrate();
        oled.println(F("Full pess minus 1mm"));
        oled.print(F("Maximum"));
        DisplayValueText();
        break;

        case 2:
        brakeSensorMax = brakeSensor;
        brakePedalCalibrationStep = 0;
        reopenLevel = true;
        break;
      }
    }

    if (brakePedalCalibrationStep > 0 && brakePedalCalibrationStep < 3)
    {
      oled.setRow(2); oled.setCol(85);
      oled.print(brakeSensor);
      CleanLine();
      DisplayMenuSet();
    }
  }

  //     _____      _ _ _               _   _                                         
  //    / ____|    | (_) |             | | (_)                                        
  //   | |     __ _| |_| |__  _ __ __ _| |_ _  ___  _ __                              
  //   | |    / _` | | | '_ \| '__/ _` | __| |/ _ \| '_ \                             
  //   | |___| (_| | | | |_) | | | (_| | |_| | (_) | | | |                            
  //    \_____\__,_|_|_|_.__/|_|  \__,_|\__|_|\___/|_| |_|    _____         _       _ 
  //       /\               | |              | |             |  __ \       | |     | |
  //      /  \   ___ ___ ___| | ___ _ __ __ _| |_ ___  _ __  | |__) |__  __| | __ _| |
  //     / /\ \ / __/ __/ _ \ |/ _ \ '__/ _` | __/ _ \| '__| |  ___/ _ \/ _` |/ _` | |
  //    / ____ \ (_| (_|  __/ |  __/ | | (_| | || (_) | |    | |  |  __/ (_| | (_| | |
  //   /_/    \_\___\___\___|_|\___|_|  \__,_|\__\___/|_|    |_|   \___|\__,_|\__,_|_|
  // 
  if (menuLevel == 7)
  {
    if (menuLevel != oldMenuLevel)
    {
      acceleratorPedalCalibrationStep = 0;
      oled.clear();
      DisplayTitleCalibration();
      oled.print(F("Accelerator pedal"));
      DisplayMenuStart();
    }

    if (button[5] && button[5] != oldButton[5])
    {
      switch (acceleratorPedalCalibrationStep)
      {
        case 0:
          acceleratorPedalCalibrationStep++;
          DisplayTitleCalibrate();
          oled.println(F("Press the pedal 1mm"));
          oled.print(F("Minimum"));
          DisplayValueText();
          break;
        
        case 1:
          acceleratorSensorMin = acceleratorSensor;
          acceleratorPedalCalibrationStep++;
          DisplayTitleCalibrate();
          oled.println(F("Full press minus 1mm"));
          oled.print(F("Maximum"));
          DisplayValueText();
          break;

        case 2:
          acceleratorSensorMax = acceleratorSensor;
          acceleratorPedalCalibrationStep = 0;
          reopenLevel = true;
          break;
      }
    }

    if (acceleratorPedalCalibrationStep > 0 && acceleratorPedalCalibrationStep < 3)
    {
      oled.setRow(2);
      oled.setCol(85);
      oled.print(acceleratorSensor);
      CleanLine();
      DisplayMenuSet();
    }
  }

  if (reopenLevel)
  {
    reopenLevel = false;
    oldMenuLevel = -1;
  }
  else
  {
    oldMenuLevel = menuLevel;
  }

  delay(2);
}

//  ██████  ██ ███████ ██████  ██       █████  ██    ██ 
//  ██   ██ ██ ██      ██   ██ ██      ██   ██  ██  ██  
//  ██   ██ ██ ███████ ██████  ██      ███████   ████   
//  ██   ██ ██      ██ ██      ██      ██   ██    ██    
//  ██████  ██ ███████ ██      ███████ ██   ██    ██    
//

void CleanLine()
{
  oled.print(F("    "));
}
void CleanLineln()
{
  oled.println(F("    "));
}

void DisplayTitle(const __FlashStringHelper* title)
{
  oled.setInvertMode(false);
  oled.setRow(0);
  oled.setCol(0);
  oled.println(title);
}

// Usage:
void DisplayTitleCalibration()     { DisplayTitle(F("CALIBRATION")); }
void DisplayTitleCalibrate()       { DisplayTitle(F("CALIBRATE STEERING")); }
void DisplayTitleForceFeedback()   { DisplayTitle(F("FORCE FEEDBACK")); }
void DisplayTitlePedalCurve()      { DisplayTitle(F("PEDAL LINEARITY")); }

void DisplayRotateToAngle()
{
  oled.println(F("Rotate to angle:"));
}

void DisplayValueText()
{
  oled.print(F(" value:"));
}

void DisplayMainScreen()
{
  oled.clear();
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.print(F("STEERING PAD 900-F")); ////////////////////////////
  oled.setRow(3); oled.setCol(0);
  oled.print(F("v3.21    hold ")); /////////////////////////////
  oled.setInvertMode(true);
  oled.print(F(" menu ")); /////////////////////////////
  oled.setInvertMode(false);
  oled.print(F(">")); /////////////////////////////
}

void DisplayMenu(const __FlashStringHelper* label, uint8_t spaces)
{
  oled.setRow(3);
  oled.setCol(0);

  // Print " exit "
  oled.setInvertMode(true);
  oled.print(F(" exit "));
  oled.setInvertMode(false);

  // Print spacing
  while (spaces--) oled.print(' ');

  // Print second label
  oled.setInvertMode(true);
  oled.print(label);
  oled.setInvertMode(false);
}

// Wrappers (optional)
void DisplayMenuSet()    { DisplayMenu(F(" set "),    10); }
void DisplayMenuChange() { DisplayMenu(F(" change "), 7); }
void DisplayMenuStart()  { DisplayMenu(F(" start "),  8); }

void DisplayConfirmationScreen()
{
  oled.clear();
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.print(F("Save changes?"));
  oled.setRow(3); oled.setCol(0);
  oled.setInvertMode(true);
  oled.print(F(" no ")); ////////////////////////////
  oled.setInvertMode(false);
  oled.print(F("            ")); //////////////////
  oled.setInvertMode(true);
  oled.print(F(" yes ")); /////////////////////////////
  oled.setInvertMode(false);
}


//  ██ ███    ██ ██████  ██    ██ ████████ 
//  ██ ████   ██ ██   ██ ██    ██    ██    
//  ██ ██ ██  ██ ██████  ██    ██    ██    
//  ██ ██  ██ ██ ██      ██    ██    ██    
//  ██ ██   ████ ██       ██████     ██    
//
void ReadAnalogSensors()
{
  steeringSensor = ADS.readADC(0);
  brakeSensor = ADS.readADC(1);
  acceleratorSensor = ADS.readADC(2);
}

void ReadButtons(unsigned long currentMillis)
{
  sensorValues[0] = analogRead(A0);
  sensorValues[1] = analogRead(A1);
  sensorValues[2] = analogRead(A2);
  sensorValues[3] = analogRead(A3);

  for (i = 0; i < 15; i++)
  {
    buttonsMux[i] = false;
  }

  for (i = 0; i < 4; i++)
  {
    if (abs(sensorValues[i] - thresholds[0]) <= errorMargin) { buttonsMux[i * 3] = true; } // Button A
    if (abs(sensorValues[i] - thresholds[1]) <= errorMargin) { buttonsMux[i * 3 + 1] = true; } // Button B
    if (abs(sensorValues[i] - thresholds[2]) <= errorMargin) { buttonsMux[i * 3 + 2] = true; } // Button C
    if (abs(sensorValues[i] - thresholds[3]) <= errorMargin) { buttonsMux[i * 3] = true; buttonsMux[i * 3 + 1] = true; } // A + B
    if (abs(sensorValues[i] - thresholds[4]) <= errorMargin) { buttonsMux[i * 3] = true; buttonsMux[i * 3 + 2] = true; } // A + C
    if (abs(sensorValues[i] - thresholds[5]) <= errorMargin) { buttonsMux[i * 3 + 1] = true; buttonsMux[i * 3 + 2] = true; } // B + C
    if (abs(sensorValues[i] - thresholds[6]) <= errorMargin) { buttonsMux[i * 3] = true; buttonsMux[i * 3 + 1] = true; buttonsMux[i * 3 + 2] = true; } // A + B + C
  }

  button[1] = !digitalRead(4);
  button[2] = !digitalRead(7);

  CheckButton3Press(currentMillis);
  button[4] = buttonsMux[6];
  button[5] = buttonsMux[10];
  button[6] = buttonsMux[11];
  button[7] = buttonsMux[7];
  button[8] = buttonsMux[8];
  if (!button13OnHold) button[9] = buttonsMux[3];
  button[10] = buttonsMux[4];
  if (!button13OnHold) button[11] =  buttonsMux[5];
  button[12] = buttonsMux[1];
  CheckButton13press();
  if (button13OnHold)
  {
    if(buttonsMux[3] || buttonsMux[5])
    {
      if(!button13AsFunction) button13AsFunction = true;
    }
  }

  if (button13AsFunction)
  {
    button[14] = buttonsMux[3];
    button[15] = buttonsMux[5];
  }
}


void UpdateOldButtons()
{
  for (i = 1; i < 16; i++){
    oldButton[i] = button[i];
  }
  oldButtonMenu = buttonMenu;
}


void CheckButton13press()
{
  if (button[13]) button[13] = false; //reset button on next loop
  if (buttonsMux[2])
  {
    if (!button13OnHold)
    {
      button13OnHold = true;
      button[9] = false;
      button[11] = false;
    }
  }
  else if (!buttonsMux[2] && button13OnHold)
  {
    button13OnHold = false;
    button[14] = false;
    button[15] = false;
    if (!button13AsFunction) button[13] = true;
    button13AsFunction = false;
  }
}


void CheckButton3Press(unsigned long currentMillis)
{
  if (!buttonsMux[9])
  {
    if (oldButton[3]) button[3] = false;
    if (oldButtonMenu) buttonMenu = false;
  }

  if (buttonsMux[9])
  {  // button pressed
    if (!button3WasPressed)
    {
      button3WasPressed = true;
      button3StartTime = currentMillis;
      longPressTriggered = false;
    }
    else
    {
      // O botão continua pressionado: verifica se já atingiu o tempo para long press
      if (!longPressTriggered && (currentMillis - button3StartTime >= LONG_PRESS_THRESHOLD))
      {
        longPressTriggered = true;
        buttonMenu = true;
      }
    }
  }
  else
  {
    // O botão não está pressionado
    if (button3WasPressed)
    {
      // Detecta a transição: o botão foi solto
      if (!longPressTriggered)
      {
        button[3] = true;
      }
      // Reinicia os estados para a próxima detecção
      button3WasPressed = false;
      button3StartTime = 0;
      longPressTriggered = false;
    }
  }
}





//   █████  ██    ██ ██   ██     ███████ ██    ██ ███    ██  ██████ ████████ ██  ██████  ███    ██ ███████ 
//  ██   ██ ██    ██  ██ ██      ██      ██    ██ ████   ██ ██         ██    ██ ██    ██ ████   ██ ██      
//  ███████ ██    ██   ███       █████   ██    ██ ██ ██  ██ ██         ██    ██ ██    ██ ██ ██  ██ ███████ 
//  ██   ██ ██    ██  ██ ██      ██      ██    ██ ██  ██ ██ ██         ██    ██ ██    ██ ██  ██ ██      ██ 
//  ██   ██  ██████  ██   ██     ██       ██████  ██   ████  ██████    ██    ██  ██████  ██   ████ ███████ 
//

// TIMER3 INTERRUPTION
ISR(TIMER3_COMPA_vect)
{
  Joystick.getUSBPID();  // update FFB
}


// EASE IN CALCULATION
int16_t EaseIn(int16_t sensorData, int16_t minIn, int16_t maxIn, int16_t minOut, int16_t maxOut)
{
  float adjusted = ((float)(sensorData - minIn) / (maxIn - minIn)) * ((float)(sensorData - minIn) / (maxIn - minIn));
  return (float)minOut + adjusted * ((float)maxOut - (float)minOut);
}

// EASE OUT CALCULATION
int16_t EaseOut(int16_t sensorData, int16_t minIn, int16_t maxIn, int16_t minOut, int16_t maxOut)
{
  float adjusted = sqrt_approx(((float)(sensorData - minIn)) / ((float)(maxIn - minIn)));
  return (float)minOut + adjusted * ((float)maxOut - (float)minOut);
}

// SQUARE ROOT FAST CALCULATION
float sqrt_approx(float x)
{
  if (x <= 0.0f)
    return 0.0f;
  float guess = x;  
  guess = 0.5f * (guess + x / guess);
  guess = 0.5f * (guess + x / guess);
  return guess;
}


// --- FILTERING ---
int16_t lastStableValue = 0;
int16_t lastX = 0;
int16_t lastVelX = 0;
int16_t lastAccelX = 0;
unsigned long lastEffectsUpdate = 0;

int16_t applyDeadband(int16_t value) {
    if ((value > lastStableValue ? value - lastStableValue : lastStableValue - value) > 10)
      lastStableValue = value;
    return lastStableValue;
}

void calculateEffectParams(unsigned long currentMillis, int16_t steeringPosition){
  // set X Axis Spring Effect Param
  // joystickMin, joystickMax
  //map(value, realMinimum, realMaximum, actualMinimum, actualMaximum);
  params[0].springMaxPosition = joystickMax;
  params[0].springPosition = -steeringPosition;
  
  int16_t diffTime = currentMillis - lastEffectsUpdate;
  if(diffTime > 0){
    lastEffectsUpdate = currentMillis;
    int16_t positionChangeX = steeringPosition - lastX;
    int16_t velX = positionChangeX / diffTime;
    int16_t accelX = ((velX - lastVelX) * 5000) / diffTime;

    params[0].frictionPositionChange = -velX;
    params[0].inertiaAcceleration = -(accelX+lastAccelX)/2;
    params[0].damperVelocity = -velX;
    lastVelX = velX;
    lastAccelX = accelX;
    lastX = steeringPosition;   
  }
  params[0].damperMaxVelocity = 20;
  params[0].inertiaMaxAcceleration = 250;
  params[0].frictionMaxPositionChange = 20;
  Joystick.setEffectParams(params);
}

void showSensors(){
    const __FlashStringHelper* arrow = F(">");
    oled.setRow(1); oled.setCol(0);
    oled.setInvertMode(true);
    oled.print(F("ffb"));
    oled.setInvertMode(false);
     oled.print(arrow);
    oled.print(forces[0]);
    oled.print(F("  "));
    oled.setRow(1); oled.setCol(50);
    oled.setInvertMode(true);
    oled.print(F("x-axis")); 
    oled.setInvertMode(false);
    oled.print(arrow);
    oled.print(steeringPosition);
    oled.print(F("  "));
    oled.setRow(2); oled.setCol(0);
    oled.setInvertMode(true);
    oled.print(F("btn")); 
    oled.setInvertMode(false);
    oled.print(arrow);
    oled.print(F("                "));
    oled.setCol(26);
    for (i = 1; i <= 15; i++){
      if(button[i]){
        oled.print(i);
        oled.print(F(" "));
      }
    }
  
}

void processBreakPedal(){
   int16_t brakeFinalValue;
  // BREAK PEDAL
  if (brakeSensor < brakeSensorMin) brakeSensor = brakeSensorMin;
  if (brakeSensor > brakeSensorMax) brakeSensor = brakeSensorMax;
  switch (brakeEase)
  {
    case EASEIN:
    brakeFinalValue = EaseIn(brakeSensor, brakeSensorMin, brakeSensorMax, joystickMin, joystickMax);
    break;
    
    case LINEAR:
    brakeFinalValue = map(brakeSensor, brakeSensorMin, brakeSensorMax, joystickMin, joystickMax);
    break;

    case EASEOUT:
    brakeFinalValue = EaseOut(brakeSensor, brakeSensorMin, brakeSensorMax, joystickMin, joystickMax);
    break;
  }
  Joystick.setYAxis(brakeFinalValue);
}

void processAccelleratorPedal(){
  int16_t acceleratorFinalValue;
  // ACCELERATOR PEDAL
  if (acceleratorSensor < acceleratorSensorMin) acceleratorSensor = acceleratorSensorMin;
  if (acceleratorSensor > acceleratorSensorMax) acceleratorSensor = acceleratorSensorMax;
  switch (acceleratorEase)
  {
    case EASEIN:
      acceleratorFinalValue = EaseIn(acceleratorSensor, acceleratorSensorMin, acceleratorSensorMax, joystickMin, joystickMax);
      break;
      
    case LINEAR:
      acceleratorFinalValue = map(acceleratorSensor, acceleratorSensorMin, acceleratorSensorMax, joystickMin, joystickMax);
      break;

    case EASEOUT:
      acceleratorFinalValue = EaseOut(acceleratorSensor, acceleratorSensorMin, acceleratorSensorMax, joystickMin, joystickMax);
      break;
  }

  Joystick.setZAxis(acceleratorFinalValue);
}

// SEND DATA TO JOYSTICK
void ProcessDataAndApply(unsigned long currentMillis)
{
  // STEERING
  //steeringPosition = mapLUT(steeringSensor);
  steeringPosition = mapLUT(applyDeadband(steeringSensor));
  
  // Apply position
  Joystick.setXAxis(steeringPosition);

  // Update state/setEffectParams with steeringPosition
  calculateEffectParams(currentMillis, steeringPosition);

  // GET FORCE FEEDBACK
  Joystick.getForce(forces);

  
  //                       damping
  //                         velocity
  int16_t finalForce = forces[0];// + (-(steeringPosition - lastPosition)) * 0.005; //(-velocity * (0.01 - ((float)forceGain / 200.0) * (0.01 - 0.001)));;
  
  lastPosition = steeringPosition;
  
  // 4. Limitar força total
  finalForce = constrain(finalForce, -255, 255);

  if (forces[0] > 0)
  {
    digitalWrite(motorPinDirection, HIGH);
    analogWrite(motorPinPWM, abs(finalForce));
  }
  else if (forces[0] < 0)
  {
    digitalWrite(motorPinDirection, LOW);
    analogWrite(motorPinPWM, abs(finalForce));
  }
  else
  {
    analogWrite(motorPinPWM, 0);
  }

  // BUTTONS
  for (i = 1; i <= 15; i++){
     if (button[i] && !oldButton[i]) Joystick.pressButton(i-1);
  }

  for (i = 1; i <= 15; i++){
    if (!button[i] && oldButton[i]) Joystick.releaseButton(i-1);
  }
}

// RETURN LUT CALCULATED VALUE

int16_t mapLUT(int16_t inputValue)
{
  // Out of limits
  if (inputValue <= steeringSensorMapLUT[0]) return outputLUT[0];
  if (inputValue >= steeringSensorMapLUT[10]) return outputLUT[10];

  for (iBlock = 0; iBlock < 10; iBlock++) {
    int16_t in_min = steeringSensorMapLUT[iBlock];
    int16_t in_max = steeringSensorMapLUT[iBlock + 1];

    if ((iBlock < 9 && inputValue >= in_min && inputValue < in_max) ||
        (iBlock == 9 && inputValue >= in_min && inputValue <= in_max)) {

      // Offset input to 0
      int32_t in_rel = inputValue - in_min;
      int32_t in_range = in_max - in_min;

      // Offset output to 0
      int32_t out_range = outputLUT[iBlock + 1] - outputLUT[iBlock];

      // Interpolação relativa com arredondamento
      int32_t rel_output = (in_rel * out_range + (in_range / 2)) / in_range;

      // Somar mínimo do bloco para obter output absoluto
      return (int16_t)(outputLUT[iBlock] + rel_output);
    }
  }
}


//  ███████ ███████ ██████  ██████   ██████  ███    ███ 
//  ██      ██      ██   ██ ██   ██ ██    ██ ████  ████ 
//  █████   █████   ██████  ██████  ██    ██ ██ ████ ██ 
//  ██      ██      ██      ██   ██ ██    ██ ██  ██  ██ 
//  ███████ ███████ ██      ██   ██  ██████  ██      ██ 
// 
#include <avr/eeprom.h>

void StoreData()
{
  // ADDRESS BOOK
  // 0 (22 bytes) Steering LUT Array
  // 21
  //////////////////////////////////
  // 22 (4 bytes) Brake Pedal
  // 25
  //////////////////////////////////
  // 26 (4 bytes) Accelerator Pedal
  // 29
  //////////////////////////////////
  // 30 (2 bytes) Force Gain
  // 31
  //////////////////////////////////
  // 32 (1 byte) Force Active
  //////////////////////////////////
  // 33 (1 byte) Brake Easing Mode
  //////////////////////////////////
  // 34 (1 byte) Accelerator Easing Mode
  //////////////////////////////////

  // Steering LUT Array
  for (i = 0; i < 11; i++)
  {
    eeprom_update_word((uint16_t*)(i * sizeof(int16_t)), steeringSensorMapLUT[i]);
  }
  
  // Brake Pedal
  eeprom_update_word((uint16_t*)22, brakeSensorMin);
  eeprom_update_word((uint16_t*)24, brakeSensorMax);
  
  // Accelerator Pedal
  eeprom_update_word((uint16_t*)26, acceleratorSensorMin);
  eeprom_update_word((uint16_t*)28, acceleratorSensorMax);
  
  // Force Feedback Gain
  eeprom_update_word((uint16_t*)30, forceGain);
  
  // Force Feedback Active
  eeprom_update_byte((uint8_t*)32, forceActive);

  // Store Easing Modes as one byte each
  eeprom_update_byte((uint8_t*)33, (uint8_t) brakeEase);
  eeprom_update_byte((uint8_t*)34, (uint8_t) acceleratorEase);
}

void ReadData()
{

  // Steering LUT Array
  for (i = 0; i < 11; i++)
  {
    steeringSensorMapLUT[i] = eeprom_read_word((uint16_t*)(i * sizeof(int16_t)));
    //Serial.println(steeringSensorMapLUT[i]);
  }

  // Break Pedal
  brakeSensorMin = eeprom_read_word((uint16_t*)22);
  brakeSensorMax = eeprom_read_word((uint16_t*)24);

  // Accelerator Pedal
  acceleratorSensorMin = eeprom_read_word((uint16_t*)26);
  acceleratorSensorMax = eeprom_read_word((uint16_t*)28);

  // Force Feedback Gain
  forceGain = eeprom_read_word((uint16_t*)30);
  
  // Force Feedback Active
  forceActive = eeprom_read_byte((uint8_t*)32);

  // Read the Easing Modes
  uint8_t temp;
  temp = eeprom_read_byte((uint8_t*)33);
  brakeEase = static_cast<EasingMode>(temp);
  
  temp = eeprom_read_byte((uint8_t*)34);
  acceleratorEase = static_cast<EasingMode>(temp);
}
