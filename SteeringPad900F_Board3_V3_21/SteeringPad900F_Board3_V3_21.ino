/*
 * this is the old way of spoofing, included usb_rename.cpp to do this. see code below, no boards.txt has to be overwritten.
 * TODO: enable CDC_DISABLED dynamically (if needed at all, needs more investigation)
 * 
 * -------------------------------------
 * 
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
 * sketch folder
 * C:\Users\%USERNAME%\AppData\Local\Temp\arduino\sketches\
 * 
 * size reporting
 * C:\Users\%USERNAME%\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.7\platform.txt
 * compiler.nm.cmd=avr-nm
 * recipe.hooks.linking.postlink.1.pattern="{compiler.path}{compiler.nm.cmd}" --size-sort --print-size -r "{build.path}/{build.project_name}.elf"
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
#to make the controller work with FH4 spoof the pid of "Logitech G27 Racing Wheel USB"
#mmicro.build.pid=0xC29B
mmicro.build.usb_product="Steering Pad 900-F"
mmicro.build.usb_manufacturer="Logitech, Inc"
mmicro.build.board=AVR_MICRO
mmicro.build.core=arduino
mmicro.build.variant=micro
mmicro.build.extra_flags={build.usb_flags}
#to make the controller work with certain games, add this line 
#(it will make the device not show up as composite device (HID+CDC) but not very handy while debugging the code)
#mmicro.build.extra_flags=-DCDC_DISABLED {build.usb_flags}

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

#include "src/usb_rename.h"

// ADC
const uint8_t SCL_PIN = SCL;
const uint8_t SDA_PIN = SDA;
const uint8_t DELAY_MICROS = 0;

#include <AceWire.h>
#include <DigitalWriteFast.h>
#include <ace_wire/SimpleWireFastInterface.h>
using ace_wire::SimpleWireFastInterface;

using WireInterface = SimpleWireFastInterface<SDA_PIN, SCL_PIN, DELAY_MICROS>;
WireInterface wireInterface;

#include "ADS1X15.h"
ADS1115<WireInterface> ADS(0x48, &wireInterface);

// ALERT/RDY pin connected to PIN0 of the pro micro?
#define ADS_INTERRUPT_PIN_0_ENABLED true

// DISPLAY
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAceWire.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiAceWire<WireInterface> oled(wireInterface);

// JOYSTICK
#include <Joystick.h>

#define NUMBUTTONS 22

bool altMode = false;
struct EarlyInit {
    EarlyInit() {
        // code that runs before setup()
        pinMode(4, INPUT_PULLUP);
        pinMode(7, INPUT_PULLUP);
        if(!digitalRead(4) || !digitalRead(7)){
          altMode = true;
        }
    }
} earlyInit;

Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,
  NUMBUTTONS,     // Buttons (Max 32)
  0,      // Hats (Max 2) - seems to be needed to be detected in FH4 ( > 0 makes it crash in beamNG because _hidReportSize too big?)
  true,   // X Axis
  true,   // Y Axis
  !altMode,  // Z Axis
  false,  // X Rotation
  false,  // Y Rotation
  altMode,   // Z Rotation
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

// OPTIONAL INTERRUPT HANDLING ON ALERT/RDY PIN
volatile bool RDY = false; // option to pause conversions when doing heavy stuff

bool showSensorsActive = true;

#if ADS_INTERRUPT_PIN_0_ENABLED
volatile int16_t adsValues[3] = { 0, 0, 0 };
// try sampling at 820SPS
void adsReady()
{
RDY = true;
}
#endif


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
  224, // button 3
  359, // button 5
  480, // button 6
  161, // buttons 3 & 5
  184, // buttons 3 & 6
  259, // buttons 5 & 6
  140  // buttons 3 & 5 & 6
}; 

const unsigned long LONG_PRESS_THRESHOLD = 1000;  // 1 second

// EEPROM

// JOYSTICK
const int16_t joystickMin = -32767;
const int16_t joystickMax = 32767;

// FORCE FEEDBACK
Gains gains[1];
EffectParams params[1];

#define motorPinDirection 8
#define motorPinPWM 9
#define motorPinEnable 10

// MENU
byte menuLength = 8;



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

bool button[NUMBUTTONS + 1]; // use 1 based index for readability
bool buttonMenu;

bool oldButton[NUMBUTTONS + 1]; // use 1 based index for readability
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
byte showSensorsStep = 0;

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
  //UDIEN &= ~(1 << SOFE);   // Disable USB Start-of-Frame interrupt
  //UDIEN |= (1 << SOFE); // Enable SOF interrupt

  //Serial.begin(230400);
  
  #if ADS_INTERRUPT_PIN_0_ENABLED
  // disable interrupt during setup if previously enabled, prevent interference
  ADS.setComparatorQueConvert(ADS1x15_COMP_QUE_CONV_DISABLE);
  ADS.requestADC(0);
  #endif

  #if isFirstTimeUploading
    StoreData(); //This will reset all calibrations - See FEATURES section above!
  #endif

  // SET BUTTON INPUTS
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);

  //already done in earlyInit
  //pinMode(4, INPUT_PULLUP);
  //pinMode(7, INPUT_PULLUP);

  // set different usb id if blinker button pressed
  // todo MisterFPGA compatibility
  // @see http://github.com/MiSTer-devel/Main_MiSTer/blob/5ea051dcea8e29ef9a61e4b732c004e724a30aa4/input.cpp#L4709
  // @see https://github.com/MiSTer-devel/Main_MiSTer/blob/5ea051dcea8e29ef9a61e4b732c004e724a30aa4/input.cpp#L4695
  // T300RS uses axis 5 (rZ) for throttle
  if(altMode){
     //0x2341 arduino id //27926
     USBRename dummy = USBRename(
        "Steering Pad 900-F (T300RS mode)",
        "Thrustmaster",
        "1337-T300RS",
        0x044f,   // VID
        0xb66e    // PID T300RS Racing Wheel (PC/PS3)
    );
  }else{
    USBRename dummy = USBRename(
        "Steering Pad 900-F",
        "tecteun",
        "1337",
        0x046d,   // VID (Logitech)
        0x0000    // PID (undef)
    );
  }

  // START DISPLAY ///////////////////////////////////////////////
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont(Stang5x7);

  // intro animation
  // use timer1 for some pseudo random noise
  byte c = 25;
  while (c-- > 0) {
    for (byte page = 0; page < 4; page++) {
      oled.setRow(page);
      oled.setCol(0);
      for (byte col = 0; col < 128; col++) {
        oled.ssd1306WriteRam((0xFF ^ TCNT1L)+analogRead(A0));
      }
    }
  }
 /*
  byte c = 25;
  while (c-- > 0) {
    for (byte page = 0; page < 4; page++) {
      oled.setRow(page);
      oled.setCol(0);
  
      for (byte col = 0; col < 128; col++) {
        float v = sin((col + c) * col/page);   // smooth wave
        uint8_t b = (v > 0) ? 0xFF : 0x00; // binary ON/OFF
        oled.ssd1306WriteRam(b);
      }
    }
  }
  */

  // START ADC ///////////////////////////////////////////////////
  ADS.begin();
  ADS.setMode(0);      // 0:continuous 1:single
  ADS.setGain(1);      // 0:6.144V  1:4.096V 2:2.048V 4:1.024V 8:0.512V 16:0.256V
  ADS.setDataRate(7);  // 0:slowest 4:default 7:fastest

  #if ADS_INTERRUPT_PIN_0_ENABLED
  //  SET ALERT RDY PIN
  ADS.setComparatorThresholdHigh(0x8000);
  ADS.setComparatorThresholdLow(0x0000);
  ADS.setComparatorQueConvert(ADS1x15_COMP_QUE_CONV_TRIGGER_1);

  //  SET INTERRUPT HANDLER TO CATCH CONVERSION READY
  pinMode(0, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(0), adsReady, RISING);
  #else
  ADS.setConversionDelay(1); // 1ms (default: 8)
  #endif

  // START JOYSTICK //////////////////////////////////////////////
  Joystick.begin(false); // <- no autoupdate, end of mainloop does this
  Joystick.setXAxisRange(joystickMin, joystickMax);
  Joystick.setYAxisRange(joystickMin, joystickMax);
  if(altMode){
    Joystick.setRzAxisRange(joystickMin, joystickMax);
  }else{
    Joystick.setZAxisRange(joystickMin, joystickMax);
  }
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

  // const effect settings
  params[0].springMaxPosition = joystickMax;
  params[0].damperMaxVelocity = 1500;
  params[0].inertiaMaxAcceleration = 8000;
  params[0].frictionMaxPositionChange = 1500;

  // START FORCE FEEDBACK /////////////////////////////////////////
  // Timer3 for FFB
  
  cli();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 399; // 5 kHz
  //OCR3A = 1999;  // 1 kHz (= equal to HID update rate)
  //OCR3A = 3999;  // 500 Hz
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

  DisplayMainScreen();

  delay(100);

  #if ADS_INTERRUPT_PIN_0_ENABLED
  ADS.requestADC(0);   //  start the interrupts, sample channel 0
  #endif

}


//  ██       ██████   ██████  ██████  
//  ██      ██    ██ ██    ██ ██   ██ 
//  ██      ██    ██ ██    ██ ██████  
//  ██      ██    ██ ██    ██ ██      
//  ███████  ██████   ██████  ██      
//
/*
volatile bool sofTick = false;
*/
#define debug 0
bool sampleReady = false;
void loop() {
  unsigned long currentMillis = millis();
  static int16_t diffTime = 1;
  #if ADS_INTERRUPT_PIN_0_ENABLED
  if(!RDY){
    return;
  }
  if(RDY && !sampleReady){ //gather new complete sample
    byte lastRequest = ADS.lastRequest();
    adsValues[lastRequest] = ADS.getValue();
    lastRequest++; //prepare to sample next channel
    if(lastRequest > 2){ 
      lastRequest = 0;
      sampleReady = true;
    }else{
      ADS.requestADC(lastRequest);
      //delayMicroseconds(1160); //(1s/860SPS)
      RDY = false;
    }
  }
  //iterate if sample not ready yet
  if(!sampleReady){
    return;
  }
  #endif
  #if debug
  unsigned long us = micros();
  static unsigned long lastTime = 0;
  static int rps = 0;
  #endif
  


  // OPERATION MODES
  // 0 - In Game
  // 1 - In Menu
  // 2 - In Confirmation
  //Joystick.getUSBPID();  // update FFB
  ReadButtons(currentMillis);
  ReadAnalogSensors();
  // IN GAME MODE
  if (operationMode == 0)  
  {    
    if(showSensorsActive)
      showSensorsSM(diffTime, steeringPosition);
    /*
     * test big blocker
     Joystick.getUSBPID();  // update FFB
      Joystick.getUSBPID();  // update FFB
       Joystick.getUSBPID();  // update FFB
        Joystick.getUSBPID();  // update FFB
         Joystick.getUSBPID();  // update FFB
          Joystick.getUSBPID();  // update FFB
           Joystick.getUSBPID();  // update FFB
            Joystick.getUSBPID();  // update FFB
    delay(200); 
    */  
    
    processAccelleratorPedal();
    processBreakPedal();
    diffTime = ProcessDataAndApply(currentMillis);
    if (buttonMenu) {
      operationMode = 1;  // Enable MENU
      oled.clear();
      menuLevel = lastMenuLevel; // Set MENU to first setting
    }
  
  }

  if (operationMode == 1){
    MenuOperations();  // IN MENU MODE
  }

  Joystick.sendState();

  UpdateOldButtons();
  oldOperationMode = operationMode;

  #if debug
  rps++;
  
  if (millis() - lastTime >= 1000){
    lastTime = millis();
    Serial.print(rps);
    Serial.print("rps looptime:");
    Serial.println(micros() - us);
    rps = 0;
  }
  #endif

  // end transmission to oled, OPTIMIZE_I2C does not end transmission, interferes with ADS1115
  wireInterface.endTransmission();
  #if ADS_INTERRUPT_PIN_0_ENABLED
  // gather new sample in next loop
  sampleReady = false;
  // reset adc to start with channel 0 again
  ADS.requestADC(0);
  // reset RDY flag
  RDY = false;
  #endif
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
  
  // static lambda helper (save some space)
  static auto steeringCalibrationStepFunc = [](const __FlashStringHelper* title, const __FlashStringHelper* pre = NULL){
    steeringSensorMapLUT[steeringCalibrationStep - 1] = steeringSensor;
    steeringCalibrationStep++;
    DisplayTitleCalibrate();
    oled.println(F("Rotate to angle:"));
    if(pre){
      oled.println(pre);
    }
    oled.print(title);
    DisplayValueText();
  };
  
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
        steeringCalibrationStepFunc(F(" 90 deg"));
        break;

        case 2:
        steeringCalibrationStepFunc(F("180 deg"));
        break;
        
        case 3:
        steeringCalibrationStepFunc(F("270 deg"));
        break;
        
        case 4:
        steeringCalibrationStepFunc(F("360 deg"));
        break;
        
        case 5:
        steeringCalibrationStepFunc(F("450 deg"), F("Center point    "));
        break;
        
        case 6:
        steeringCalibrationStepFunc(F("540 deg"));
        break;
        
        case 7:
        steeringCalibrationStepFunc(F("630 deg"));
        break;
        
        case 8:
        steeringCalibrationStepFunc(F("720 deg"));
        break;
        
        case 9:
        steeringCalibrationStepFunc(F("810 deg"));
        break;
        
        case 10:
        steeringCalibrationStepFunc(F("900 deg"));
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
        oled.println(F("Full press minus 1mm"));
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
  // SHOWSENSORS ACTIVE
  if (menuLevel == 8)
  {
    if (menuLevel != oldMenuLevel)
    {
      oled.clear();
    }
    DisplayTitleShowSensorsActive();
    oled.print(F("Active: "));
    if (!showSensorsActive) { oled.print(F("No")); }
    else { oled.print(F("Yes")); }
    CleanLine();
    DisplayMenuChange();
    if (button[5] && !oldButton[5])  // SET NEW VALUE
    {
      showSensorsActive = !showSensorsActive;
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
void DisplayTitleShowSensorsActive()   { DisplayTitle(F("SHOW SENSORDATA")); }
void DisplayTitlePedalCurve()      { DisplayTitle(F("PEDAL LINEARITY")); }



void DisplayValueText()
{
  oled.print(F(" value:"));
}

void DisplayMainScreen()
{
  oled.clear();
  oled.setInvertMode(false);
  oled.setRow(0); oled.setCol(0);
  oled.setLetterSpacing(0);
  oled.print(F("STEERING PAD 900-F")); ////////////////////////////
  if(altMode){
    oled.print(F(" (ALT)"));
  }
  oled.setLetterSpacing(1);
  oled.setRow(3); oled.setCol(0);
  oled.print(F("         hold ")); /////////////////////////////
  oled.setInvertMode(true);
  oled.print(F(" menu ")); /////////////////////////////
  oled.setInvertMode(false);
  oled.print(F(">")); /////////////////////////////

  if(showSensorsActive){
    showSensorsStep = 0;
    showSensorsLabels();
  }
}

void DisplayMenu(const __FlashStringHelper* label, uint8_t spaces)
{
  oled.setLetterSpacing(1);
  
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
  #if ADS_INTERRUPT_PIN_0_ENABLED
  steeringSensor = adsValues[1];
  brakeSensor = adsValues[2];
  acceleratorSensor = adsValues[0];
  //Serial.println(steeringSensor);
  #else
  steeringSensor = ADS.readADC(1);
  brakeSensor = ADS.readADC(2);
  acceleratorSensor = ADS.readADC(0);
  #endif
  #if debug
  Serial.print(steeringSensor);
  Serial.print('\t');
  Serial.print(brakeSensor);
  Serial.print('\t');
  Serial.println(acceleratorSensor);
  #endif
}

void ReadButtons(unsigned long currentMillis)
{
  sensorValues[0] = analogRead(A0);
  sensorValues[1] = analogRead(A1);
  sensorValues[2] = analogRead(A2);
  sensorValues[3] = analogRead(A3);
/*
  for (i = 0; i < 15; i++)
  {
    buttonsMux[i] = false;
  }
*/
 // Fast reset of 15 booleans 
  memset(buttonsMux, 0, 15);

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
  if (!button13OnHold) button[4] = buttonsMux[6];
  if (!button13OnHold) button[5] = buttonsMux[10];
  if (!button13OnHold) button[6] = buttonsMux[11];
  if (!button13OnHold) button[7] = buttonsMux[7];
  if (!button13OnHold) button[8] = buttonsMux[8];
  if (!button13OnHold) button[9] = buttonsMux[3];
  if (!button13OnHold) button[10]= buttonsMux[4];
  if (!button13OnHold) button[11]= buttonsMux[5];
  button[12] = buttonsMux[1];
  CheckButton13press();
  if (button13OnHold)
  {
    if(buttonsMux[3] || buttonsMux[4] || buttonsMux[5] || buttonsMux[6] || buttonsMux[7] ||  buttonsMux[8] || buttonsMux[9] || buttonsMux[10] || buttonsMux[11])
    {
      if(!button13AsFunction) button13AsFunction = true;
    }
  }

  if (button13AsFunction)
  {
    // 3/14 is special, also has menu function -> CheckButton3Press
    button[15] = buttonsMux[6];
    button[16] = buttonsMux[10];
    button[17] = buttonsMux[11];
    button[18] = buttonsMux[7];
    button[19] = buttonsMux[8];
    button[20] = buttonsMux[3];
    button[21] = buttonsMux[4];
    button[22] = buttonsMux[5];
  }
}


void UpdateOldButtons()
{
  /*
  for (i = 1; i <= 17; i++){
    oldButton[i] = button[i];
  }
  */
  
  memcpy(oldButton + 1, button + 1, NUMBUTTONS);
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
      button[6] = false;
      button[9] = false;
      button[11] = false;
    }
  }
  else if (!buttonsMux[2] && button13OnHold)
  {
    button13OnHold = false;

    button[15] = false;
    button[16] = false;
    button[17] = false;
    button[18] = false;
    button[19] = false;
    button[20] = false;
    button[21] = false;
    button[22] = false;
    if (!button13AsFunction) button[13] = true;
    button13AsFunction = false;
  }
}


void CheckButton3Press(unsigned long currentMillis)
{
  if (!buttonsMux[9])
  {
    if (oldButton[14]) button[14] = false;
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
        if(button13OnHold){
          button[14] = true;
        }else{
          button[3] = true;
        }
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
int16_t applyDeadband(int16_t value) {
    static int16_t lastStableValue = 0;
    if ((value > lastStableValue ? value - lastStableValue : lastStableValue - value) > 10)
      lastStableValue = value;
    return lastStableValue;
}

int16_t smoothSignal(int16_t raw) { 
  static int32_t filtered = 0; 
  if(filtered == 0){
    filtered = raw;
  }
   filtered += raw; 
   return (int16_t)filtered/2; 
}
#define BUFLEN 4
int16_t posBuf[BUFLEN];
int16_t dtBuf[BUFLEN];
uint8_t pIdx = 0;

int16_t calculateEffectParams(unsigned long now, int16_t pos){
  static unsigned long lastEffectsUpdate = 0;
  static int16_t lastX = 0;
  static int16_t sum = 0;
  static int16_t sumdt = 0;
  static int16_t lastAvg = 0;
  
  params[0].springPosition = -pos;

  int16_t dt = now - lastEffectsUpdate;
  if(dt > 0){
    // --- rolling window update ---
    lastEffectsUpdate = now;
    
    sum   -= posBuf[pIdx];
    sumdt -= dtBuf[pIdx];
    
    int16_t d = pos - lastX;   // raw Δx
    posBuf[pIdx] = d;
    dtBuf[pIdx]  = dt;
    
    sum   += d;
    sumdt += dt;
    
    pIdx = (pIdx + 1) % BUFLEN;
    
    lastX = pos;
    
    // --- velocity over the whole window ---
    // vel = (ΣΔx / ΣΔt) * scale
    int32_t velNum = (int32_t)sum * 50;   // scale first
    int16_t vel = velNum / (int32_t)sumdt;
    
    // --- acceleration from change in velocity ---
    // a = Δv / Δt  (using *actual* dt, not window dt)
    static int16_t lastVel = 0;
    
    int32_t dv = (int32_t)vel - lastVel;
    
    // scale factor: increase if you want stronger inertia
    int16_t rawAcc = (dv * 1000) / dt;  
    
    lastVel = vel;
    
    
    // --- smooth acceleration to remove peaks ---
    static int16_t accFilt = 0;
    
    // 80% old, 20% new (tune to taste)
    accFilt = (accFilt * 8 + rawAcc * 2) / 10;
    
    int16_t acc = accFilt;
    
    
    // --- output ---
    params[0].frictionPositionChange = -vel;
    params[0].inertiaAcceleration    = acc;
    params[0].damperVelocity         = -vel;


    //Serial.print(pos);
    //Serial.print('\t');    
    //Serial.print(sum);
    //Serial.print('\t'); 
    //Serial.print(vel);
    //Serial.print('\t');
    //Serial.print(acc);
    //Serial.println();
    
  }
  
  Joystick.setEffectParams(params);
  return dt;
}

void showSensorsLabels2(){
  const __FlashStringHelper* arrow = F(">");
    oled.setRow(1); oled.setCol(0);
    oled.setInvertMode(true);
    oled.print(F("ffb"));
    oled.setInvertMode(false);
    oled.print(arrow);
    
    oled.setCol(50);
    oled.setInvertMode(true);
    oled.print(F("x-axis")); 
    oled.setInvertMode(false);
    oled.print(arrow);
    
    oled.setRow(2); oled.setCol(0);
    oled.setInvertMode(true);
    oled.print(F("btn")); 
    oled.setInvertMode(false);
    oled.print(arrow);
    
    oled.setRow(3); oled.setCol(0);
    oled.setInvertMode(true);
    oled.print(F("Hz")); 
    oled.setInvertMode(false);
    oled.print(arrow);
}
void showSensorsLabels(){
  const __FlashStringHelper* arrow = F(">");
    oled.setLetterSpacing(0);
    oled.setRow(1); oled.setCol(0);
    oled.setInvertMode(true);

    oled.print(F(" "));
    oled.setCol(2);
    oled.print(F("ffb"));
    oled.print(F(" "));
    oled.setCol(19);
    oled.setInvertMode(false);
    oled.print(arrow);
    
    oled.setCol(50);
    oled.setInvertMode(true);
    oled.print(F(" "));
    oled.setCol(52);
    oled.print(F("btn"));
    oled.print(F(" "));
    oled.setCol(69);
    oled.setInvertMode(false);
    oled.print(arrow);
    
    oled.setRow(3); oled.setCol(0);
    oled.setInvertMode(true);
    oled.print(F(" "));
    oled.setCol(2);
    oled.print(F("Hz")); 
    oled.print(F(" "));
    oled.setCol(14);
    oled.setInvertMode(false);
    oled.print(arrow);
    oled.setLetterSpacing(1);
}

void showSensorsSM2(int16_t diffTime, int16_t steeringPosition)
{
    const __FlashStringHelper* empty = F("  ");
    static uint8_t pos = 0;
    static char buf[32];
    static double diffTimeAvg = 0;

    if(diffTimeAvg == 0)
      diffTimeAvg = diffTime;
    else
      diffTimeAvg = ((.8*diffTimeAvg) + (.2*diffTime));

    switch (showSensorsStep)
    {
        case 0: // forces + steering (row 1)
            oled.setRow(1); oled.setCol(25);
            oled.print(forces[0]);
            oled.print(empty);
            break;
        case 1: 
            oled.setCol(91);
            oled.print(steeringPosition);
            oled.print(empty);
            break;
        case 2: 
            oled.setRow(2); oled.setCol(25);
            
            memset(buf, 0, sizeof(buf)); //clear buf
            pos = 0;
        break;
        case 3: 
            for (uint8_t i = 1; i <= NUMBUTTONS; i++) {
              if (button[i]) {
                // print tens digit only if >= 10
                if (i >= 10) {
                  buf[pos++] = '0' + (i / 10);
                }
                buf[pos++] = '0' + (i % 10);  // ones digit
                buf[pos++] = ' ';            // space
              }
            }
            buf[pos] = '\0';
        break;
        case 4: // FPS (row 3)
            oled.clearToEOL();
            oled.print(buf);
            break;
        case 5:
        oled.setRow(3); oled.setCol(16);
        
          oled.print((uint16_t)(1000.0/diffTimeAvg));
          oled.print(empty);
        break;
    }

  showSensorsStep++;
  if (showSensorsStep > 5) showSensorsStep = 0;
}

void showSensorsSM(int16_t diffTime, int16_t steeringPosition)
{
    uint16_t out = map((steeringPosition >> 9)+64, 0, 127, 0, 122);
    const __FlashStringHelper* empty = F("  ");
    static uint8_t pos = 0;
    static char buf[32];
    static double diffTimeAvg = 0;
    static byte fcount = 0;

    if(diffTimeAvg == 0)
      diffTimeAvg = diffTime;
    else
      diffTimeAvg = ((.8*diffTimeAvg) + (.2*diffTime));

    switch (showSensorsStep)
    {
        case 0:
        {
            oled.setRow(1); oled.setCol(25);
            oled.print(forces[0]);
            oled.print(empty);
            oled.setCol(73);
            break;
        }
        case 1: 
        {
            pos = 0;
            memset(buf, 0, sizeof(buf)); //clear buf
            for (uint8_t i = 1; i <= NUMBUTTONS; i++) {
              if (!button[i]) continue;
              
              // print tens digit only if >= 10
              if (i >= 10) {
                buf[pos++] = '0' + (i / 10);
              }
              buf[pos++] = '0' + (i % 10);  // ones digit
              buf[pos++] = ',';            
            }
            if(pos > 0){
              buf[pos-1] = '\0';
            }
        }
        break;
        case 2: 
        {
            oled.clearToEOL();
            if(pos > 0){
              oled.print(buf);
            }
            oled.setRow(2); 
            oled.setCol(0);
        }
        break;
        case 3:
        { 
            fcount++;
            byte strober = min(20, abs((int8_t)61-(int8_t)out))/2; //abs distance to center, strobe more often when close to center
            bool centered = out >= 60 && out <= 62;
            oled.setLetterSpacing(0);
            oled.setInvertMode(centered || (strober < 10 && !(fcount % strober)));
            oled.print(F("-----------   ------------"));
            oled.setCol(61-5);
            if (centered){
              oled.print("( )");
            }else{
              oled.print(") (");
            }
            oled.setCol(out);
            oled.print('O');
            oled.setInvertMode(false);
        }
        break;
        case 4:
        {
            oled.setLetterSpacing(1);
            oled.setRow(3); oled.setCol(18);
            if(diffTimeAvg == 0){
              diffTimeAvg = diffTime;
            }else{
              diffTimeAvg = (diffTimeAvg + diffTime)/2.0;
              oled.print((uint16_t)(1000.0/diffTimeAvg));
              oled.print(empty);
            }
        }
        break;
    }
    showSensorsStep++;
    if (showSensorsStep > 4) showSensorsStep = 0;
}


void showSensors(int16_t diffTime, int16_t steeringPosition){
  
    const __FlashStringHelper* arrow = F(">");
    const __FlashStringHelper* empty = F("  ");

    // print forces
    oled.setRow(1); oled.setCol(25);
    oled.print(forces[0]);
    oled.print(empty);

    // steering pos
    oled.setCol(91);
    oled.print(steeringPosition);
    oled.print(empty);

    // button states
    oled.setRow(2); oled.setCol(25);
    oled.clearToEOL();

    char buf[32];
    uint8_t pos = 0;
    
    for (uint8_t i = 1; i <= NUMBUTTONS; i++) {
      if (button[i]) {
        // print tens digit only if >= 10
        if (i >= 10) {
          buf[pos++] = '0' + (i / 10);
        }
        buf[pos++] = '0' + (i % 10);  // ones digit
        buf[pos++] = ' ';            // space
      }
    }
    buf[pos] = '\0';
    oled.print(buf);

    oled.setRow(3); oled.setCol(16);
    oled.print(1000/diffTime);
    oled.print(empty);
}

static inline int16_t processPedal(
    int16_t value,
    int16_t minVal,
    int16_t maxVal,
    EasingMode ease
){
    if (value < minVal) value = minVal;
    if (value > maxVal) value = maxVal;

    switch (ease) {
        case EASEIN:
            return EaseIn(value, minVal, maxVal, joystickMin, joystickMax);

        case EASEOUT:
            return EaseOut(value, minVal, maxVal, joystickMin, joystickMax);

        default: // LINEAR
            return map(value, minVal, maxVal, joystickMin, joystickMax);
    }
}

void processBreakPedal()
{
    int16_t v = processPedal(
        brakeSensor,
        brakeSensorMin,
        brakeSensorMax,
        brakeEase
    );
    Joystick.setYAxis(v);
}

void processAccelleratorPedal()
{
    int16_t v = processPedal(
        acceleratorSensor,
        acceleratorSensorMin,
        acceleratorSensorMax,
        acceleratorEase
    );

    if (altMode)
        Joystick.setRzAxis(v);
    else
        Joystick.setZAxis(v);
}

int16_t clamp255(int16_t v)
{
    if (v > 255)  return 255;
    if (v < -255) return -255;
    return v;
}

// SEND DATA TO JOYSTICK
int16_t ProcessDataAndApply(unsigned long currentMillis)
{
  // STEERING
  //steeringPosition = mapLUT(steeringSensor);
  steeringPosition = mapLUT(applyDeadband(steeringSensor)); // maybe just smooth here?
  
  // Apply position
  Joystick.setXAxis(steeringPosition);

  // Update state/setEffectParams with steeringPosition
  int16_t diffTime = calculateEffectParams(currentMillis, mapLUT(steeringSensor));

  // GET FORCE FEEDBACK
  Joystick.getForce(forces);

  // 4. Limitar força total
  int16_t finalForce = finalForce = clamp255(forces[0]);

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
  for (i = 1; i <= NUMBUTTONS; i++){
     button[i] ? Joystick.pressButton(i-1) : Joystick.releaseButton(i-1);
  }

  return diffTime;
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
    uint16_t addr = 0;

    // Steering LUT Array (11 × uint16_t = 22 bytes)
    for (uint8_t i = 0; i < 11; i++, addr += 2) {
        eeprom_update_word((uint16_t*)addr, steeringSensorMapLUT[i]);
    }

    // Brake pedal
    eeprom_update_word((uint16_t*)addr, brakeSensorMin); addr += 2;
    eeprom_update_word((uint16_t*)addr, brakeSensorMax); addr += 2;

    // Accelerator pedal
    eeprom_update_word((uint16_t*)addr, acceleratorSensorMin); addr += 2;
    eeprom_update_word((uint16_t*)addr, acceleratorSensorMax); addr += 2;

    // Force feedback gain
    eeprom_update_word((uint16_t*)addr, forceGain); addr += 2;

    // Force feedback active
    eeprom_update_byte((uint8_t*)addr, forceActive); addr++;

    // Easing modes
    eeprom_update_byte((uint8_t*)addr, (uint8_t)brakeEase); addr++;
    eeprom_update_byte((uint8_t*)addr, (uint8_t)acceleratorEase); addr++;

    // Sensor data active
    eeprom_update_byte((uint8_t*)addr, showSensorsActive);
}

void ReadData()
{
    uint16_t addr = 0;

    // Steering LUT Array (11 × uint16_t = 22 bytes)
    for (uint8_t i = 0; i < 11; i++, addr += 2) {
        steeringSensorMapLUT[i] = eeprom_read_word((uint16_t*)addr);
    }

    // Brake pedal
    brakeSensorMin = eeprom_read_word((uint16_t*)addr); addr += 2;
    brakeSensorMax = eeprom_read_word((uint16_t*)addr); addr += 2;

    // Accelerator pedal
    acceleratorSensorMin = eeprom_read_word((uint16_t*)addr); addr += 2;
    acceleratorSensorMax = eeprom_read_word((uint16_t*)addr); addr += 2;

    // Force feedback gain
    forceGain = eeprom_read_word((uint16_t*)addr); addr += 2;

    // Force feedback active
    forceActive = eeprom_read_byte((uint8_t*)addr); addr++;

    // Easing modes
    brakeEase = static_cast<EasingMode>(eeprom_read_byte((uint8_t*)addr)); addr++;
    acceleratorEase = static_cast<EasingMode>(eeprom_read_byte((uint8_t*)addr)); addr++;

    // Sensor data active
    showSensorsActive = eeprom_read_byte((uint8_t*)addr);
}
