//
//    FILE: ADS_continuous_3_channel.ino
//  AUTHOR: Rob.Tillaart
// PURPOSE: read multiple analog inputs continuously
//          interrupt driven to catch all conversions.
//     URL: https://github.com/RobTillaart/ADS1X15
//          https://github.com/RobTillaart/ADS1X15/issues/49
//
//  experimental, not tested extensively

//  test
//  connect multiple potmeters
//
//  RDY ----------------- pin 2 (for IRQ, adjust if needed)
//
//  GND ---[   x   ]------ 5V
//             |
//             |
//           ADS(n)
//
//  measure at x  - connect to AIN0..1.
//
//  for the test it is an option to have AIN2 connected to 5V and AIN3 to GND
//  so one can see these as references in the output.
//
//  has an issue with the index of the channels. not not investigated yet.
#include <util/atomic.h> // this library includes the ATOMIC_BLOCK macro.

//ACEWIRE
const uint8_t DELAY_MICROS = 0;
#include <AceWire.h>
#include <DigitalWriteFast.h>
#include <ace_wire/SimpleWireFastInterface.h>
using ace_wire::SimpleWireFastInterface;

// ADC
using WireInterface = SimpleWireFastInterface<SDA, SCL, DELAY_MICROS>;
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
#define DISPLAY_WIDTH = 16;


volatile bool pause = false;
volatile bool isbusy = false;

volatile uint8_t channel = 0;
volatile int16_t val[4] = { 0, 0, 0, 0 };

int SPS = 0;
uint32_t lastTime = 0;

// JOYSTICK
#include <Joystick.h>

Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,
  17,     // Buttons (Max 32)
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

const int16_t joystickMin = -32767;
const int16_t joystickMax = 32767;
void setup()
{
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont(Stang5x7);
  oled.println("---startaaap---");
  
  Serial.begin(230400);       //  <<<<<<<<<  fast!
  Serial.println();
  Serial.println(__FILE__);
  Serial.print("ADS1X15_LIB_VERSION: ");
  Serial.println(ADS1X15_LIB_VERSION);
  Serial.println();


  ADS.begin();
  ADS.setGain(1);        //  6.144 volt
  ADS.setDataRate(7);    //  0 = slow   4 = medium   7 = fast

  //  SET ALERT RDY PIN
  ADS.setComparatorThresholdHigh(0x8000);
  ADS.setComparatorThresholdLow(0x0000);
  ADS.setComparatorQueConvert(ADS1x15_COMP_QUE_CONV_TRIGGER_1);

  //  SET INTERRUPT HANDLER TO CATCH CONVERSION READY
  pinMode(0, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(0), adsReady, RISING);

  ADS.setMode(0);            //  continuous mode
  channel = 0;
  ADS.requestADC(channel);   //  start at 0

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

    Joystick.begin(true);
  Joystick.setXAxisRange(joystickMin, joystickMax);
  Joystick.setYAxisRange(joystickMin, joystickMax);
  Joystick.setZAxisRange(joystickMin, joystickMax);
  
}

void adsReady()
{
  if(!pause){
    byte lastRequest = ADS.lastRequest();
    val[lastRequest] = ADS.getValue();
    lastRequest++;
    ADS.requestADC(lastRequest > 2 ? 0 : lastRequest);
  }
}
unsigned long last = 0;
void loop() {

  
  //time for interrupt:
  //would be other processing
  float now = millis();
  if(true){//(now - last > 100){
    last = now;
    if(true){
      for (int i = 0; i < 4; i++)
      {  
        Serial.print('\t');
        Serial.print(val[i]);
      }
      Serial.println();
    }
    pause = true;
    //this interferes with ads1115
    //delay(100);
    oled.clear();
    for (int i = 0; i < 4; i++)
    {  
      oled.print(val[i]);
      oled.print("-");
    }
    
    pause = false;
  }
}

/*
void loop()
{
  if(false){
    for (int i = 0; i < 4; i++)
    {  
    cli();
      Serial.print('\t');
      Serial.print(val[i]);
      sei();
       //       oled.print(val[i]);
      //oled.print("-");
    }
    Serial.println();
    
    //joySerial.println();
    Joystick.setXAxis(val[0]);
    Joystick.setYAxis(val[1]);
    Joystick.setZAxis(val[2]);
  }


  //  print the SPS
  if (millis() - lastTime >= 1000)
  {
    lastTime = millis();
    //Serial.print("SPS: ");
    //Serial.println(SPS);
    SPS = 0;
  }

 ADS.setComparatorQueConvert(ADS1x15_COMP_QUE_CONV_DISABLE);

  //detachInterrupt(digitalPinToInterrupt(0));
  int count = 0;
  while(isbusy){ delayMicroseconds(1); count++; }

  //pretend work thats not interruptable
  if(true){
    //cli();
    //delay(100);
    //sei();
    //delay(100);
    for (int i = 0; i < 4; i++)
    {  
      Serial.print('\t');
      Serial.print(val[i]);
      
    }
    Serial.println();
    
    oled.clear();
    for (int i = 0; i < 4; i++)
    {  
      oled.print(val[i]);
      oled.print("-");
    }

    
  }
   ADS.setComparatorQueConvert(ADS1x15_COMP_QUE_CONV_TRIGGER_1);
   Serial.println(ADS.lastRequest());
   ADS.requestADC(ADS.lastRequest());

}
//  interrupt service routine
//  kept as minimal as possible

void adsReady()
{
  isbusy = true;
  val[ADS.lastRequest()] = ADS.getValue();
  ADS.requestADC(ADS.lastRequest() + 1);
  isbusy = false;
}
*/

// TIMER3 INTERRUPTION
ISR(TIMER3_COMPA_vect)
{
  Joystick.getUSBPID();  // update FFB
}


void detached(){
  isbusy = false;
  channel = ADS.lastRequest();
}



//  -- END OF FILE --
