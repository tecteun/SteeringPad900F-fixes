/*
  LICENSE

  Project Name: Steering PAD 900-F Board 3 Test v2
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
#include "avdweb_AnalogReadFast.h"
const int thresholds[7] = 
{
  236, // button 3
  395, // button 5
  493, // button 6
  178, // buttons 3 & 5
  193, // buttons 3 & 6
  284, // buttons 5 & 6
  153  // buttons 3 & 5 & 6
}; // 

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

// DISPLAY
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAceWire.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiAceWire<WireInterface> oled(wireInterface);

#define motorPinDirection 8
#define motorPinPWM 9
#define motorPinEnable 10

int16_t steeringSensor;
int16_t breakSensor;
int16_t acceleratorSensor;

const int errorMargin = 5;
int previousValues[4] = {-1, -1, -1, -1};
bool prevDigitalStates[2] = {false, false};





void setup() {

// Set ADC prescaler to 16 → 1 MHz ADC clock
ADCSRA = (ADCSRA & 0b11111000) | 0x04;


  Serial.begin(9600);

  // ANALOG PINS
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);

  // DIGITAL PINS
  pinMode(4, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont(Stang5x7);
  oled.setInvertMode(false);

  // START ADC ///////////////////////////////////////////////////
  ADS.setMode(0);      // 0:continuous 1:single
  ADS.setGain(1);      // 0:6.144V  1:4.096V 2:2.048V 4:1.024V 8:0.512V 16:0.256V
  ADS.setDataRate(7);  // 0:slowest 4:default 7:fastest
  ADS.begin();

  pinMode(motorPinDirection, OUTPUT);
  pinMode(motorPinPWM, OUTPUT);
  pinMode(motorPinEnable, OUTPUT);

}

void ReadAnalogSensors() {
  steeringSensor = ADS.readADC(0);
  breakSensor = ADS.readADC(1);
  acceleratorSensor = ADS.readADC(2);
}

void checkButtonCombination(int analogValue, int btn1, int btn2, int btn3, bool* buttons) {
  if (abs(analogValue - thresholds[0]) <= errorMargin) buttons[btn1] = true;
  if (abs(analogValue - thresholds[1]) <= errorMargin) buttons[btn2] = true;
  if (abs(analogValue - thresholds[2]) <= errorMargin) buttons[btn3] = true;
  if (abs(analogValue - thresholds[3]) <= errorMargin) { buttons[btn1] = true; buttons[btn2] = true; }
  if (abs(analogValue - thresholds[4]) <= errorMargin) { buttons[btn1] = true; buttons[btn3] = true; }
  if (abs(analogValue - thresholds[5]) <= errorMargin) { buttons[btn2] = true; buttons[btn3] = true; }
  if (abs(analogValue - thresholds[6]) <= errorMargin) { buttons[btn1] = true; buttons[btn2] = true; buttons[btn3] = true; }
}

void updateButtonsDisplay(bool* buttons) {
  oled.setRow(0); oled.setCol(0);
  for (int i = 0; i < 13; i++) {

    String buttonState = buttons[i] ? "1" : "0";
    oled.print(buttonState);
    Serial.print(buttonState);
  }
  oled.print("               ");
  Serial.println();
}

void updateAnalogDisplay(int* sensorValues) {
  oled.setRow(1); oled.setCol(0);
  oled.print(sensorValues[3]);
  oled.print("               ");
}

void updateSensorsDisplay() {
  oled.setRow(3); oled.setCol(0);
  oled.print(steeringSensor);
  oled.print(" ");
  oled.print(breakSensor);
  oled.print(" ");
  oled.print(acceleratorSensor);
  oled.print("               ");
}

void loop() {
  ReadAnalogSensors();

  int sensorValues[4] = {analogReadFast(A0), analogReadFast(A1), analogReadFast(A2), analogReadFast(A3)};
  bool buttons[13] = {false};
  bool hasChanged = false;

  bool currentDigital1 = !digitalRead(4);
  bool currentDigital2 = !digitalRead(7);

  if (currentDigital1 != prevDigitalStates[0] || currentDigital2 != prevDigitalStates[1]) {
    hasChanged = true;
    prevDigitalStates[0] = currentDigital1;
    prevDigitalStates[1] = currentDigital2;
  }

  buttons[0] = currentDigital1;
  buttons[1] = currentDigital2;

  for (int i = 0; i < 4; i++) {
    if (abs(sensorValues[i] - previousValues[i]) > errorMargin) {
      hasChanged = true;
      previousValues[i] = sensorValues[i];
    }
  }

  if (hasChanged) {
    checkButtonCombination(sensorValues[3], 2, 4, 5, buttons);
    checkButtonCombination(sensorValues[2], 3, 6, 7, buttons);
    checkButtonCombination(sensorValues[1], 8, 9, 10, buttons);

    if (abs(sensorValues[0] - thresholds[1]) <= errorMargin) buttons[11] = true;
    if (abs(sensorValues[0] - thresholds[2]) <= errorMargin) buttons[12] = true;
    if (abs(sensorValues[0] - thresholds[5]) <= errorMargin) {
      buttons[11] = true;
      buttons[12] = true;
    }

    updateButtonsDisplay(buttons);
  }

  updateAnalogDisplay(sensorValues);
  updateSensorsDisplay();


  if (currentDigital1 == true && currentDigital2 == false)
  {
    digitalWrite(motorPinEnable, HIGH);
    digitalWrite(motorPinDirection, HIGH);
    analogWrite(motorPinPWM, 128);
  }
  if (currentDigital1 == false && currentDigital2 == true)
  {
    digitalWrite(motorPinEnable, HIGH);
    digitalWrite(motorPinDirection, LOW);
    analogWrite(motorPinPWM, 128);
  }

  if (currentDigital1 == true && currentDigital2 == true)
  {
    digitalWrite(motorPinEnable, LOW);
    digitalWrite(motorPinDirection, LOW);
    analogWrite(motorPinPWM, 0);
  }

    if (currentDigital1 == false && currentDigital2 == false)
  {
    digitalWrite(motorPinEnable, LOW);
    digitalWrite(motorPinDirection, LOW);
    analogWrite(motorPinPWM, 0);
  }

  //SERIAL PRINT
  for (int i = 0; i < 13; i++) {
    String buttonState = buttons[i] ? "1" : "0";
    Serial.print(buttonState);
  }
  Serial.print("     Button combination: ");
  Serial.print(sensorValues[3]);

  Serial.print("     Steering: ");
  Serial.print(steeringSensor);

  Serial.print("     Brake: ");
  Serial.print(breakSensor);

  Serial.print("     Accelerator: ");
  Serial.print(acceleratorSensor);

  Serial.println();

  delay(10);
}
