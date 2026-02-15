#pragma once
//
//    FILE: ADS1X15.h
//  AUTHOR: Rob Tillaart
// VERSION: 0.5.3
//    DATE: 2013-03-24
// PURPOSE: Arduino library for ADS1015 and ADS1115
//     URL: https://github.com/RobTillaart/ADS1X15
//


#include "Arduino.h"
#include "AceWire.h"

#define ADS1X15_LIB_VERSION               (F("0.5.3"))

//  allow compile time default address
//  address in { 0x48, 0x49, 0x4A, 0x4B }, no test...
#ifndef ADS1015_ADDRESS
#define ADS1015_ADDRESS                   ( 0x48 )
#endif

#ifndef ADS1115_ADDRESS
#define ADS1115_ADDRESS                   ( 0x48 )
#endif


//  ERROR CONSTANTS
#define ADS1X15_OK                        ( 0 )
#define ADS1X15_INVALID_VOLTAGE           ( -100 )
#define ADS1X15_ERROR_TIMEOUT             ( -101 )
#define ADS1X15_ERROR_I2C                 ( -102 )
#define ADS1X15_INVALID_GAIN              ( 0xFF )
#define ADS1X15_INVALID_MODE              ( 0xFE )


//  PARAMETER CONSTANTS (not used in the code)

//  PARAMETER setMode()
#define ADS1X15_MODE_CONTINUOUS           ( 0x00 )
#define ADS1X15_MODE_SINGLE               ( 0x01 )

//  PARAMETER setDataRate()
#define ADS1X15_DATARATE_0                ( 0x00 )
#define ADS1X15_DATARATE_1                ( 0x01 )
#define ADS1X15_DATARATE_2                ( 0x02 )
#define ADS1X15_DATARATE_3                ( 0x03 )
#define ADS1X15_DATARATE_4                ( 0x04 )
#define ADS1X15_DATARATE_5                ( 0x05 )
#define ADS1X15_DATARATE_6                ( 0x06 )
#define ADS1X15_DATARATE_7                ( 0x07 )

//  PARAMETER setGain()    read MV as  miliVolt
#define ADS1X15_GAIN_6144MV               ( 0x00 )
#define ADS1X15_GAIN_4096MV               ( 0x01 )
#define ADS1X15_GAIN_2048MV               ( 0x02 )
#define ADS1X15_GAIN_1024MV               ( 0x04 )
#define ADS1X15_GAIN_0512MV               ( 0x08 )
#define ADS1X15_GAIN_0256MV               ( 0x10 )

//  PARAMETER setComparatorMode()
#define ADS1x15_COMP_MODE_TRADITIONAL     ( 0x00 )
#define ADS1x15_COMP_MODE_WINDOW          ( 0x01 )

//  PARAMETER setComparatorPolarity()
#define ADS1x15_COMP_POL_FALLING_EDGE     ( 0x00 )
#define ADS1x15_COMP_POL_RISING_EDGE      ( 0x01 )

//  PARAMETER setComparatorLatch()
#define ADS1x15_COMP_POL_NOLATCH          ( 0x00 )
#define ADS1x15_COMP_POL_LATCH            ( 0x01 )

//  PARAMETER setComparatorQueConvert()
#define ADS1x15_COMP_QUE_CONV_TRIGGER_1   ( 0x00 )
#define ADS1x15_COMP_QUE_CONV_TRIGGER_2   ( 0x01 )
#define ADS1x15_COMP_QUE_CONV_TRIGGER_4   ( 0x02 )
#define ADS1x15_COMP_QUE_CONV_DISABLE     ( 0x03 )


//  GAIN TO VOLTAGE FULL SCALE (See #91)
//  used in getMaxVoltage()
#define ADS1x15_GAIN_6144MV_FSRANGE_V     ( 6.144 )
#define ADS1x15_GAIN_4096MV_FSRANGE_V     ( 4.096 )
#define ADS1x15_GAIN_2048MV_FSRANGE_V     ( 2.048 )
#define ADS1x15_GAIN_1024MV_FSRANGE_V     ( 1.024 )
#define ADS1x15_GAIN_0512MV_FSRANGE_V     ( 0.512 )
#define ADS1x15_GAIN_0256MV_FSRANGE_V     ( 0.256 )

template <typename T_WIREI>
class ADS1X15
{
public:
  void     reset();

  bool     begin();
  bool     isConnected();

  //           GAIN
  //  0  =  +- 6.144V  default
  //  1  =  +- 4.096V
  //  2  =  +- 2.048V
  //  4  =  +- 1.024V
  //  8  =  +- 0.512V
  //  16 =  +- 0.256V
  void     setGain(uint8_t gain = 0);    //  invalid values are mapped to 0 (default).
  uint8_t  getGain();                    //  0xFF == invalid gain error.

  //  both return ADS1X15_INVALID_VOLTAGE if the gain is invalid.
  float    toVoltage(float value = 1);   //  converts raw to voltage (can be an average!)
  float    getMaxVoltage();              //  returns voltage with current gain

  //  MODE
  //  0  =  CONTINUOUS
  //  1  =  SINGLE       default
  void     setMode(uint8_t mode = 1);    //  invalid values are mapped to 1 (default)
  uint8_t  getMode();                    //  0xFE == invalid mode error.

  //  DATARATE
  //  0  =  slowest
  //  7  =  fastest
  //  4  =  default
  void     setDataRate(uint8_t dataRate = 4);  //  invalid values are mapped on 4 (default)
  uint8_t  getDataRate();                      //  actual speed depends on device

  //  READ
  int16_t  readADC(uint8_t pin = 0);
  int16_t  readADC_Differential_0_1();

  //  used by continuous mode and async mode.
  //  [[deprecated("Use getValue() instead")]]
  //  int16_t  getLastValue() { return getValue(); };  //  will be obsolete in the future 0.4.0
  int16_t  getValue();


  //  ASYNC INTERFACE
  //  requestADC(pin) -> isBusy() or isReady() -> getValue();
  //  see examples
  void     requestADC(uint8_t pin = 0);
  void     requestADC_Differential_0_1();
  bool     isBusy();
  bool     isReady();

  //  returns a pin 0x0[0..3] or
  //          a differential "mode" 0x[pin second][pin first] or
  //          0xFF (no request / invalid request)
  uint8_t  lastRequest();


  //  COMPARATOR
  //  0    = TRADITIONAL   > high          => on      < low   => off
  //  else = WINDOW        > high or < low => on      between => off
  void     setComparatorMode(uint8_t mode);
  uint8_t  getComparatorMode();

  //  0    = LOW (default)
  //  else = HIGH
  void     setComparatorPolarity(uint8_t pol);
  uint8_t  getComparatorPolarity();

  //  0    = NON LATCH
  //  else = LATCH
  void     setComparatorLatch(uint8_t latch);
  uint8_t  getComparatorLatch();

  //  0   = trigger alert after 1 conversion
  //  1   = trigger alert after 2 conversions
  //  2   = trigger alert after 4 conversions
  //  3   = Disable comparator =  default, also for all other values.
  void     setComparatorQueConvert(uint8_t mode);
  uint8_t  getComparatorQueConvert();

  void     setComparatorThresholdLow(int16_t lo);
  int16_t  getComparatorThresholdLow();
  void     setComparatorThresholdHigh(int16_t hi);
  int16_t  getComparatorThresholdHigh();

  //  ERROR HANDLING
  int8_t   getError();


  //  EXPERIMENTAL
  //  see https://github.com/RobTillaart/ADS1X15/issues/22
  //      to be removed when next issue is solved
  //      https://github.com/arduino/Arduino/issues/11457
  void     setWireClock(uint32_t clockSpeed = 100000);
  //  prototype
  //  - getWireClock returns the value set by setWireClock
  //    not necessary the actual value as it can be overwritten
  //    by other code.
  uint32_t getWireClock();
  
  void     setConversionDelay(uint8_t delay);

  //  EXPERIMENTAL
  //  see https://github.com/RobTillaart/ADS1X15/issues/91
  //  returns the max raw value when reading.
  inline uint16_t getMaxRegValue()
  {
    return (_config & 0x04) ? 32767 : 2047;
  };


protected:
  ADS1X15();

  //  CONFIGURATION
  //  BIT   DESCRIPTION
  //  0     # channels        0 == 1    1 == 4;
  //  1     0
  //  2     # resolution      0 == 12   1 == 16
  //  3     0
  //  4     has gain          0 = NO    1 = YES
  //  5     has comparator    0 = NO    1 = YES
  //  6     0
  //  7     0
  uint8_t  _config;
  uint8_t  _maxPorts;
  uint8_t  _address;
  uint8_t  _conversionDelay;
  uint8_t  _bitShift;
  uint16_t _gain;
  uint16_t _mode;
  uint16_t _datarate;

  //  COMPARATOR variables
  //  TODO merge these into one COMPARATOR MASK?  (low priority)
  //       would speed up code in _requestADC() and save 3 bytes RAM.
  //  TODO boolean flags for first three, or make it mask value that
  //       can be or-ed.   (low priority)
  uint8_t  _compMode;
  uint8_t  _compPol;
  uint8_t  _compLatch;
  uint8_t  _compQueConvert;

  //  variable to track the last pin requested,
  //  to allow for round robin query of
  //  pins based on this state == if no last request then == 0xFFFF.
  uint16_t  _lastRequest;

  int16_t  _readADC(uint16_t readmode);
  void     _requestADC(uint16_t readmode);
  bool     _writeRegister(uint8_t address, uint8_t reg, uint16_t value);
  uint16_t _readRegister(uint8_t address, uint8_t reg);
  int8_t   _error = ADS1X15_OK;

  T_WIREI*  _wire;
  uint32_t  _clockSpeed = 0;
};


///////////////////////////////////////////////////////////////////////////
//
//  DERIVED CLASSES from ADS1X15
//
template <typename T_WIREI>
class ADS1013 : public ADS1X15<T_WIREI>
{
public:
  ADS1013(uint8_t Address = ADS1015_ADDRESS, T_WIREI *wire = nullptr);
  void setGain(uint8_t gain);
  uint8_t getGain();
};

template <typename T_WIREI>
class ADS1014 : public ADS1X15<T_WIREI>
{
public:
  ADS1014(uint8_t Address = ADS1015_ADDRESS, T_WIREI *wire = nullptr);
};

template <typename T_WIREI>
class ADS1015 : public ADS1X15<T_WIREI>
{
public:
  ADS1015(uint8_t Address = ADS1015_ADDRESS, T_WIREI *wire = nullptr);
  int16_t  readADC_Differential_0_3();
  int16_t  readADC_Differential_1_3();
  int16_t  readADC_Differential_2_3();
  int16_t  readADC_Differential_0_2();   //  not possible in async
  int16_t  readADC_Differential_1_2();   //  not possible in async
  void     requestADC_Differential_0_3();
  void     requestADC_Differential_1_3();
  void     requestADC_Differential_2_3();
};

template <typename T_WIREI>
class ADS1113 : public ADS1X15<T_WIREI>
{
public:
  ADS1113(uint8_t address = ADS1115_ADDRESS, T_WIREI *wire = nullptr);
  void setGain(uint8_t gain);
  uint8_t getGain();
};

template <typename T_WIREI>
class ADS1114 : public ADS1X15<T_WIREI>
{
public:
  ADS1114(uint8_t address = ADS1115_ADDRESS, T_WIREI *wire = nullptr);
};

template <typename T_WIREI>
class ADS1115 : public ADS1X15<T_WIREI>
{
public:
  ADS1115(uint8_t address = ADS1115_ADDRESS, T_WIREI *wire = nullptr);
  int16_t  readADC_Differential_0_3();
  int16_t  readADC_Differential_1_3();
  int16_t  readADC_Differential_2_3();
  int16_t  readADC_Differential_0_2();   //  not possible in async
  int16_t  readADC_Differential_1_2();   //  not possible in async
  void     requestADC_Differential_0_3();
  void     requestADC_Differential_1_3();
  void     requestADC_Differential_2_3();
};

//  -- END OF FILE --

// -- TEMPLATE IMPLEMENTATION --

//
//    FILE: (WAS PREVIOUSLY) ADS1X15.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.5.3
//    DATE: 2013-03-24
// PURPOSE: Arduino library for ADS1015 and ADS1115
//     URL: https://github.com/RobTillaart/ADS1X15
//
//	MODIFIED FOR ACEWIRE
//


#include "ADS1X15.h"


#define ADS1015_CONVERSION_DELAY    ( 1 )
#define ADS1115_CONVERSION_DELAY    ( 8 )


//  Kept #defines a bit in line with Adafruit library.

//  REGISTERS
#define ADS1X15_REG_CONVERT         ( 0x00 )
#define ADS1X15_REG_CONFIG          ( 0x01 )
#define ADS1X15_REG_LOW_THRESHOLD   ( 0x02 )
#define ADS1X15_REG_HIGH_THRESHOLD  ( 0x03 )


//  CONFIG REGISTER

//  BIT 15      Operational Status           //  1 << 15
#define ADS1X15_OS_BUSY             ( 0x0000 )
#define ADS1X15_OS_NOT_BUSY         ( 0x8000 )
#define ADS1X15_OS_START_SINGLE     ( 0x8000 )

//  BIT 12-14   read differential
#define ADS1X15_MUX_DIFF_0_1        ( 0x0000 )
#define ADS1X15_MUX_DIFF_0_3        ( 0x1000 )
#define ADS1X15_MUX_DIFF_1_3        ( 0x2000 )
#define ADS1X15_MUX_DIFF_2_3        ( 0x3000 )
//              read single
#define ADS1X15_READ_0              ( 0x4000 )   //  pin << 12
#define ADS1X15_READ_1              ( 0x5000 )   //  pin = 0..3
#define ADS1X15_READ_2              ( 0x6000 )
#define ADS1X15_READ_3              ( 0x7000 )


//  BIT 9-11    gain                         //  (0..5) << 9
#define ADS1X15_PGA_6_144V          ( 0x0000 )   //  voltage
#define ADS1X15_PGA_4_096V          ( 0x0200 )   //
#define ADS1X15_PGA_2_048V          ( 0x0400 )   //  default
#define ADS1X15_PGA_1_024V          ( 0x0600 )
#define ADS1X15_PGA_0_512V          ( 0x0800 )
#define ADS1X15_PGA_0_256V          ( 0x0A00 )

//  BIT 8       mode                         //  1 << 8
#define ADS1X15_MODE_CONTINUE       ( 0x0000 )
#define ADS1X15_MODE_ONCE           ( 0x0100 )

//  BIT 5-7     data rate sample per second  //  (0..7) << 5
/*
differs for different devices, check datasheet or readme.md

|  data rate  |  ADS101x  |  ADS111x  |   Notes   |
|:-----------:|----------:|----------:|:---------:|
|     0       |   128     |    8      |  slowest  |
|     1       |   250     |    16     |           |
|     2       |   490     |    32     |           |
|     3       |   920     |    64     |           |
|     4       |   1600    |    128    |  default  |
|     5       |   2400    |    250    |           |
|     6       |   3300    |    475    |           |
|     7       |   3300    |    860    |  fastest  |
*/

//  BIT 4 comparator modi                    //  1 << 4
#define ADS1X15_COMP_MODE_TRADITIONAL   ( 0x0000 )
#define ADS1X15_COMP_MODE_WINDOW        ( 0x0010 )

//  BIT 3 ALERT active value                 //  1 << 3
#define ADS1X15_COMP_POL_ACTIV_LOW      ( 0x0000 )
#define ADS1X15_COMP_POL_ACTIV_HIGH     ( 0x0008 )

//  BIT 2 ALERT latching                     //  1 << 2
#define ADS1X15_COMP_NON_LATCH          ( 0x0000 )
#define ADS1X15_COMP_LATCH              ( 0x0004 )

//  BIT 0-1 ALERT mode                       //  (0..3)
#define ADS1X15_COMP_QUE_1_CONV         ( 0x0000 )  //  trigger alert after 1 convert
#define ADS1X15_COMP_QUE_2_CONV         ( 0x0001 )  //  trigger alert after 2 converts
#define ADS1X15_COMP_QUE_4_CONV         ( 0x0002 )  //  trigger alert after 4 converts
#define ADS1X15_COMP_QUE_NONE           ( 0x0003 )  //  disable comparator


//  _CONFIG masks
//
//  |  bit  |  description           |
//  |:-----:|:-----------------------|
//  |   0   |  # channels            |
//  |   1   |  -                     |
//  |   2   |  resolution            |
//  |   3   |  -                     |
//  |   4   |  GAIN supported        |
//  |   5   |  COMPARATOR supported  |
//  |   6   |  -                     |
//  |   7   |  -                     |
//
#define ADS_CONF_CHAN_1                 ( 0x00 )
#define ADS_CONF_CHAN_4                 ( 0x01 )
#define ADS_CONF_RES_12                 ( 0x00 )
#define ADS_CONF_RES_16                 ( 0x04 )
#define ADS_CONF_NOGAIN                 ( 0x00 )
#define ADS_CONF_GAIN                   ( 0x10 )
#define ADS_CONF_NOCOMP                 ( 0x00 )
#define ADS_CONF_COMP                   ( 0x20 )


//////////////////////////////////////////////////////
//
//  BASE CONSTRUCTOR
//
template<typename T_WIREI>
ADS1X15<T_WIREI>::ADS1X15()
{
  reset();
}


//////////////////////////////////////////////////////
//
//  PUBLIC
//
template<typename T_WIREI>
void ADS1X15<T_WIREI>::reset()
{
  setGain(0);      //  _gain = ADS1X15_PGA_6_144V;
  setMode(1);      //  _mode = ADS1X15_MODE_ONCE;
  setDataRate(4);  //  middle speed, depends on device.

  //  COMPARATOR variables   #  see notes .h
  _compMode       = 0;
  _compPol        = 1;
  _compLatch      = 0;
  _compQueConvert = 3;
  _lastRequest    = 0xFFFF;  //  no request yet
}

template<typename T_WIREI>
bool ADS1X15<T_WIREI>::begin()
{
  if ((_address < 0x48) || (_address > 0x4B)) return false;
  if (! isConnected()) return false;
  return true;
}

template<typename T_WIREI>
bool ADS1X15<T_WIREI>::isConnected()
{
  _wire->beginTransmission(_address);
  return (_wire->endTransmission() == 0);
}

template<typename T_WIREI>
void ADS1X15<T_WIREI>::setGain(uint8_t gain)
{
  if (!(_config & ADS_CONF_GAIN)) gain = 0;
  switch (gain)
  {
    default:  //  catch invalid values and go for the safest gain.
    case 0:  _gain = ADS1X15_PGA_6_144V;  break;
    case 1:  _gain = ADS1X15_PGA_4_096V;  break;
    case 2:  _gain = ADS1X15_PGA_2_048V;  break;
    case 4:  _gain = ADS1X15_PGA_1_024V;  break;
    case 8:  _gain = ADS1X15_PGA_0_512V;  break;
    case 16: _gain = ADS1X15_PGA_0_256V;  break;
  }
}

template<typename T_WIREI>
uint8_t ADS1X15<T_WIREI>::getGain()
{
  if (!(_config & ADS_CONF_GAIN)) return 0;
  switch (_gain)
  {
    case ADS1X15_PGA_6_144V: return 0;
    case ADS1X15_PGA_4_096V: return 1;
    case ADS1X15_PGA_2_048V: return 2;
    case ADS1X15_PGA_1_024V: return 4;
    case ADS1X15_PGA_0_512V: return 8;
    case ADS1X15_PGA_0_256V: return 16;
  }
  _error = ADS1X15_INVALID_GAIN;
  return _error;
}

template<typename T_WIREI>
float ADS1X15<T_WIREI>::toVoltage(float value)
{
  if (value == 0) return 0;

  float volts = getMaxVoltage();
  if (volts < 0) return volts;    //  propagate error

  volts *= value;
  if (_config & ADS_CONF_RES_16)
  {
    //  value = 16 bits - sign bit = 15 bits mantissa
    volts *= 3.0518509475997E-5;  //  volts /= 32767;
  }
  else
  {
    //  value = 12 bits - sign bit = 11 bit mantissa
    volts *= 4.8851978505129E-4;  //  volts /= 2047;
  }
  return volts;
}

template<typename T_WIREI>
float ADS1X15<T_WIREI>::getMaxVoltage()
{
  switch (_gain)
  {
    case ADS1X15_PGA_6_144V: return ADS1x15_GAIN_6144MV_FSRANGE_V;
    case ADS1X15_PGA_4_096V: return ADS1x15_GAIN_4096MV_FSRANGE_V;
    case ADS1X15_PGA_2_048V: return ADS1x15_GAIN_2048MV_FSRANGE_V;
    case ADS1X15_PGA_1_024V: return ADS1x15_GAIN_1024MV_FSRANGE_V;
    case ADS1X15_PGA_0_512V: return ADS1x15_GAIN_0512MV_FSRANGE_V;
    case ADS1X15_PGA_0_256V: return ADS1x15_GAIN_0256MV_FSRANGE_V;
  }
  _error = ADS1X15_INVALID_VOLTAGE;
  return _error;
}

template<typename T_WIREI>
void ADS1X15<T_WIREI>::setMode(uint8_t mode)
{
  switch (mode)
  {
    case 0: _mode = ADS1X15_MODE_CONTINUE; break;
    default:  //  catch invalid modi
    case 1: _mode = ADS1X15_MODE_ONCE;   break;
  }
}

template<typename T_WIREI>
uint8_t ADS1X15<T_WIREI>::getMode(void)
{
  switch (_mode)
  {
    case ADS1X15_MODE_CONTINUE: return 0;
    case ADS1X15_MODE_ONCE:   return 1;
  }
  _error = ADS1X15_INVALID_MODE;
  return _error;
}

template<typename T_WIREI>
void ADS1X15<T_WIREI>::setDataRate(uint8_t dataRate)
{
  _datarate = dataRate;
  if (_datarate > 7) _datarate = 4;  //  default
  _datarate <<= 5;      //  convert 0..7 to mask needed.
}

template<typename T_WIREI>
uint8_t ADS1X15<T_WIREI>::getDataRate(void)
{
  return (_datarate >> 5) & 0x07;  //  convert mask back to 0..7
}

template<typename T_WIREI>
int16_t ADS1X15<T_WIREI>::readADC(uint8_t pin)
{
  if (pin >= _maxPorts) return 0;
  uint16_t mode = ((4 + pin) << 12);  //  pin to mask
  return _readADC(mode);
}

template<typename T_WIREI>
int16_t ADS1X15<T_WIREI>::readADC_Differential_0_1()
{
  return _readADC(ADS1X15_MUX_DIFF_0_1);
}

template<typename T_WIREI>
int16_t ADS1X15<T_WIREI>::getValue()
{
  int16_t raw = _readRegister(_address, ADS1X15_REG_CONVERT);
  if (_bitShift) raw >>= _bitShift;  //  Shift 12-bit results
  return raw;
}

template<typename T_WIREI>
void ADS1X15<T_WIREI>::requestADC(uint8_t pin)
{
  if (pin >= _maxPorts) return;
  uint16_t mode = ((4 + pin) << 12);   //  pin to mask
  _requestADC(mode);
}

template<typename T_WIREI>
void  ADS1X15<T_WIREI>::requestADC_Differential_0_1()
{
  _requestADC(ADS1X15_MUX_DIFF_0_1);
}

template<typename T_WIREI>
bool ADS1X15<T_WIREI>::isBusy()
{
  return isReady() == false;
}

template<typename T_WIREI>
bool ADS1X15<T_WIREI>::isReady()
{
  uint16_t val = _readRegister(_address, ADS1X15_REG_CONFIG);
  return ((val & ADS1X15_OS_NOT_BUSY) > 0);
}

template<typename T_WIREI>
uint8_t ADS1X15<T_WIREI>::lastRequest()
{
  switch (_lastRequest)
  {
    case ADS1X15_READ_0:       return 0x00;
    case ADS1X15_READ_1:       return 0x01;
    case ADS1X15_READ_2:       return 0x02;
    case ADS1X15_READ_3:       return 0x03;
    //  technically 0x01 -- but would collide with READ_1
    case ADS1X15_MUX_DIFF_0_1: return 0x10;
    case ADS1X15_MUX_DIFF_0_3: return 0x30;
    case ADS1X15_MUX_DIFF_1_3: return 0x31;
    case ADS1X15_MUX_DIFF_2_3: return 0x32;
  }
  return 0xFF;
}

template<typename T_WIREI>
void ADS1X15<T_WIREI>::setComparatorMode(uint8_t mode)
{
  _compMode = mode == 0 ? 0 : 1;
}

template<typename T_WIREI>
uint8_t ADS1X15<T_WIREI>::getComparatorMode()
{
  return _compMode;
}

template<typename T_WIREI>
void ADS1X15<T_WIREI>::setComparatorPolarity(uint8_t pol)
{
  _compPol = pol == 0 ? 0 : 1;
}

template<typename T_WIREI>
uint8_t ADS1X15<T_WIREI>::getComparatorPolarity()
{
  return _compPol;
}

template<typename T_WIREI>
void ADS1X15<T_WIREI>::setComparatorLatch(uint8_t latch)
{
  _compLatch = latch == 0 ? 0 : 1;
}

template<typename T_WIREI>
uint8_t ADS1X15<T_WIREI>::getComparatorLatch()
{
  return _compLatch;
}

template<typename T_WIREI>
void ADS1X15<T_WIREI>::setComparatorQueConvert(uint8_t mode)
{
  _compQueConvert = (mode < 3) ? mode : 3;
}

template<typename T_WIREI>
uint8_t ADS1X15<T_WIREI>::getComparatorQueConvert()
{
  return _compQueConvert;
}

template<typename T_WIREI>
void ADS1X15<T_WIREI>::setComparatorThresholdLow(int16_t lo)
{
  _writeRegister(_address, ADS1X15_REG_LOW_THRESHOLD, lo);
}

template<typename T_WIREI>
int16_t ADS1X15<T_WIREI>::getComparatorThresholdLow()
{
  return _readRegister(_address, ADS1X15_REG_LOW_THRESHOLD);
};

template<typename T_WIREI>
void ADS1X15<T_WIREI>::setComparatorThresholdHigh(int16_t hi)
{
  _writeRegister(_address, ADS1X15_REG_HIGH_THRESHOLD, hi);
};

template<typename T_WIREI>
int16_t ADS1X15<T_WIREI>::getComparatorThresholdHigh()
{
  return _readRegister(_address, ADS1X15_REG_HIGH_THRESHOLD);
};

template<typename T_WIREI>
int8_t ADS1X15<T_WIREI>::getError()
{
  int8_t rv = _error;
  _error = ADS1X15_OK;
  return rv;
}


//////////////////////////////////////////////////////
//
//  EXPERIMENTAL
//
template<typename T_WIREI>
void ADS1X15<T_WIREI>::setWireClock(uint32_t clockSpeed)
{
  _clockSpeed = clockSpeed;
  _wire->setClock(_clockSpeed);
}

//  see https://github.com/RobTillaart/ADS1X15/issues/22
//      https://github.com/arduino/Arduino/issues/11457
//  TODO: get the real clock speed from the I2C interface if possible.
template<typename T_WIREI>
uint32_t ADS1X15<T_WIREI>::getWireClock()
{
//  UNO 328 and
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  uint32_t speed = F_CPU / ((TWBR * 2) + 16);
  return speed;

#elif defined(ESP32)
  return (uint32_t) _wire->getClock();

//  #elif defined(ESP8266)
//  core_esp8266_si2c.cpp holds the data see => void Twi::setClock(
//  not supported.
//  return -1;

#else  //  best effort is remembering it
  return _clockSpeed;
#endif
}

template<typename T_WIREI>
void ADS1X15<T_WIREI>::setConversionDelay(uint8_t delay)
{
	this->_conversionDelay = delay;
}


//////////////////////////////////////////////////////
//
//  PROTECTED
//
template<typename T_WIREI>
int16_t ADS1X15<T_WIREI>::_readADC(uint16_t readmode)
{
  //  note readmode includes the channel
  _requestADC(readmode);

  if (_mode == ADS1X15_MODE_ONCE)
  {
    uint32_t start = millis();
    //  timeout == { 138, 74, 42, 26, 18, 14, 12, 11 }
    //  added 10 ms more than maximum conversion time from datasheet.
    //  to prevent premature timeout in RTOS context.
    //  See #82
    uint8_t timeOut = (128 >> (_datarate >> 5)) + 10;
    while (isBusy())
    {
      if ( (millis() - start) > timeOut)
      {
        _error = ADS1X15_ERROR_TIMEOUT;
        return ADS1X15_ERROR_TIMEOUT;
      }
      yield();   //  wait for conversion; yield for ESP.
    }
  }
  else
  {
    //  needed in continuous mode too, otherwise one get an old value.
    delay(_conversionDelay);
  }
  return getValue();
}

template<typename T_WIREI>
void ADS1X15<T_WIREI>::_requestADC(uint16_t readmode)
{
  //  write to register is needed in continuous mode as other flags can be changed
  uint16_t config = ADS1X15_OS_START_SINGLE;  //  bit 15     force wake up if needed
  config |= readmode;                         //  bit 12-14
  config |= _gain;                            //  bit 9-11
  config |= _mode;                            //  bit 8
  config |= _datarate;                        //  bit 5-7
  if (_compMode)  config |= ADS1X15_COMP_MODE_WINDOW;         //  bit 4      comparator modi
  else            config |= ADS1X15_COMP_MODE_TRADITIONAL;
  if (_compPol)   config |= ADS1X15_COMP_POL_ACTIV_HIGH;      //  bit 3      ALERT active value
  else            config |= ADS1X15_COMP_POL_ACTIV_LOW;
  if (_compLatch) config |= ADS1X15_COMP_LATCH;
  else            config |= ADS1X15_COMP_NON_LATCH;           //  bit 2      ALERT latching
  config |= _compQueConvert;                                  //  bit 0..1   ALERT mode
  _writeRegister(_address, ADS1X15_REG_CONFIG, config);

  //  remember last request type.
  _lastRequest = readmode;
}

template<typename T_WIREI>
bool ADS1X15<T_WIREI>::_writeRegister(uint8_t address, uint8_t reg, uint16_t value)
{
	_wire->sendHsMasterCode();
  _wire->beginTransmission(address);
  _wire->write((uint8_t)reg);
  _wire->write((uint8_t)(value >> 8));
  _wire->write((uint8_t)(value & 0xFF));
  int rv = _wire->endTransmission();
  if (rv != 0)
  {
    _error =  ADS1X15_ERROR_I2C;
    return false;
  }
  return true;
}

template<typename T_WIREI>
uint16_t ADS1X15<T_WIREI>::_readRegister(uint8_t address, uint8_t reg)
{
	_wire->sendHsMasterCode();
  _wire->beginTransmission(address);
  _wire->write(reg);
  int rv = _wire->endTransmission();
  if (rv == 0)
  {
    rv = _wire->requestFrom((int) address, (int) 2);
    if (rv == 2)
    {
      uint16_t value = _wire->read() << 8;
      value += _wire->read();
      return value;
    }
  }
  _error =  ADS1X15_ERROR_I2C;
  return 0x0000;
}



///////////////////////////////////////////////////////////////////////////
//
//  DERIVED CLASSES
//


///////////////////////////////////////////////////////////////////////////
//
//  ADS1013
//
template<typename T_WIREI>
ADS1013<T_WIREI>::ADS1013(uint8_t address, T_WIREI *wire)
{
  this->_address = address;
  this->_wire = wire;
  this->_config = ADS_CONF_NOCOMP | ADS_CONF_NOGAIN | ADS_CONF_RES_12 | ADS_CONF_CHAN_1;
  this->_conversionDelay = ADS1015_CONVERSION_DELAY;
  this->_bitShift = 4;
  this->_maxPorts = 1;
  this->_gain     = ADS1X15_PGA_2_048V;  //  fixed value
}


//  ADS1x13 has no gain so set default.
//  Table 8. Config Register Field Descriptions
template<typename T_WIREI>
void ADS1013<T_WIREI>::setGain(uint8_t gain)
{
  this->_gain = gain;  //  keep compiler happy.
  this->_gain = ADS1X15_PGA_2_048V;  //  fixed value
}

template<typename T_WIREI>
uint8_t ADS1013<T_WIREI>::getGain()
{
  return 2;  //  fixed value
}


///////////////////////////////////////////////////////////////////////////
//
//  ADS1014
//
template<typename T_WIREI>
ADS1014<T_WIREI>::ADS1014(uint8_t address, T_WIREI *wire)
{
  this->_address = address;
  this->_wire = wire;
  this->_config = ADS_CONF_COMP | ADS_CONF_GAIN | ADS_CONF_RES_12 | ADS_CONF_CHAN_1;
  this->_conversionDelay = ADS1015_CONVERSION_DELAY;
  this->_bitShift = 4;
  this->_maxPorts = 1;
}


///////////////////////////////////////////////////////////////////////////
//
//  ADS1015
//
template<typename T_WIREI>
ADS1015<T_WIREI>::ADS1015(uint8_t address, T_WIREI *wire)
{
  this->_address = address;
  this->_wire = wire;
  this->_config = ADS_CONF_COMP | ADS_CONF_GAIN | ADS_CONF_RES_12 | ADS_CONF_CHAN_4;
  this->_conversionDelay = ADS1015_CONVERSION_DELAY;
  this->_bitShift = 4;
  this->_maxPorts = 4;
}

template<typename T_WIREI>
int16_t ADS1015<T_WIREI>::readADC_Differential_0_3()
{
  return this->_readADC(ADS1X15_MUX_DIFF_0_3);
}

template<typename T_WIREI>
int16_t ADS1015<T_WIREI>::readADC_Differential_1_3()
{
  return this->_readADC(ADS1X15_MUX_DIFF_1_3);
}

template<typename T_WIREI>
int16_t ADS1015<T_WIREI>::readADC_Differential_2_3()
{
  return this->_readADC(ADS1X15_MUX_DIFF_2_3);
}

template<typename T_WIREI>
int16_t ADS1015<T_WIREI>::readADC_Differential_0_2()
{
  return this->readADC(2) - this->readADC(0);
}

template<typename T_WIREI>
int16_t ADS1015<T_WIREI>::readADC_Differential_1_2()
{
  return this->readADC(2) - this->readADC(1);;
}

template<typename T_WIREI>
void ADS1015<T_WIREI>::requestADC_Differential_0_3()
{
  this->_requestADC(ADS1X15_MUX_DIFF_0_3);
}

template<typename T_WIREI>
void ADS1015<T_WIREI>::requestADC_Differential_1_3()
{
  this->_requestADC(ADS1X15_MUX_DIFF_1_3);
}

template<typename T_WIREI>
void ADS1015<T_WIREI>::requestADC_Differential_2_3()
{
  this->_requestADC(ADS1X15_MUX_DIFF_2_3);
}


///////////////////////////////////////////////////////////////////////////
//
//  ADS1113
//
template<typename T_WIREI>
ADS1113<T_WIREI>::ADS1113(uint8_t address, T_WIREI *wire)
{
  this->_address = address;
  this->_wire = wire;
  this->_config = ADS_CONF_NOCOMP | ADS_CONF_NOGAIN | ADS_CONF_RES_16 | ADS_CONF_CHAN_1;
  this->_conversionDelay = ADS1115_CONVERSION_DELAY;
  this->_bitShift = 0;
  this->_maxPorts = 1;
  this->_gain     = ADS1X15_PGA_2_048V;  //  fixed value
}


//  ADS1x13 has no gain so set default.
//  Table 8. Config Register Field Descriptions
template<typename T_WIREI>
void ADS1113<T_WIREI>::setGain(uint8_t gain)
{
  this->_gain = gain;  //  keep compiler happy.
  this->_gain = ADS1X15_PGA_2_048V;  //  fixed value
}

template<typename T_WIREI>
uint8_t ADS1113<T_WIREI>::getGain()
{
  return 2;  //  fixed value
}


///////////////////////////////////////////////////////////////////////////
//
//  ADS1114
//
template<typename T_WIREI>
ADS1114<T_WIREI>::ADS1114(uint8_t address, T_WIREI *wire)
{
  this->_address = address;
  this->_wire = wire;
  this->_config = ADS_CONF_COMP | ADS_CONF_GAIN | ADS_CONF_RES_16 | ADS_CONF_CHAN_1;
  this->_conversionDelay = ADS1115_CONVERSION_DELAY;
  this->_bitShift = 0;
  this->_maxPorts = 1;
}


///////////////////////////////////////////////////////////////////////////
//
//  ADS1115
//
template<typename T_WIREI>
ADS1115<T_WIREI>::ADS1115(uint8_t address, T_WIREI *wire)
{
  this->_address = address;
  this->_wire = wire;
  this->_config = ADS_CONF_COMP | ADS_CONF_GAIN | ADS_CONF_RES_16 | ADS_CONF_CHAN_4;
  this->_conversionDelay = ADS1115_CONVERSION_DELAY;
  this->_bitShift = 0;
  this->_maxPorts = 4;
}

template<typename T_WIREI>
int16_t ADS1115<T_WIREI>::readADC_Differential_0_3()
{
  return this->_readADC(ADS1X15_MUX_DIFF_0_3);
}

template<typename T_WIREI>
int16_t ADS1115<T_WIREI>::readADC_Differential_1_3()
{
  return this->_readADC(ADS1X15_MUX_DIFF_1_3);
}

template<typename T_WIREI>
int16_t ADS1115<T_WIREI>::readADC_Differential_2_3()
{
  return this->_readADC(ADS1X15_MUX_DIFF_2_3);
}

template<typename T_WIREI>
int16_t ADS1115<T_WIREI>::readADC_Differential_0_2()
{
  return this->readADC(2) - this->readADC(0);
}

template<typename T_WIREI>
int16_t ADS1115<T_WIREI>::readADC_Differential_1_2()
{
  return this->readADC(2) - this->readADC(1);;
}

template<typename T_WIREI>
void ADS1115<T_WIREI>::requestADC_Differential_0_3()
{
  this->_requestADC(ADS1X15_MUX_DIFF_0_3);
}

template<typename T_WIREI>
void ADS1115<T_WIREI>::requestADC_Differential_1_3()
{
  this->_requestADC(ADS1X15_MUX_DIFF_1_3);
}

template<typename T_WIREI>
void ADS1115<T_WIREI>::requestADC_Differential_2_3()
{
  this->_requestADC(ADS1X15_MUX_DIFF_2_3);
}

//  -- END OF FILE --

