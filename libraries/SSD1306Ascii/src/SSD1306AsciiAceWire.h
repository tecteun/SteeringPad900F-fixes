/* Arduino SSD1306Ascii Library
 * Copyright (C) 2015 by William Greiman
 * Copyright (C) 2022 by Brian Park
 *
 * This file is part of the Arduino SSD1306Ascii Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino SSD1306Ascii Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file SSD1306AsciiAceWire.h
 * @brief Class for I2C displays using AceWire
 * (https://github.com/bxparks/AceWire)
 */
#ifndef SSD1306AsciiAceWire_h
#define SSD1306AsciiAceWire_h
#include "SSD1306Ascii.h"
/**
 * @class SSD1306AsciiAceWire
 * @brief Class for I2C displays using AceWire.
 */
template <typename T_WIREI>
class SSD1306AsciiAceWire : public SSD1306Ascii {
 public:
  /**
   * @brief Initialize object on specific I2C bus.
   *
   * @param[in] bus The I2C bus to be used.
   */
  explicit SSD1306AsciiAceWire(T_WIREI& bus) : m_oledWire(bus) {}
  /**
   * @brief Initialize the display controller.
   *
   * @param[in] dev A device initialization structure.
   * @param[in] i2cAddr The I2C address of the display controller.
   */
  void begin(const DevType* dev, uint8_t i2cAddr) {
    m_i2cAddr = i2cAddr;
    init(dev);
  }
  /**
   * @brief Initialize the display controller.
   *
   * @param[in] dev A device initialization structure.
   * @param[in] i2cAddr The I2C address of the display controller.
   * @param[in] rst The display controller reset pin.
   */
  void begin(const DevType* dev, uint8_t i2cAddr, uint8_t rst) {
    oledReset(rst);
    begin(dev, i2cAddr);
  }

 protected:
  void writeDisplay(uint8_t b, uint8_t mode) override {
#if OPTIMIZE_I2C
    if (m_nData > 16 || (m_nData && mode == SSD1306_MODE_CMD)) {
      m_oledWire.endTransmission();
      m_nData = 0;
    }
    if (m_nData == 0) {
      m_oledWire.beginTransmission(m_i2cAddr);
      m_oledWire.write(mode == SSD1306_MODE_CMD ? 0X00 : 0X40);
    }
    m_oledWire.write(b);
    if (mode == SSD1306_MODE_RAM_BUF) {
      m_nData++;
    } else {
      m_oledWire.endTransmission();
      m_nData = 0;
    }
#else  // OPTIMIZE_I2C
    m_oledWire.beginTransmission(m_i2cAddr);
    m_oledWire.write(mode == SSD1306_MODE_CMD ? 0X00: 0X40);
    m_oledWire.write(b);
    m_oledWire.endTransmission();
#endif    // OPTIMIZE_I2C
  }

 protected:
  T_WIREI m_oledWire;
  uint8_t m_i2cAddr;
#if OPTIMIZE_I2C
  uint8_t m_nData;
#endif  // OPTIMIZE_I2C
};
#endif  // SSD1306AsciiAceWire_h
