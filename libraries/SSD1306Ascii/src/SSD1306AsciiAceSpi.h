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
 * @file SSD1306AsciiAceSpi.h
 * @brief Class for hardware SPI displays using AceSPI
 * (https://github.com/bxparks/AceSPI).
 */
#ifndef SSD1306AsciiAceSpi_h
#define SSD1306AsciiAceSpi_h
#include "SSD1306Ascii.h"
//------------------------------------------------------------------------------
/**
 * @class SSD1306AsciiAceSpi
 * @brief Class for SPI displays on the hardware SPI bus.
 */
template <typename T_SPII>
class SSD1306AsciiAceSpi : public SSD1306Ascii {
 public:
  /**
   * @brief Initialize object on specific SPI bus.
   *
   * @param[in] bus The SPI bus to be used.
   */
  explicit SSD1306AsciiAceSpi(T_SPII& spi) : m_oledSpi(spi) {}
  /**
   * @brief Initialize the display controller.
   *
   * @param[in] dev A device initialization structure.
   * @param[in] cs The display controller chip select pin.
   * @param[in] dc The display controller data/command pin.
   */
  void begin(const DevType* dev, uint8_t cs, uint8_t dc) {
    m_cs = cs;
    m_dc = dc;
    pinMode(m_cs, OUTPUT);
    pinMode(m_dc, OUTPUT);
    init(dev);
  }
  /**
   * @brief Initialize the display controller.
   *
   * @param[in] dev A device initialization structure.
   * @param[in] cs The display controller chip select pin.
   * @param[in] dc The display controller cdata/command pin.
   * @param[in] rst The display controller reset pin.
   */
  void begin(const DevType* dev, uint8_t cs, uint8_t dc, uint8_t rst) {
    oledReset(rst);
    begin(dev, cs, dc);
  }

 protected:
  void writeDisplay(uint8_t b, uint8_t mode) {
    digitalWrite(m_dc, mode != SSD1306_MODE_CMD);
    m_oledSpi.beginTransaction();
    m_oledSpi.send8(b);
    m_oledSpi.endTransaction();
  }

  int8_t m_cs;
  int8_t m_dc;
  T_SPII m_oledSpi;
};
#endif  // SSD1306AsciiSpi_h
