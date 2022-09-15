/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#ifndef SRC_BMP3_H_
#define SRC_BMP3_H_

#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
#else
#include "core/core.h"
#endif
#include <stddef.h>
#include <stdint.h>
#include "bst/bmp3.h"

namespace bfs {

class Bmp3 {
 public:
  enum I2cAddr : uint8_t {
    I2C_ADDR_PRIM = BMP3_ADDR_I2C_PRIM,
    I2C_ADDR_SEC = BMP3_ADDR_I2C_SEC
  };
  enum Oversampling : uint8_t {
    OVERSAMPLING_1X = BMP3_NO_OVERSAMPLING,
    OVERSAMPLING_2X = BMP3_OVERSAMPLING_2X,
    OVERSAMPLING_4X = BMP3_OVERSAMPLING_4X,
    OVERSAMPLING_8X = BMP3_OVERSAMPLING_8X,
    OVERSAMPLING_16X = BMP3_OVERSAMPLING_16X,
    OVERSAMPLING_32X = BMP3_OVERSAMPLING_32X
  };
  // enum OversamplingMode : uint8_t {
  //   OS_MODE_ULTRA_LOW_POWER = BMP2_OS_MODE_ULTRA_LOW_POWER,
  //   OS_MODE_LOW_POWER = BMP2_OS_MODE_LOW_POWER,
  //   OS_MODE_STANDARD_RESOLUTION = BMP2_OS_MODE_STANDARD_RESOLUTION,
  //   OS_MODE_HIGH_RESOLUTION = BMP2_OS_MODE_HIGH_RESOLUTION,
  //   OS_MODE_ULTRA_HIGH_RESOLUTION = BMP2_OS_MODE_ULTRA_HIGH_RESOLUTION
  // };
  enum FilterCoef : uint8_t {
    FILTER_COEF_OFF = BMP3_IIR_FILTER_DISABLE,
    FILTER_COEF_2 = BMP3_IIR_FILTER_COEFF_1,
    FILTER_COEF_4 = BMP3_IIR_FILTER_COEFF_3,
    FILTER_COEF_8 = BMP3_IIR_FILTER_COEFF_7,
    FILTER_COEF_16 = BMP3_IIR_FILTER_COEFF_15,
    FILTER_COEF_32 = BMP3_IIR_FILTER_COEFF_31,
    FILTER_COEF_64 = BMP3_IIR_FILTER_COEFF_63,
    FILTER_COEF_128 = BMP3_IIR_FILTER_COEFF_127,
  };
  enum Odr : uint8_t {
    ODR_200_HZ = BMP3_ODR_200_HZ,
    ODR_100_HZ = BMP3_ODR_100_HZ,
    ODR_50_HZ = BMP3_ODR_50_HZ,
    ODR_25_HZ = BMP3_ODR_25_HZ,
    ODR_12_5_HZ = BMP3_ODR_12_5_HZ,
    ODR_6_25_HZ = BMP3_ODR_6_25_HZ,
    ODR_3_1_HZ = BMP3_ODR_3_1_HZ,
    ODR_1_5_HZ = BMP3_ODR_1_5_HZ,
    ODR_0_78_HZ = BMP3_ODR_0_78_HZ,
    ODR_0_39_HZ = BMP3_ODR_0_39_HZ,
    ODR_0_2_HZ = BMP3_ODR_0_2_HZ,
    ODR_0_1_HZ = BMP3_ODR_0_1_HZ,
    ODR_0_05_HZ = BMP3_ODR_0_05_HZ,
    ODR_0_02_HZ = BMP3_ODR_0_02_HZ,
    ODR_0_01_HZ = BMP3_ODR_0_01_HZ,
    ODR_0_006_HZ = BMP3_ODR_0_006_HZ,
    ODR_0_003_HZ = BMP3_ODR_0_003_HZ,
    ODR_0_001_HZ = BMP3_ODR_0_001_HZ
  };
  Bmp3(TwoWire *i2c, const I2cAddr addr);
  Bmp3(SPIClass *spi, const uint8_t cs);
  bool Begin();
  bool ConfigTempOversampling(const Oversampling val);
  inline Oversampling temp_oversampling() const {
    return static_cast<Oversampling>(settings_.odr_filter.temp_os);
  }
  bool ConfigPresOversampling(const Oversampling val);
  inline Oversampling pres_oversampling() const {
    return static_cast<Oversampling>(settings_.odr_filter.press_os);
  }
  bool ConfigFilterCoef(const FilterCoef val);
  inline FilterCoef filter_coef() const {
    return static_cast<FilterCoef>(settings_.odr_filter.iir_filter);
  }
  bool ConfigOdr(const Odr val);
  inline Odr odr() const {
    return static_cast<Odr>(settings_.odr_filter.odr);
  }
  bool Read();
  inline bool Reset() {return (bmp3_soft_reset(&dev_) == BMP3_OK);}
  inline double pres_pa() const {return data_.pressure;}
  inline double die_temp_c() const {return data_.temperature;}

 private:
  /* BMP3 device settings */
  bmp3_dev dev_;
  /* BMP3 data */
  bmp3_data data_;
  /* BMP2 config */
  uint8_t pm_;
  uint32_t set_sel_;
  bmp3_settings req_set_, settings_;
  bmp3_status status_;
  /* Description of I2C and SPI interfaces */
  struct I2cIntf {
    uint8_t addr;
    TwoWire *i2c;
  } i2c_intf_;
  struct SpiIntf {
    uint8_t cs;
    SPIClass *spi;
  } spi_intf_;
  /* SPI clock speed, 10 MHz */
  static constexpr int32_t SPI_CLK_ = 10000000;
  /* Function prototype for delaying, ms */
  static void Delay_us(uint32_t period, void *intf_ptr);
  /* Function prototypes for reading and writing I2C and SPI data */
  static int8_t I2cWriteRegisters(uint8_t reg, const uint8_t * data,
                                  uint32_t len, void * intf);
  static int8_t I2cReadRegisters(uint8_t reg, uint8_t * data, uint32_t len,
                                 void * intf);
  static int8_t SpiWriteRegisters(uint8_t reg, const uint8_t * data,
                                  uint32_t len, void * intf);
  static int8_t SpiReadRegisters(uint8_t reg, uint8_t * data, uint32_t len,
                                 void * intf);
};

}  // namespace bfs

#endif  // SRC_BMP2_H_
