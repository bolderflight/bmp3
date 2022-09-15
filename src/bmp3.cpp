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

#include "bmp3.h"  // NOLINT
#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
#else
#include "core/core.h"
#endif
#include <stddef.h>
#include <stdint.h>

namespace bfs {

Bmp3::Bmp3(TwoWire *i2c, const I2cAddr addr) {
  i2c_intf_.i2c = i2c;
  i2c_intf_.addr = static_cast<uint8_t>(addr);
  dev_.intf_ptr = &i2c_intf_;
  dev_.intf = BMP3_I2C_INTF;
  dev_.read = I2cReadRegisters;
  dev_.write = I2cWriteRegisters;
  dev_.delay_us = Delay_us;
}

Bmp3::Bmp3(SPIClass *spi, const uint8_t cs) {
  pinMode(cs, OUTPUT);
  spi_intf_.spi = spi;
  spi_intf_.cs = cs;
  dev_.intf_ptr = &spi_intf_;
  dev_.intf = BMP3_SPI_INTF;
  dev_.read = SpiReadRegisters;
  dev_.write = SpiWriteRegisters;
  dev_.delay_us = Delay_us;
}

bool Bmp3::Begin() {
  /* Initialize communication */
  if (bmp3_init(&dev_) != BMP3_OK) {
    return false;
  }
  /* Set defaults */
  req_set_.press_en = BMP3_ENABLE;
  req_set_.temp_en = BMP3_ENABLE;
  req_set_.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
  req_set_.odr_filter.temp_os =BMP3_NO_OVERSAMPLING;
  req_set_.odr_filter.odr = BMP3_ODR_50_HZ;
  req_set_.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_1;
  req_set_.int_settings.drdy_en = BMP3_ENABLE;
  req_set_.int_settings.latch = BMP3_INT_PIN_NON_LATCH;
  req_set_.int_settings.level = BMP3_INT_PIN_ACTIVE_HIGH;
  req_set_.int_settings.output_mode = BMP3_INT_PIN_PUSH_PULL;
  set_sel_ = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS |
             BMP3_SEL_TEMP_OS | BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR |
             BMP3_SEL_OUTPUT_MODE | BMP3_SEL_LEVEL | BMP3_SEL_LATCH |
             BMP3_SEL_DRDY_EN;
  if (bmp3_set_sensor_settings(set_sel_, &req_set_, &dev_) != BMP3_OK) {
    return false;
  }
  /* Get the sensor config */
  if (bmp3_get_sensor_settings(&settings_, &dev_) != BMP3_OK) {
    return false;
  }
  /* Set the power mode */
  req_set_ = settings_;
  req_set_.op_mode = BMP3_MODE_NORMAL;
  if (bmp3_set_op_mode(&req_set_, &dev_) != BMP3_OK) {
    return false;
  }
  /* Get the power mode */
  if (bmp3_get_op_mode(&pm_, &dev_) != BMP3_OK) {
    return false;
  }
  settings_.op_mode = pm_;
  return true;
}

bool Bmp3::ConfigTempOversampling(const Oversampling val) {
  req_set_ = settings_;
  req_set_.odr_filter.temp_os = static_cast<uint8_t>(val);
  set_sel_ = BMP3_SEL_TEMP_OS;
  if (bmp3_set_sensor_settings(set_sel_, &req_set_, &dev_) != BMP3_OK) {
    return false;
  }
  /* Get the sensor config */
  if (bmp3_get_sensor_settings(&settings_, &dev_) != BMP3_OK) {
    return false;
  }
  return true;
}

bool Bmp3::ConfigPresOversampling(const Oversampling val) {
  req_set_ = settings_;
  req_set_.odr_filter.press_os = static_cast<uint8_t>(val);
  set_sel_ = BMP3_SEL_PRESS_OS;
  if (bmp3_set_sensor_settings(set_sel_, &req_set_, &dev_) != BMP3_OK) {
    return false;
  }
  /* Get the sensor config */
  if (bmp3_get_sensor_settings(&settings_, &dev_) != BMP3_OK) {
    return false;
  }
  return true;
}

bool Bmp3::ConfigFilterCoef(const FilterCoef val) {
  req_set_ = settings_;
  req_set_.odr_filter.iir_filter = static_cast<uint8_t>(val);
  set_sel_ = BMP3_SEL_IIR_FILTER;
  if (bmp3_set_sensor_settings(set_sel_, &req_set_, &dev_) != BMP3_OK) {
    return false;
  }
  /* Get the sensor config */
  if (bmp3_get_sensor_settings(&settings_, &dev_) != BMP3_OK) {
    return false;
  }
  return true;
}

bool Bmp3::ConfigOdr(const Odr val) {
  req_set_ = settings_;
  req_set_.odr_filter.odr = static_cast<uint8_t>(val);
  set_sel_ = BMP3_SEL_ODR;
  if (bmp3_set_sensor_settings(set_sel_, &req_set_, &dev_) != BMP3_OK) {
    return false;
  }
  /* Get the sensor config */
  if (bmp3_get_sensor_settings(&settings_, &dev_) != BMP3_OK) {
    return false;
  }
  return true;
}

bool Bmp3::Read() {
  if (bmp3_get_status(&status_, &dev_) != BMP3_OK) {
    return false;
  }
  if (status_.intr.drdy) {
    if (bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data_, &dev_) != BMP3_OK) {
      return false;
    }
    if (bmp3_get_status(&status_, &dev_) != BMP3_OK) {
      return false;
    }
    return true;
  }
  return false;
}

void Bmp3::Delay_us(uint32_t period, void *intf_ptr) {
  delayMicroseconds(period);
}

int8_t Bmp3::I2cWriteRegisters(uint8_t reg, const uint8_t * data,
                               uint32_t len, void * intf) {
  /* NULL pointer check */
  if ((!data) || (!intf)) {
    return BMP3_E_NULL_PTR;
  }
  /* Check length */
  if (len == 0) {
    return BMP3_E_INVALID_LEN;
  }
  I2cIntf *iface = static_cast<I2cIntf *>(intf);
  iface->i2c->beginTransmission(iface->addr);
  if (iface->i2c->write(reg) != 1) {
    return BMP3_E_COMM_FAIL;
  }
  if (iface->i2c->write(data, len) != len) {
    return BMP3_E_COMM_FAIL;
  }
  iface->i2c->endTransmission();
  return BMP3_OK;
}

int8_t Bmp3::I2cReadRegisters(uint8_t reg, uint8_t * data, uint32_t len,
                              void * intf) {
  /* NULL pointer check */
  if ((!data) || (!intf)) {
    return BMP3_E_NULL_PTR;
  }
  /* Check length */
  if (len == 0) {
    return BMP3_E_INVALID_LEN;
  }
  I2cIntf *iface = static_cast<I2cIntf *>(intf);
  iface->i2c->beginTransmission(iface->addr);
  if (iface->i2c->write(reg) != 1) {
    return BMP3_E_COMM_FAIL;
  }
  iface->i2c->endTransmission(false);
  if (iface->i2c->requestFrom(iface->addr, len) != len) {
    return BMP3_E_COMM_FAIL;
  }
  for (size_t i = 0; i < len; i++) {
    data[i] = iface->i2c->read();
  }
  return BMP3_OK;
}

int8_t Bmp3::SpiWriteRegisters(uint8_t reg, const uint8_t * data,
                               uint32_t len, void * intf) {
  /* NULL pointer check */
  if ((!data) || (!intf)) {
    return BMP3_E_NULL_PTR;
  }
  /* Check length */
  if (len == 0) {
    return BMP3_E_INVALID_LEN;
  }
  SpiIntf *iface = static_cast<SpiIntf *>(intf);
  iface->spi->beginTransaction(SPISettings(SPI_CLK_, MSBFIRST, SPI_MODE0));
  #if defined(TEENSYDUINO)
  digitalWriteFast(iface->cs, LOW);
  #else
  digitalWrite(iface->cs, LOW);
  #endif
  iface->spi->transfer(reg);
  for (size_t i = 0; i < len; i++) {
    iface->spi->transfer(data[i]);
  }
  #if defined(TEENSYDUINO)
  digitalWriteFast(iface->cs, HIGH);
  #else
  digitalWrite(iface->cs, HIGH);
  #endif
  iface->spi->endTransaction();
  return BMP3_OK;
}

int8_t Bmp3::SpiReadRegisters(uint8_t reg, uint8_t * data, uint32_t len,
                              void * intf) {
  /* NULL pointer check */
  if ((!data) || (!intf)) {
    return BMP3_E_NULL_PTR;
  }
  /* Check length */
  if (len == 0) {
    return BMP3_E_INVALID_LEN;
  }
  SpiIntf *iface = static_cast<SpiIntf *>(intf);
  iface->spi->beginTransaction(SPISettings(SPI_CLK_, MSBFIRST, SPI_MODE0));
  #if defined(TEENSYDUINO)
  digitalWriteFast(iface->cs, LOW);
  #else
  digitalWrite(iface->cs, LOW);
  #endif
  iface->spi->transfer(reg);
  for (size_t i = 0; i < len; i++) {
    data[i] = iface->spi->transfer(0x00);
  }
  #if defined(TEENSYDUINO)
  digitalWriteFast(iface->cs, HIGH);
  #else
  digitalWrite(iface->cs, HIGH);
  #endif
  iface->spi->endTransaction();
  return BMP3_OK;
}

}  // namespace bfs
