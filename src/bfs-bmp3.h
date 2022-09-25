/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
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

#ifndef BMP3_SRC_BFS_BMP3_H_  // NOLINT
#define BMP3_SRC_BFS_BMP3_H_

#include "bmp3.h"  // NOLINT
#include "types.h"  // NOLINT

namespace bfs {

class BfsBmp3 : public Bmp3 {
 public:
  struct Config {
    OsMode oversampling_mode = OS_MODE_PRES_16X_TEMP_2X;
    FilterCoef filter_coef = FILTER_COEF_16;
  };
  BfsBmp3(TwoWire *i2c, const I2cAddr addr) : pres_(i2c, addr) {}
  BfsBmp3(SPIClass *spi, const uint8_t cs) : pres_(spi, cs) {}
  bool Init(const Config &config);
  bool Read();
  inline StaticPressData static_pres_data() const {return data_;}

 private:
  Bmp3 pres_;
  StaticPressData data_;
};

}  // namespace bfs

#endif  // BMP3_SRC_BFS_BMP3_H_ NOLINT
