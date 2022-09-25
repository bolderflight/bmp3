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

#include "bfs-bmp3.h"

namespace bfs {

bool BfsBmp3::Init(const Config &config) {
  data_.installed = false;
  if (!pres_.Begin()) {
    return false;
  }
  if (!pres_.ConfigOsMode(config.oversampling_mode)) {
    return false;
  }
  if (!pres_.ConfigFilterCoef(config.filter_coef)) {
    return false;
  }
  data_.installed = true;
  return true;
}

bool BfsBmp3::Read() {
  data_.new_data = pres_.Read();
  if (data_.new_data) {
    data_.pres_pa = pres_.pres_pa();
    data_.die_temp_c = pres_.die_temp_c();
  }
  return data_.new_data;
}

}  // namespace bfs
