// Copyright 2018 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "backend/time_conversion.h"

#include <math.h>

namespace delphyne {
namespace backend {

std::pair<int64_t, int64_t> MicrosToSecsAndNanos(int64_t micros) {
  double integral{}, decimal{};
  decimal = modf(static_cast<double>(micros) * 1.0e-6, &integral);
  const int64_t sec = static_cast<int64_t>(integral);
  const int64_t nsecs = static_cast<int64_t>(decimal * 1.0e9);
  return {sec, nsecs};
}

std::pair<int64_t, int64_t> MillisToSecsAndNanos(int64_t millis) {
  double integral{}, decimal{};
  decimal = modf(static_cast<double>(millis) * 1.0e-3, &integral);
  const int64_t sec = static_cast<int64_t>(integral);
  const int64_t nsecs = static_cast<int64_t>(decimal * 1.0e9);
  return {sec, nsecs};
}

double SecsAndNanosToMillis(int64_t secs, int64_t nsecs) {
  return static_cast<double>(secs) * 1e3 + static_cast<double>(nsecs) * 1e-6;
}

std::pair<int64_t, int64_t> ToSecsAndNanos(double time) {
  double integral{}, decimal{};
  decimal = modf(time, &integral);
  const int64_t sec = static_cast<int64_t>(integral);
  const int64_t nsecs = static_cast<int64_t>(decimal * 1.0e9);
  return {sec, nsecs};
}

}  // namespace backend
}  // namespace delphyne
