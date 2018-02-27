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

#include <cmath>

#include <ignition/msgs.hh>

#include "backend/system.h"

namespace delphyne {
namespace backend {

std::pair<int64_t, int64_t> MicrosToSecsAndNanos(int64_t micros) {
  DELPHYNE_DEMAND(micros >= 0);
  double integral{}, decimal{};
  decimal = std::modf(static_cast<double>(micros) * 1.0e-6, &integral);
  const int64_t sec = static_cast<int64_t>(integral);
  const int64_t nsecs = static_cast<int64_t>(decimal * 1.0e9);
  return {sec, nsecs};
}

std::pair<int64_t, int64_t> MillisToSecsAndNanos(int64_t millis) {
  DELPHYNE_DEMAND(millis >= 0);
  double integral{}, decimal{};
  decimal = std::modf(static_cast<double>(millis) * 1.0e-3, &integral);
  const int64_t sec = static_cast<int64_t>(integral);
  const int64_t nsecs = static_cast<int64_t>(decimal * 1.0e9);
  return {sec, nsecs};
}

std::pair<int64_t, int64_t> SecsToSecsAndNanos(double time) {
  DELPHYNE_DEMAND(!std::isnan(time));
  DELPHYNE_DEMAND(time >= 0.);
  double integral{}, decimal{};
  decimal = std::modf(time, &integral);
  const int64_t sec = static_cast<int64_t>(integral);
  const int64_t nsecs = static_cast<int64_t>(decimal * 1.0e9);
  return {sec, nsecs};
}

double SecsAndNanosToMillis(int64_t secs, int64_t nsecs) {
  DELPHYNE_DEMAND(secs >= 0);
  DELPHYNE_DEMAND(nsecs >= 0);
  return static_cast<double>(secs) * 1e3 + static_cast<double>(nsecs) * 1e-6;
}

ignition::msgs::Time MillisToIgnitionTime(int64_t millis) {
  DELPHYNE_DEMAND(millis >= 0);
  ignition::msgs::Time ign_time;
  std::pair<int64_t, int64_t> secs_and_nanos = MillisToSecsAndNanos(millis);
  ign_time.set_sec(std::get<0>(secs_and_nanos));
  ign_time.set_nsec(std::get<1>(secs_and_nanos));
  return ign_time;
}

ignition::msgs::Time MicrosToIgnitionTime(int64_t micros) {
  DELPHYNE_DEMAND(micros >= 0);
  ignition::msgs::Time ign_time;
  std::pair<int64_t, int64_t> secs_and_nanos = MicrosToSecsAndNanos(micros);
  ign_time.set_sec(std::get<0>(secs_and_nanos));
  ign_time.set_nsec(std::get<1>(secs_and_nanos));
  return ign_time;
}

ignition::msgs::Time SecsToIgnitionTime(double secs) {
  DELPHYNE_DEMAND(!std::isnan(secs));
  DELPHYNE_DEMAND(secs >= 0.);
  ignition::msgs::Time ign_time;
  std::pair<int64_t, int64_t> secs_and_nanos = SecsToSecsAndNanos(secs);
  ign_time.set_sec(std::get<0>(secs_and_nanos));
  ign_time.set_nsec(std::get<1>(secs_and_nanos));
  return ign_time;
}

DELPHYNE_BACKEND_VISIBLE int64_t
IgnitionTimeToMillis(const ignition::msgs::Time ign_time) {
  return SecsAndNanosToMillis(ign_time.sec(), ign_time.nsec());
}

}  // namespace backend
}  // namespace delphyne
