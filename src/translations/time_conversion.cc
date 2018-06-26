// Copyright 2017 Toyota Research Institute

#include "translations/time_conversion.h"

#include <cmath>

#include <ignition/msgs.hh>

#include "delphyne/macros.h"

namespace delphyne {

std::pair<int64_t, int64_t> MicrosToSecsAndNanos(int64_t micros) {
  DELPHYNE_VALIDATE(micros >= 0, std::invalid_argument,
                    "Microseconds must be >= 0");
  double integral{}, decimal{};
  decimal = std::modf(static_cast<double>(micros) * 1.0e-6, &integral);
  const int64_t sec = static_cast<int64_t>(integral);
  const int64_t nsecs = static_cast<int64_t>(decimal * 1.0e9);
  return {sec, nsecs};
}

std::pair<int64_t, int64_t> MillisToSecsAndNanos(int64_t millis) {
  DELPHYNE_VALIDATE(millis >= 0, std::invalid_argument,
                    "Milliseconds must be >= 0");
  double integral{}, decimal{};
  decimal = std::modf(static_cast<double>(millis) * 1.0e-3, &integral);
  const int64_t sec = static_cast<int64_t>(integral);
  const int64_t nsecs = static_cast<int64_t>(decimal * 1.0e9);
  return {sec, nsecs};
}

std::pair<int64_t, int64_t> SecsToSecsAndNanos(double time) {
  DELPHYNE_VALIDATE(!std::isnan(time), std::invalid_argument,
                    "Time must be a valid number");
  DELPHYNE_VALIDATE(time >= 0.0, std::invalid_argument, "Time must be >= 0");
  double integral{}, decimal{};
  decimal = std::modf(time, &integral);
  const int64_t sec = static_cast<int64_t>(integral);
  const int64_t nsecs = static_cast<int64_t>(decimal * 1.0e9);
  return {sec, nsecs};
}

double SecsAndNanosToMillis(int64_t secs, int64_t nsecs) {
  DELPHYNE_VALIDATE(secs >= 0, std::invalid_argument, "Seconds must be >= 0");
  DELPHYNE_VALIDATE(nsecs >= 0, std::invalid_argument,
                    "Nanoseconds must be >= 0");
  return static_cast<double>(secs) * 1e3 + static_cast<double>(nsecs) * 1e-6;
}

ignition::msgs::Time MillisToIgnitionTime(int64_t millis) {
  DELPHYNE_VALIDATE(millis >= 0, std::invalid_argument,
                    "Milliseconds must be >= 0");
  ignition::msgs::Time ign_time;
  std::pair<int64_t, int64_t> secs_and_nanos = MillisToSecsAndNanos(millis);
  ign_time.set_sec(std::get<0>(secs_and_nanos));
  ign_time.set_nsec(std::get<1>(secs_and_nanos));
  return ign_time;
}

ignition::msgs::Time MicrosToIgnitionTime(int64_t micros) {
  DELPHYNE_VALIDATE(micros >= 0, std::invalid_argument,
                    "Microseconds must be >= 0");
  ignition::msgs::Time ign_time;
  std::pair<int64_t, int64_t> secs_and_nanos = MicrosToSecsAndNanos(micros);
  ign_time.set_sec(std::get<0>(secs_and_nanos));
  ign_time.set_nsec(std::get<1>(secs_and_nanos));
  return ign_time;
}

ignition::msgs::Time SecsToIgnitionTime(double secs) {
  DELPHYNE_VALIDATE(!std::isnan(secs), std::invalid_argument,
                    "Seconds must be a valid number");
  DELPHYNE_VALIDATE(secs >= 0.0, std::invalid_argument, "Seconds must be >= 0");
  ignition::msgs::Time ign_time;
  std::pair<int64_t, int64_t> secs_and_nanos = SecsToSecsAndNanos(secs);
  ign_time.set_sec(std::get<0>(secs_and_nanos));
  ign_time.set_nsec(std::get<1>(secs_and_nanos));
  return ign_time;
}

int64_t IgnitionTimeToMillis(const ignition::msgs::Time ign_time) {
  return SecsAndNanosToMillis(ign_time.sec(), ign_time.nsec());
}

}  // namespace delphyne
