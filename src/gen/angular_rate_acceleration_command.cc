// Copyright 2020 Toyota Research Institute

#include "gen/angular_rate_acceleration_command.h"

namespace delphyne {

const int AngularRateAccelerationCommandIndices::kNumCoordinates;
const int AngularRateAccelerationCommandIndices::kAngularRate;
const int AngularRateAccelerationCommandIndices::kAcceleration;

const std::vector<std::string>& AngularRateAccelerationCommandIndices::GetCoordinateNames() {
  static const maliput::common::never_destroyed<std::vector<std::string>> coordinates(std::vector<std::string>{
      "angular_rate",  // BR
      "acceleration",  // BR
  });
  return coordinates.access();
}

}  // namespace delphyne
