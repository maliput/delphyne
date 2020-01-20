// Copyright 2018 Toyota Research Institute

#include "gen/driving_command.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace delphyne {

const int DrivingCommandIndices::kNumCoordinates;
const int DrivingCommandIndices::kSteeringAngle;
const int DrivingCommandIndices::kAcceleration;

const std::vector<std::string>& DrivingCommandIndices::GetCoordinateNames() {
  static const maliput::common::never_destroyed<std::vector<std::string>> coordinates(std::vector<std::string>{
      "steering_angle",  // BR
      "acceleration",    // BR
  });
  return coordinates.access();
}

}  // namespace delphyne
