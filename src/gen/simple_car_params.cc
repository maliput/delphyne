// Copyright 2018 Toyota Research Institute

#include "gen/simple_car_params.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace delphyne {

const int SimpleCarParamsIndices::kNumCoordinates;
const int SimpleCarParamsIndices::kWheelbase;
const int SimpleCarParamsIndices::kTrack;
const int SimpleCarParamsIndices::kMaxAbsSteeringAngle;
const int SimpleCarParamsIndices::kMaxVelocity;
const int SimpleCarParamsIndices::kMaxAcceleration;
const int SimpleCarParamsIndices::kVelocityLimitKp;

const std::vector<std::string>& SimpleCarParamsIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "wheelbase",               // BR
          "track",                   // BR
          "max_abs_steering_angle",  // BR
          "max_velocity",            // BR
          "max_acceleration",        // BR
          "velocity_limit_kp",       // BR
      });
  return coordinates.access();
}

}  // namespace delphyne
