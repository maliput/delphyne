// Copyright 2018 Toyota Research Institute

#include "gen/simple_car_state.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace delphyne {

const int SimpleCarStateIndices::kNumCoordinates;
const int SimpleCarStateIndices::kX;
const int SimpleCarStateIndices::kY;
const int SimpleCarStateIndices::kHeading;
const int SimpleCarStateIndices::kVelocity;

const std::vector<std::string>& SimpleCarStateIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(std::vector<std::string>{
      "x",         // BR
      "y",         // BR
      "heading",   // BR
      "velocity",  // BR
  });
  return coordinates.access();
}

}  // namespace delphyne
