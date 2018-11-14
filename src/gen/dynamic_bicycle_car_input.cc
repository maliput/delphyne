// Copyright 2018 Toyota Research Institute

#include "gen/dynamic_bicycle_car_input.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace delphyne {

const int DynamicBicycleCarInputIndices::kNumCoordinates;
const int DynamicBicycleCarInputIndices::kSteerCd;
const int DynamicBicycleCarInputIndices::kFCpX;

const std::vector<std::string>&
DynamicBicycleCarInputIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "steer_CD",  // BR
          "f_Cp_x",    // BR
      });
  return coordinates.access();
}

}  // namespace delphyne
