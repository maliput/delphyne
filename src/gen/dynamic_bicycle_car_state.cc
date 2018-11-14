// Copyright 2018 Toyota Research Institute

#include "gen/dynamic_bicycle_car_state.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace delphyne {

const int DynamicBicycleCarStateIndices::kNumCoordinates;
const int DynamicBicycleCarStateIndices::kPLocpX;
const int DynamicBicycleCarStateIndices::kPLocpY;
const int DynamicBicycleCarStateIndices::kYawLc;
const int DynamicBicycleCarStateIndices::kVLcpX;
const int DynamicBicycleCarStateIndices::kVLcpY;
const int DynamicBicycleCarStateIndices::kYawdtLc;

const std::vector<std::string>&
DynamicBicycleCarStateIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "p_LoCp_x",  // BR
          "p_LoCp_y",  // BR
          "yaw_LC",    // BR
          "v_LCp_x",   // BR
          "v_LCp_y",   // BR
          "yawDt_LC",  // BR
      });
  return coordinates.access();
}

}  // namespace delphyne
