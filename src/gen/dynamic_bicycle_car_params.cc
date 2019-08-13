// Copyright 2018 Toyota Research Institute

#include "gen/dynamic_bicycle_car_params.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace delphyne {

const int DynamicBicycleCarParamsIndices::kNumCoordinates;
const int DynamicBicycleCarParamsIndices::kMass;
const int DynamicBicycleCarParamsIndices::kIzz;
const int DynamicBicycleCarParamsIndices::kCAlphaF;
const int DynamicBicycleCarParamsIndices::kCAlphaR;
const int DynamicBicycleCarParamsIndices::kMu;
const int DynamicBicycleCarParamsIndices::kLf;
const int DynamicBicycleCarParamsIndices::kLb;
const int DynamicBicycleCarParamsIndices::kPLocpZ;
const int DynamicBicycleCarParamsIndices::kGravity;

const std::vector<std::string>& DynamicBicycleCarParamsIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(std::vector<std::string>{
      "mass",       // BR
      "izz",        // BR
      "c_alpha_f",  // BR
      "c_alpha_r",  // BR
      "mu",         // BR
      "Lf",         // BR
      "Lb",         // BR
      "p_LoCp_z",   // BR
      "gravity",    // BR
  });
  return coordinates.access();
}

}  // namespace delphyne
