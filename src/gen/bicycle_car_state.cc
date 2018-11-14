// Copyright 2018 Toyota Research Institute

#include "gen/bicycle_car_state.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace delphyne {

const int BicycleCarStateIndices::kNumCoordinates;
const int BicycleCarStateIndices::kPsi;
const int BicycleCarStateIndices::kPsiDot;
const int BicycleCarStateIndices::kBeta;
const int BicycleCarStateIndices::kVel;
const int BicycleCarStateIndices::kSx;
const int BicycleCarStateIndices::kSy;

const std::vector<std::string>& BicycleCarStateIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "Psi",      // BR
          "Psi_dot",  // BR
          "beta",     // BR
          "vel",      // BR
          "sx",       // BR
          "sy",       // BR
      });
  return coordinates.access();
}

}  // namespace delphyne
