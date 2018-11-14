// Copyright 2018 Toyota Research Institute

#include "gen/bicycle_car_parameters.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace delphyne {

const int BicycleCarParametersIndices::kNumCoordinates;
const int BicycleCarParametersIndices::kMass;
const int BicycleCarParametersIndices::kLf;
const int BicycleCarParametersIndices::kLr;
const int BicycleCarParametersIndices::kIz;
const int BicycleCarParametersIndices::kCf;
const int BicycleCarParametersIndices::kCr;

const std::vector<std::string>&
BicycleCarParametersIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "mass",  // BR
          "lf",    // BR
          "lr",    // BR
          "Iz",    // BR
          "Cf",    // BR
          "Cr",    // BR
      });
  return coordinates.access();
}

}  // namespace delphyne
