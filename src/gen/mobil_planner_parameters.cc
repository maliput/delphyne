// Copyright 2018 Toyota Research Institute

#include "gen/mobil_planner_parameters.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace delphyne {

const int MobilPlannerParametersIndices::kNumCoordinates;
const int MobilPlannerParametersIndices::kP;
const int MobilPlannerParametersIndices::kThreshold;
const int MobilPlannerParametersIndices::kMaxDeceleration;

const std::vector<std::string>& MobilPlannerParametersIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(std::vector<std::string>{
      "p",                 // BR
      "threshold",         // BR
      "max_deceleration",  // BR
  });
  return coordinates.access();
}

}  // namespace delphyne
