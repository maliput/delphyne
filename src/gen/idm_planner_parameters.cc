// Copyright 2018 Toyota Research Institute

#include "gen/idm_planner_parameters.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace delphyne {

const int IdmPlannerParametersIndices::kNumCoordinates;
const int IdmPlannerParametersIndices::kVRef;
const int IdmPlannerParametersIndices::kA;
const int IdmPlannerParametersIndices::kB;
const int IdmPlannerParametersIndices::kS0;
const int IdmPlannerParametersIndices::kTimeHeadway;
const int IdmPlannerParametersIndices::kDelta;
const int IdmPlannerParametersIndices::kBloatDiameter;
const int IdmPlannerParametersIndices::kDistanceLowerLimit;
const int IdmPlannerParametersIndices::kScanAheadDistance;

const std::vector<std::string>&
IdmPlannerParametersIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{
          "v_ref",                 // BR
          "a",                     // BR
          "b",                     // BR
          "s_0",                   // BR
          "time_headway",          // BR
          "delta",                 // BR
          "bloat_diameter",        // BR
          "distance_lower_limit",  // BR
          "scan_ahead_distance",   // BR
      });
  return coordinates.access();
}

}  // namespace delphyne
