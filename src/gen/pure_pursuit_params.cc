// Copyright 2018 Toyota Research Institute

#include "gen/pure_pursuit_params.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace delphyne {

const int PurePursuitParamsIndices::kNumCoordinates;
const int PurePursuitParamsIndices::kSLookahead;

const std::vector<std::string>& PurePursuitParamsIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(std::vector<std::string>{
      "s_lookahead",  // BR
  });
  return coordinates.access();
}

}  // namespace delphyne
