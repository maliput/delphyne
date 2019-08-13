// Copyright 2018 Toyota Research Institute

#include "gen/maliput_railcar_state.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace delphyne {

const int MaliputRailcarStateIndices::kNumCoordinates;
const int MaliputRailcarStateIndices::kS;
const int MaliputRailcarStateIndices::kSpeed;

const std::vector<std::string>& MaliputRailcarStateIndices::GetCoordinateNames() {
  static const drake::never_destroyed<std::vector<std::string>> coordinates(std::vector<std::string>{
      "s",      // BR
      "speed",  // BR
  });
  return coordinates.access();
}

}  // namespace delphyne
