// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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

const std::vector<std::string>& IdmPlannerParametersIndices::GetCoordinateNames() {
  static const maliput::common::never_destroyed<std::vector<std::string>> coordinates(std::vector<std::string>{
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
