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

#include "gen/simple_car_params.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

namespace delphyne {

const int SimpleCarParamsIndices::kNumCoordinates;
const int SimpleCarParamsIndices::kWheelbase;
const int SimpleCarParamsIndices::kTrack;
const int SimpleCarParamsIndices::kMaxAbsSteeringAngle;
const int SimpleCarParamsIndices::kMaxVelocity;
const int SimpleCarParamsIndices::kMaxAcceleration;
const int SimpleCarParamsIndices::kVelocityLimitKp;

const std::vector<std::string>& SimpleCarParamsIndices::GetCoordinateNames() {
  static const maliput::common::never_destroyed<std::vector<std::string>> coordinates(std::vector<std::string>{
      "wheelbase",               // BR
      "track",                   // BR
      "max_abs_steering_angle",  // BR
      "max_velocity",            // BR
      "max_acceleration",        // BR
      "velocity_limit_kp",       // BR
  });
  return coordinates.access();
}

}  // namespace delphyne
