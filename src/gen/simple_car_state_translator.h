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

#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <memory>
#include <vector>

#include <drake/lcmt_simple_car_state_t.hpp>
#include <drake/systems/lcm/lcm_and_vector_base_translator.h>

#include "gen/simple_car_state.h"

namespace delphyne {

/**
 * Translates between LCM message objects and VectorBase objects for the
 * SimpleCarState type.
 */
class SimpleCarStateTranslator final : public drake::systems::lcm::LcmAndVectorBaseTranslator {
 public:
  SimpleCarStateTranslator() : LcmAndVectorBaseTranslator(SimpleCarStateIndices::kNumCoordinates) {}
  std::unique_ptr<drake::systems::BasicVector<double>> AllocateOutputVector() const final;
  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
                   drake::systems::VectorBase<double>* vector_base) const final;
  void Serialize(double time, const drake::systems::VectorBase<double>& vector_base,
                 std::vector<uint8_t>* lcm_message_bytes) const final;
};

}  // namespace delphyne
