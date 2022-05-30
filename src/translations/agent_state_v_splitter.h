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

#include <memory>

#include <drake/systems/framework/leaf_system.h>

#include "delphyne/protobuf/agent_state.pb.h"

namespace delphyne {

/// @brief A system that takes a AgentState_V and splits it into separate
/// AgentState messages.
template <typename T>
class AgentState_v_Splitter : public drake::systems::LeafSystem<T> {
 public:
  explicit AgentState_v_Splitter(int agents_number);

 private:
  /// @brief Sets the output value with the AgentState message that's at the
  ///
  /// In contrast to typical system's `Calc` methods, the presence of an extra
  /// argument in this function's signature comes from the need to specify which
  /// of the vector elements must be used to generate the output.
  /// given index of the AgentState_v.
  /// @param[in] context The simulation context.
  /// @param[in] output A pointer to an abstracted AgentState message.
  /// @param[in] agent_index The index to the desired agent in the input vector.
  void DoSplit(const drake::systems::Context<T>& context, drake::AbstractValue* output, int agent_index) const;

  /// @brief Allocates an abstract value object.
  std::unique_ptr<drake::AbstractValue> DoAlloc() const;

  /// @brief The message instance that is updated with the system's output
  /// on each simulation step.
  ignition::msgs::AgentState state_;
};

}  // namespace delphyne
