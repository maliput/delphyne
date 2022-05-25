// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
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

#include <drake/systems/framework/leaf_system.h>
#include <ignition/msgs.hh>

namespace delphyne {

/// Outputs the ids of the received ignition models.
///
/// Input Port 0: ignition::msgs::Model_V messages from which to obtain the ids.
///   (InputPort getter: get_models_input_port())
///
/// Output Port 0: Ids of the received ignition models in an ignition::msgs::UInt32_V message.
///   (OutputPort getter: get_ids_output_port())
class IgnModelsToIds : public drake::systems::LeafSystem<double> {
 public:
  IgnModelsToIds();

  const drake::systems::InputPort<double>& get_models_input_port() const {
    return get_input_port(models_input_port_index_);
  }

  int get_models_input_port_index() const { return models_input_port_index_; }

  const drake::systems::OutputPort<double>& get_ids_output_port() const {
    return get_output_port(ids_output_port_index_);
  }
  int get_ids_output_port_index() const { return ids_output_port_index_; }

 private:
  void CalcIdsFromModelVMessage(const drake::systems::Context<double>& context,
                                ignition::msgs::UInt32_V* output_ids) const;

  int models_input_port_index_{};
  int ids_output_port_index_{};
};

}  // namespace delphyne
