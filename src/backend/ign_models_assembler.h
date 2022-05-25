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

#include <drake/systems/framework/leaf_system.h>
#include <ignition/msgs.hh>

namespace delphyne {

/// A helper System to assemble ignition::msgs::Model messages (given
/// as an ignition::msgs::Model_V message i.e. a collection of them)
/// using externally provided details like name and pose.
///
/// Besides the base ignition::msgs::Model_V input port, a
/// drake::systems::rendering::PoseBundle<double> input port provides
/// poses and names.
class IgnModelsAssembler : public drake::systems::LeafSystem<double> {
 public:
  IgnModelsAssembler();

  /// Returns the input descriptor for models. This port expects an
  /// ignition::msgs::Model_V message, containing the models to be assembled.
  const drake::systems::InputPort<double>& get_models_input_port() const {
    return get_input_port(models_input_port_index_);
  }

  int get_models_input_port_index() const { return models_input_port_index_; }

  /// Returns the input port descriptor for model states. This port expects a
  /// drake::systems::rendering::PoseBundle<double>, containing all models
  /// states.
  const drake::systems::InputPort<double>& get_states_input_port() const {
    return get_input_port(states_input_port_index_);
  }

  int get_states_input_port_index() const { return states_input_port_index_; }

 private:
  // Calculates (i.e. assembles) an output model vector message
  // based on the input model vector and model states.
  void CalcAssembledIgnModelVMessage(const drake::systems::Context<double>& context,
                                     ignition::msgs::Model_V* output_models) const;

  // The index of the models' abstract input port.
  int models_input_port_index_;
  // The index of the states' abstract input port.
  int states_input_port_index_;
};

}  // namespace delphyne
