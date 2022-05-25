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

namespace delphyne {

/// A pass through system with input `u` and output `y = u` that only
/// redirects to the output when the commanded switch is true.
///
/// This system is expected to receive ignition msgs data types,
/// as they are expected to provide a `Clear()` method for clearing out the content.
///
/// Input Port 0: Data to be passed through.
///   (InputPort getter: get_data_input_port())
///
/// Input Port 1: Switch to command behavior.
///   (InputPort getter: get_switch_input_port())
///
/// Output Port 0: Data passed through.
///   (OutputPort getter: get_data_output_port())
/// @tparam T The data type of the input data and output.
template <typename T>
class IgnCommandedPassThrough : public drake::systems::LeafSystem<double> {
 public:
  IgnCommandedPassThrough();

  const drake::systems::InputPort<double>& get_data_input_port() const {
    return get_input_port(data_input_port_index_);
  }

  int get_data_input_port_index() const { return data_input_port_index_; }

  const drake::systems::InputPort<double>& get_switch_input_port() const {
    return get_input_port(switch_input_port_index_);
  }

  int get_switch_input_port_index() const { return switch_input_port_index_; }

  const drake::systems::OutputPort<double>& get_data_output_port() const {
    return get_output_port(data_output_port_index_);
  }
  int get_data_output_port_index() const { return data_output_port_index_; }

 private:
  void CalcOutput(const drake::systems::Context<double>& context, T* output_ids) const;

  int data_input_port_index_{};
  int data_output_port_index_{};
  int switch_input_port_index_{};
};

}  // namespace delphyne
