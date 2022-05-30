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
#include <maliput/api/road_network.h>

namespace delphyne {

/// System in charge of updating the traffic lights models according to the
/// current value of the traffic lights state obtained through the maliput::api::RoadNetwork.
///
/// Input Port 0: ignition::msgs::Model_V message with models from a scene.
///   (InputPort getter: get_models_input_port())
///
/// Output Port 0: ignition::msgs::Model_V message with updated traffic light models.
///   (OutputPort getter: get_traffic_lights_models_output_port())
///
/// Output Port 1: ignition::msgs::Boolean message to indicate new traffic light state data is present.
///   (OutputPort getter: get_new_data_output_port())
class IgnModelsTrafficLights : public drake::systems::LeafSystem<double> {
 public:
  IgnModelsTrafficLights(maliput::api::RoadNetwork* road_network);

  /// Returns the input descriptor for models. This port expects an
  /// ignition::msgs::Model_V message, containing the models in the scene.
  const drake::systems::InputPort<double>& get_models_input_port() const {
    return get_input_port(models_input_port_index_);
  }
  int get_models_input_port_index() const { return models_input_port_index_; }

  const drake::systems::OutputPort<double>& get_traffic_lights_models_output_port() const {
    return get_output_port(traffic_light_output_port_index_);
  }
  int get_traffic_light_output_port_index() const { return traffic_light_output_port_index_; }

  const drake::systems::OutputPort<double>& get_new_data_output_port() const {
    return get_output_port(new_data_output_port_index_);
  }
  int get_new_data_output_port_index() const { return new_data_output_port_index_; }

 private:
  static const maliput::math::Vector4 kRedBulbColorOn;
  static const maliput::math::Vector4 kGreenBulbColorOn;
  static const maliput::math::Vector4 kYellowBulbColorOn;
  static const maliput::math::Vector4 kRedBulbColorOff;
  static const maliput::math::Vector4 kGreenBulbColorOff;
  static const maliput::math::Vector4 kYellowBulbColorOff;

  void CalcIgnModelVMessage(const drake::systems::Context<double>& context,
                            ignition::msgs::Model_V* output_models) const;

  void CalcNewDataMessage(const drake::systems::Context<double>& context,
                          ignition::msgs::Boolean* output_new_data) const;

  void SetBulbState(ignition::msgs::Link* link, const maliput::api::rules::BulbColor& bulb_color,
                    maliput::api::rules::BulbState& bulb_state) const;

  maliput::api::RoadNetwork* road_network_{nullptr};

  int models_input_port_index_{};
  int new_data_output_port_index_{};
  int traffic_light_output_port_index_{};

  mutable bool new_data_{false};
  mutable maliput::api::rules::BulbStates last_bulb_states_{};
};

}  // namespace delphyne
