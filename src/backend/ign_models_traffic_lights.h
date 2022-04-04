// Copyright 2022 Toyota Research Institute

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
