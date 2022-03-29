// Copyright 2022 Toyota Research Institute
#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <ignition/msgs.hh>

namespace delphyne {

/// Outputs the ids of the received ignition models.
///
/// Input Port 0: ignition::msgs::Model_V messages to be obtained the ids from.
///   (InputPort getter: get_models_input_port())
///
/// Output Port 0: Ids of the received ignition models in a ignition::msgs::UInt32_V message.
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

  // The index of the models' abstract input port.
  int models_input_port_index_;
  int ids_output_port_index_;
};

}  // namespace delphyne
