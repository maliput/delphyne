// Copyright 2018 Toyota Research Institute

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
