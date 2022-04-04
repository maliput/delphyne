// Copyright 2022 Toyota Research Institute
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
