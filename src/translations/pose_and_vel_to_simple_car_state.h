// Copyright 2018 Toyota Research Institute

#pragma once

#include <drake/automotive/simple_car.h>
#include <drake/systems/framework/leaf_system.h>

#include "delphyne/protobuf/simple_car_state.pb.h"

namespace delphyne {

// @brief A system that takes a pose and a velocity vectors from a maliput rail
// car and generates a SimpleCarState message.
class PoseAndVelToSimpleCarState : public drake::systems::LeafSystem<double> {
 public:
  PoseAndVelToSimpleCarState();

  /// @brief Returns the output port of the system.
  const drake::systems::OutputPort<double>& simple_car_state_output() const {
    return this->get_output_port(output_port_index_);
  }

 protected:
  /// @brief Calculates the SimpleCarState message based on the given context.
  void CalcSimpleCarState(const drake::systems::Context<double>& context,
                          ignition::msgs::SimpleCarState* output) const;

 private:
  int pose_input_port_index_{};

  int velocity_input_port_index_{};

  int output_port_index_{};
};

}  // namespace delphyne
