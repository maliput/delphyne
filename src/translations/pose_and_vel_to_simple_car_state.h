// Copyright 2018 Toyota Research Institute

#pragma once

#include <drake/systems/framework/leaf_system.h>

#include "delphyne/protobuf/simple_car_state.pb.h"

namespace delphyne {

/// @brief A system that takes a pose and a velocity vectors from a car and
/// generates a SimpleCarState message.
class PoseAndVelToSimpleCarState : public drake::systems::LeafSystem<double> {
 public:
  PoseAndVelToSimpleCarState();

  /// @brief Returns the output port of the system.
  const drake::systems::OutputPort<double>& get_simple_car_state_output()
      const {
    return this->get_output_port(output_port_index_);
  }

  /// @brief Returns the input port descriptor for pose. This port expects a
  /// drake::sytems::rendering::PoseVector<double>.
  const drake::systems::InputPortDescriptor<double>& get_pose_input_port()
      const {
    return get_input_port(pose_input_port_index_);
  }

  /// @brief Returns the input port descriptor for velocity. This port expects a
  /// drake::sytems::rendering::FrameVelocity<double>.
  const drake::systems::InputPortDescriptor<double>& get_velocity_input_port()
      const {
    return get_input_port(velocity_input_port_index_);
  }

 protected:
  /// @brief Calculates the SimpleCarState message based on the given context.
  void CalcSimpleCarState(const drake::systems::Context<double>& context,
                          ignition::msgs::SimpleCarState* output) const;

 private:
  // @brief The pose input port index assigned by the system.
  int pose_input_port_index_{};

  // @brief The velocity input port index assigned by the system.
  int velocity_input_port_index_{};

  // @brief The abstract output port index assigned by the system.
  int output_port_index_{};
};

}  // namespace delphyne
