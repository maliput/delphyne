// Copyright 2018 Toyota Research Institute

#pragma once

#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

#include "delphyne/macros.h"

namespace delphyne {

/// The SpeedSystem implements a very simple speed controller, taking as an
/// input the current frame velocity (from an InputPort) and the desired speed
/// (from a second InputPort), and producing an acceleration on an OutputPort
/// to reach that speed.
template <typename T>
class SpeedSystem final : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpeedSystem)

  /// Default constructor.
  SpeedSystem() {
    speed_feedback_input_port_index_ =
        this->DeclareVectorInputPort(drake::systems::rendering::FrameVelocity<T>()).get_index();

    command_input_port_index_ = this->DeclareVectorInputPort(drake::systems::BasicVector<T>(1)).get_index();

    accel_output_port_index_ =
        this->DeclareVectorOutputPort(drake::systems::BasicVector<T>(1), &SpeedSystem::CalcOutputAcceleration)
            .get_index();
  }

  ~SpeedSystem() override {}

  const drake::systems::OutputPort<T>& acceleration_output() const {
    return this->get_output_port(accel_output_port_index_);
  }

  const drake::systems::InputPort<T>& feedback_input() const {
    return this->get_input_port(speed_feedback_input_port_index_);
  }

  const drake::systems::InputPort<T>& command_input() const { return this->get_input_port(command_input_port_index_); }

 protected:
  void CalcOutputAcceleration(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const {
    const drake::systems::rendering::FrameVelocity<T>* feedback_input =
        this->template EvalVectorInput<drake::systems::rendering::FrameVelocity>(context,
                                                                                 speed_feedback_input_port_index_);

    if (feedback_input == nullptr) {
      // If the feedback is not connected, we can't do anything, so leave
      // acceleration at 0
      output->SetAtIndex(0, 0.0);
      return;
    }

    const drake::systems::BasicVector<T>* cmd_input =
        this->template EvalVectorInput<drake::systems::BasicVector>(context, command_input_port_index_);

    if (cmd_input == nullptr) {
      // If the command input is not connected, we can't do anything, so leave
      // acceleration at 0
      output->SetAtIndex(0, 0.0);
      return;
    }

    T cmd_speed = cmd_input->GetAtIndex(0);

    if (cmd_speed < 0.0) {
      // If the input_speed is negative, that means we shouldn't do anything in
      // this controller so we just set the acceleration to 0.
      output->SetAtIndex(0, 0.0);
      return;
    }

    const drake::multibody::SpatialVelocity<T> vel = feedback_input->get_velocity();

    // Let's calculate the magnitude of the vector to get an estimate
    // of our forward speed.
    T magnitude = vel.translational().norm();

    if (cmd_speed > magnitude) {
      output->SetAtIndex(0, kAccelerationSetPoint);
    } else if (cmd_speed < magnitude) {
      output->SetAtIndex(0, -kAccelerationSetPoint);
    } else {
      output->SetAtIndex(0, 0.0);
    }
  }

 private:
  // The amount of acceleration that will be applied if the acceleration needs
  // to be changed.
  static constexpr T kAccelerationSetPoint = 10.0;

  /********************
   * System Indices
   *******************/
  int speed_feedback_input_port_index_;
  int accel_output_port_index_;
  int command_input_port_index_;
};

}  // namespace delphyne
