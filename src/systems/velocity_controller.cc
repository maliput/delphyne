/**
 * @file src/agents/velocity_controller.cc
 *
 * Copyright 2018 Toyota Research Institute
 */

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "systems/velocity_controller.h"

#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/rendering/frame_velocity.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

template <typename T>
VelocityController<T>::VelocityController() {
  velocity_command_input_port_index_ =
      this->DeclareVectorInputPort(drake::systems::BasicVector<T>(1))
          .get_index();
  velocity_feedback_input_port_index_ =
      this->DeclareVectorInputPort(
              drake::systems::rendering::FrameVelocity<T>())
          .get_index();
  accel_output_port_index_ =
      this->DeclareVectorOutputPort(drake::systems::BasicVector<T>(1),
                                    &VelocityController::CalcOutputAcceleration)
          .get_index();
}

template <typename T>
const drake::systems::InputPortDescriptor<T>&
VelocityController<T>::command_input() const {
  return this->get_input_port(velocity_command_input_port_index_);
}

template <typename T>
const drake::systems::OutputPort<T>&
VelocityController<T>::acceleration_output() const {
  return this->get_output_port(accel_output_port_index_);
}

template <typename T>
const drake::systems::InputPortDescriptor<T>&
VelocityController<T>::feedback_input() const {
  return this->get_input_port(velocity_feedback_input_port_index_);
}

template <typename T>
void VelocityController<T>::CalcOutputAcceleration(
    const drake::systems::Context<T>& context,
    drake::systems::BasicVector<T>* output) const {
  Eigen::VectorXd constant_value(1);

  const drake::systems::rendering::FrameVelocity<T>* feedback_input =
      this->template EvalVectorInput<drake::systems::rendering::FrameVelocity>(
          context, velocity_feedback_input_port_index_);

  if (feedback_input == nullptr) {
    // If the feedback is not connected, we can't do anything, so leave
    // acceleration at 0
    constant_value[0] = 0.0;
    output->set_value(constant_value);
    return;
  }

  drake::multibody::SpatialVelocity<T> vel = feedback_input->get_velocity();

  // Let's calculate the magnitude of the vector to get an estimate
  // of our forward velocity.
  double magnitude =
      std::sqrt(vel[3] * vel[3] + vel[4] * vel[4] + vel[5] * vel[5]);

  const drake::systems::BasicVector<T>* command_input =
      this->template EvalVectorInput<drake::systems::BasicVector>(
          context, velocity_command_input_port_index_);

  // Allocates and uses a BasicVector containing a zero acceleration command in
  // case the input contains nullptr.
  const auto default_input = drake::systems::BasicVector<T>::Make(magnitude);
  if (command_input == nullptr) {
    command_input = default_input.get();
  }

  if (command_input->GetAtIndex(0) > magnitude) {
    constant_value[0] = 10.0;
  } else if (command_input->GetAtIndex(0) < magnitude) {
    constant_value[0] = -10.0;
  } else {
    constant_value[0] = 0.0;
  }
  output->set_value(constant_value);
}

template class VelocityController<double>;

}  // namespace delphyne
