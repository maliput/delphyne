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
  const drake::systems::rendering::FrameVelocity<T>* feedback_input =
      this->template EvalVectorInput<drake::systems::rendering::FrameVelocity>(
          context, velocity_feedback_input_port_index_);

  if (feedback_input == nullptr) {
    // If the feedback is not connected, we can't do anything, so leave
    // acceleration at 0
    output->SetAtIndex(0, 0.0);
    return;
  }

  const drake::multibody::SpatialVelocity<T> vel =
      feedback_input->get_velocity();

  // Let's calculate the magnitude of the vector to get an estimate
  // of our forward velocity.
  double magnitude = vel.translational().norm();

  const drake::systems::BasicVector<T>* command_input =
      this->template EvalVectorInput<drake::systems::BasicVector>(
          context, velocity_command_input_port_index_);

  // Allocates and uses a BasicVector containing a zero acceleration command in
  // case the input contains nullptr.
  drake::systems::BasicVector<T> default_input(magnitude);
  if (command_input == nullptr) {
    command_input = &default_input;
  }

  if (command_input->GetAtIndex(0) > magnitude) {
    output->SetAtIndex(0, 10.0);
  } else if (command_input->GetAtIndex(0) < magnitude) {
    output->SetAtIndex(0, -10.0);
  } else {
    output->SetAtIndex(0, 0.0);
  }
}

template class VelocityController<double>;

}  // namespace delphyne
