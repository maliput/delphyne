/**
 * @file src/agents/velocity_controller.h
 *
 * Copyright 2018 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#pragma once

#include <drake/systems/framework/leaf_system.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/// @brief A simple controller that computes an acceleration from velocities.
///
/// The controller has two inputs; the commanded velocity, and the feedback
/// from the last computed velocity.  Based on how far away from the commanded
/// velocity the current velocity is, produces an acceleration output.
template <typename T>
class VelocityController : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VelocityController);

  /// @brief Default Constructor
  VelocityController();

  /// Getter methods for input and output ports.
  /// @{
  const drake::systems::InputPortDescriptor<T>& command_input() const;
  const drake::systems::InputPortDescriptor<T>& feedback_input() const;
  const drake::systems::OutputPort<T>& acceleration_output() const;
  /// @}

 private:
  // Callback for calculating the output acceleration based on the commanded and
  // feedback velocities.
  void CalcOutputAcceleration(const drake::systems::Context<T>& context,
                              drake::systems::BasicVector<T>* output) const;

  /********************
   * System Indices
   *******************/
  int velocity_command_input_port_index_;
  int velocity_feedback_input_port_index_;
  int accel_output_port_index_;
};

}  // namespace delphyne
