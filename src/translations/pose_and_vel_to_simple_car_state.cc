// Copyright 2018 Toyota Research Institute

#include "translations/pose_and_vel_to_simple_car_state.h"

#include <algorithm>

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/vector_base.h>

#include "delphyne/protobuf/simple_car_state.pb.h"

namespace delphyne {

PoseAndVelToSimpleCarState::PoseAndVelToSimpleCarState() {
  const int kPoseVectorSize{7};
  const int kVelocityVectorSize{6};
  // Declares input ports.
  pose_input_port_index_ =
      this->DeclareInputPort(drake::systems::kVectorValued, kPoseVectorSize)
          .get_index();
  velocity_input_port_index_ =
      this->DeclareInputPort(drake::systems::kVectorValued, kVelocityVectorSize)
          .get_index();
  // Declares output port.
  output_port_index_ = this->DeclareAbstractOutputPort(
                               &PoseAndVelToSimpleCarState::CalcSimpleCarState)
                           .get_index();
}

void PoseAndVelToSimpleCarState::CalcSimpleCarState(
    const drake::systems::Context<double>& context,
    ignition::msgs::SimpleCarState* output) const {
  // Obtains car pose as a VectorBase.
  const drake::systems::VectorBase<double>* const pose_vector =
      EvalVectorInput(context, pose_input_port_index_);
  // Casts VectorBase as PoseVector.
  const drake::systems::rendering::PoseVector<double>* const pose =
      dynamic_cast<const drake::systems::rendering::PoseVector<double>*>(
          pose_vector);
  const Eigen::Translation<double, 3> pose_translation =
      pose->get_translation();
  const Eigen::Quaternion<double> pose_rotation = pose->get_rotation();
  // Translates pose from quaternion to euler.
  const Eigen::Vector3d euler_rotation =
      pose_rotation.toRotationMatrix().eulerAngles(0, 1, 2);

  // Obtains car velocity as a VectorBase
  const drake::systems::VectorBase<double>* const velocity_vector =
      EvalVectorInput(context, velocity_input_port_index_);
  // Casts VectorBase as FrameVelocity.
  const drake::systems::rendering::FrameVelocity<double>* const velocity =
      dynamic_cast<const drake::systems::rendering::FrameVelocity<double>*>(
          velocity_vector);

  const drake::multibody::SpatialVelocity<double> spatial_velocity =
      velocity->get_velocity();
  const double velocity_norm =
      static_cast<double>(spatial_velocity.translational().norm());

  // Creates and fills a SimpleCarState message.
  ignition::msgs::SimpleCarState state{};
  state.set_x(pose_translation.x());
  state.set_y(pose_translation.y());
  state.set_heading(euler_rotation(2));
  state.set_velocity(velocity_norm);
  // Don't allow small negative velocities to escape the state.
  state.set_velocity(std::max(0.0, state.velocity()));

  // Set output value with the generated SimpleCarState message.
  output->CopyFrom(state);
}

}  // namespace delphyne
