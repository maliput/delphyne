// Copyright 2018 Toyota Research Institute

#include "translations/pose_bundle_to_simple_car_state_v.h"

#include <algorithm>

#include <drake/common/eigen_types.h>
#include <drake/systems/rendering/pose_bundle.h>

#include "delphyne/protobuf/simple_car_state_v.pb.h"

namespace delphyne {

using drake::systems::rendering::PoseBundle;

PoseBundleToSimpleCarState_V::PoseBundleToSimpleCarState_V() {
  const int kPoseBundleVectorSize{0};
  // Declares input ports.
  pose_bundle_input_port_index_ =
      this->DeclareInputPort(drake::systems::kVectorValued,
                             kPoseBundleVectorSize)
          .get_index();
  // Declares output port.
  output_port_index_ =
      this->DeclareAbstractOutputPort(
              &PoseBundleToSimpleCarState_V::CalcSimpleCarState_V)
          .get_index();
}

void PoseBundleToSimpleCarState_V::CalcSimpleCarState_V(
    const drake::systems::Context<double>& context,
    ignition::msgs::SimpleCarState_V* output) const {
  // Obtains the input and output.
  const PoseBundle<double>& pose_bundle =
      drake::systems::System<double>::EvalAbstractInput(
          context, pose_bundle_input_port_index_)
          ->template GetValue<PoseBundle<double>>();

  // Clears the current SimpleCarState_V message from its previous content.
  output->Clear();

  for (int i = 0; i < pose_bundle.get_num_poses(); ++i) {
    // Gets the car's pose.
    const drake::Isometry3<double>& pose = pose_bundle.get_pose(i);
    // Translates pose from quaternion to euler.
    const Eigen::Vector3d& euler_rotation =
        pose.rotation().eulerAngles(0, 1, 2);

    // Calculates car's velocity.
    const drake::systems::rendering::FrameVelocity<double>& velocity =
        pose_bundle.get_velocity(i);
    const drake::multibody::SpatialVelocity<double>& spatial_velocity =
        velocity.get_velocity();
    const double& velocity_norm =
        static_cast<double>(spatial_velocity.translational().norm());

    // Appends a new state to the vector.
    auto* current_state = output->add_states();
    current_state->set_name("/agent/" + pose_bundle.get_name(i) + "/state");
    current_state->set_x(pose.translation().x());
    current_state->set_y(pose.translation().y());
    current_state->set_heading(euler_rotation(2));
    current_state->set_velocity(velocity_norm);
  }
}

}  // namespace delphyne
