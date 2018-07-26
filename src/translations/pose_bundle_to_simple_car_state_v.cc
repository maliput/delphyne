// Copyright 2018 Toyota Research Institute

#include "translations/pose_bundle_to_simple_car_state_v.h"

#include <algorithm>

#include <drake/common/eigen_types.h>
#include <drake/systems/rendering/pose_bundle.h>

#include "delphyne/protobuf/simple_car_state_v.pb.h"
#include "delphyne/protobuf/simple_car_state.pb.h"
#include "translations/drake_to_ign.h"

namespace delphyne {

const unsigned int kPoseBundleVectorSize{0};

void PoseBundleToSimpleCarState_V::DoDrakeToIgnTranslation(
    const drake::systems::rendering::PoseBundle<double>& drake_message,
    ignition::msgs::SimpleCarState_V* ign_message, int64_t time_ms) const {

  // Clears state from the previous call.
  // @see DrakeToIgn::DoDrakeToIgnTranslation
  ign_message->Clear();

  for (int i = 0; i < drake_message.get_num_poses(); ++i) {
    // Gets the car's pose.
    const drake::Isometry3<double>& pose = drake_message.get_pose(i);
    // Translates pose from quaternion to euler.
    const Eigen::Vector3d euler_rotation =
        pose.rotation().eulerAngles(0, 1, 2);

    // Calculates car's velocity.
    const drake::systems::rendering::FrameVelocity<double> velocity =
        drake_message.get_velocity(i);
    const drake::multibody::SpatialVelocity<double> spatial_velocity =
        velocity.get_velocity();
    const double velocity_norm =
        static_cast<double>(spatial_velocity.translational().norm());

    // Appends a new state to the vector.
    ignition::msgs::SimpleCarState* current_state = ign_message->add_states();
    current_state->set_name(
        "/agent/" + std::to_string(drake_message.get_model_instance_id(i)) +
        "/state");
    current_state->set_x(pose.translation().x());
    current_state->set_y(pose.translation().y());
    current_state->set_heading(euler_rotation(2));
    current_state->set_velocity(velocity_norm);
  }
}

}  // namespace delphyne
