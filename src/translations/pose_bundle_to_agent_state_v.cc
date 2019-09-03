// Copyright 2018 Toyota Research Institute

#include "translations/pose_bundle_to_agent_state_v.h"

#include <algorithm>

#include <drake/common/eigen_types.h>
#include <drake/systems/rendering/pose_bundle.h>

#include "delphyne/protobuf/agent_state.pb.h"
#include "delphyne/protobuf/agent_state_v.pb.h"
#include "translations/drake_to_ign.h"

namespace delphyne {

const unsigned int kPoseBundleVectorSize{0};

void PoseBundleToAgentState_V::DoDrakeToIgnTranslation(
    const drake::systems::rendering::PoseBundle<double>& drake_message, ignition::msgs::AgentState_V* ign_message,
    int64_t time_ms) const {
  // Clears state from the previous call.
  // @see DrakeToIgn::DoDrakeToIgnTranslation
  ign_message->Clear();

  for (int i = 0; i < drake_message.get_num_poses(); ++i) {
    // Gets the car's pose.
    const drake::Isometry3<double>& pose = drake_message.get_pose(i);
    // Gets the agent's orientation in the form of a vector of euler angles
    // following the x-y-z convention (roll-pitch-yaw).
    // As the transformation is an isometry, its linear part is solely comprised
    // of a rotation i.e. no scaling nor shearing.
    // The indexes of 0, 1 and 2 represent the x, y and z axis, respectively.
    const Eigen::Vector3d euler_rotation = pose.linear().eulerAngles(0, 1, 2);

    // Calculates car's velocity.
    const drake::multibody::SpatialVelocity<double>& spatial_velocity = drake_message.get_velocity(i).get_velocity();
    const drake::Vector3<double>& rotational = spatial_velocity.rotational();
    const drake::Vector3<double>& translational = spatial_velocity.translational();

    // Appends a new state to the vector.
    ignition::msgs::AgentState* current_state = ign_message->add_states();
    current_state->set_name("/agent/" + drake_message.get_name(i) + "/state");
    current_state->mutable_position()->set_x(pose.translation().x());
    current_state->mutable_position()->set_y(pose.translation().y());
    current_state->mutable_position()->set_z(pose.translation().z());

    current_state->mutable_orientation()->set_roll(euler_rotation(0));
    current_state->mutable_orientation()->set_pitch(euler_rotation(1));
    current_state->mutable_orientation()->set_yaw(euler_rotation(2));

    current_state->mutable_linear_velocity()->set_x(translational(0));
    current_state->mutable_linear_velocity()->set_y(translational(1));
    current_state->mutable_linear_velocity()->set_z(translational(2));

    current_state->mutable_angular_velocity()->set_x(rotational(0));
    current_state->mutable_angular_velocity()->set_y(rotational(1));
    current_state->mutable_angular_velocity()->set_z(rotational(2));
  }
}

}  // namespace delphyne
