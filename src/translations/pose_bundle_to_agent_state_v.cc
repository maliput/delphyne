// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "translations/pose_bundle_to_agent_state_v.h"

#include <algorithm>

#include <drake/common/eigen_types.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/rendering/pose_bundle.h>
#include <maliput/common/maliput_unused.h>

#include "delphyne/protobuf/agent_state.pb.h"
#include "delphyne/protobuf/agent_state_v.pb.h"
#include "translations/drake_to_ign.h"

namespace delphyne {

void PoseBundleToAgentState_V::DoDrakeToIgnTranslation(
    const drake::systems::rendering::PoseBundle<double>& drake_message, ignition::msgs::AgentState_V* ign_message,
    int64_t time_ms) const {
  maliput::common::unused(time_ms);
  // Clears state from the previous call.
  // @see DrakeToIgn::DoDrakeToIgnTranslation
  ign_message->Clear();

  for (int i = 0; i < drake_message.get_num_poses(); ++i) {
    // Gets the car's pose.
    const drake::Isometry3<double>& pose = drake_message.get_transform(i).GetAsIsometry3();
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
