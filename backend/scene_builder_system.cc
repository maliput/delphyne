// Copyright 2018 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <string>

#include <ignition/common/Console.hh>

#include <drake/lcmt_viewer_draw.hpp>

#include "backend/scene_builder_system.h"

#include "bridge/lcm_to_ign_translation.h"

namespace delphyne {
namespace backend {

SceneBuilderSystem::SceneBuilderSystem() { this->DeclareAbstractInputPort(); }

void SceneBuilderSystem::DoPublish(
    const drake::systems::Context<double>& context,
    const std::vector<const drake::systems::PublishEvent<double>*>&) const {
  const PoseBundle<double>& poses =
      this->EvalAbstractInput(context, 0)
          ->template GetValue<PoseBundle<double>>();
  last_poses_update_ = std::make_unique<PoseBundle<double>>(poses);
}

void SceneBuilderSystem::UpdateModels(ignition::msgs::Model_V* robotModels) {
  const int n = last_poses_update_->get_num_poses();

  for (int pose_index = 0; pose_index < n; ++pose_index) {
    // Find the corresponding model for the pose.
    const auto robot_id = last_poses_update_->get_model_instance_id(pose_index);

    ignition::msgs::Model* model = nullptr;

    for (int j = 0; j < robotModels->models_size(); ++j) {
      if (robotModels->models(j).id() == int64_t(robot_id)) {
        model = robotModels->mutable_models(j);
        break;
      }
    }

    if (model == nullptr) {
      ignerr << "No model matching id " << robot_id << std::endl;
      return;
    }

    // Find the corresponding link for the pose.
    const std::string& link_name = last_poses_update_->get_name(pose_index);

    ignition::msgs::Link* link = nullptr;

    for (int j = 0; j < model->link_size(); ++j) {
      if (model->link(j).name() == link_name) {
        link = model->mutable_link(j);
        break;
      }
    }

    if (link == nullptr) {
      ignerr << "No link matching name " << link_name << std::endl;
      return;
    }

    // Get the pose and apply it to the model
    ignition::msgs::Pose* pose = link->mutable_pose();

    const Eigen::Translation<double, 3> translation(
        last_poses_update_->get_pose(pose_index).translation());
    ignition::msgs::Vector3d* position = pose->mutable_position();
    position->set_x(translation.x());
    position->set_y(translation.y());
    position->set_z(translation.z());

    const Eigen::Quaternion<double> quaternion(
        last_poses_update_->get_pose(pose_index).linear());
    ignition::msgs::Quaternion* orientation = pose->mutable_orientation();
    orientation->set_w(quaternion.w());
    orientation->set_x(quaternion.x());
    orientation->set_y(quaternion.y());
    orientation->set_z(quaternion.z());
  }
}

}  // namespace backend
}  // namespace delphyne
