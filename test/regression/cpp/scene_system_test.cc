// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2017-2022, Toyota Research Institute. All rights reserved.
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

#include "backend/scene_system.h"

#include <algorithm>
#include <chrono>
#include <thread>

#include <drake/systems/framework/framework_common.h>
#include <gtest/gtest.h>

#include "test_utilities/helpers.h"

namespace delphyne {

// Checks that a scene system is created, based on the Model_V on the system's
// input port.
GTEST_TEST(SceneSystemTest, CalcSceneTest) {
  const ignition::msgs::Model_V updated_pose_models{test::BuildPreloadedModelVMsg()};

  // Create a set of geometry models, equal to the updated pose models, but with
  // null pose on each link.
  ignition::msgs::Model_V geometry_models;
  for (const ignition::msgs::Model& updated_pose_model : updated_pose_models.models()) {
    ignition::msgs::Model* new_model = geometry_models.add_models();
    new_model->CopyFrom(updated_pose_model);

    for (ignition::msgs::Link& new_model_link : *new_model->mutable_link()) {
      auto position = new_model_link.mutable_pose()->mutable_position();
      position->set_x(0);
      position->set_y(0);
      position->set_z(0);

      auto orientation = new_model_link.mutable_pose()->mutable_orientation();
      orientation->set_x(0);
      orientation->set_y(0);
      orientation->set_z(0);
      orientation->set_w(1);
    }
  }

  SceneSystem scene_system;
  std::unique_ptr<drake::systems::Context<double>> context = scene_system.AllocateContext();

  context->FixInputPort(scene_system.get_geometry_models_input_port_index(),
                        drake::Value<ignition::msgs::Model_V>(geometry_models));

  context->FixInputPort(scene_system.get_updated_pose_models_input_port_index(),
                        drake::Value<ignition::msgs::Model_V>(updated_pose_models));

  std::unique_ptr<drake::systems::SystemOutput<double>> output = scene_system.AllocateOutput();
  scene_system.CalcOutput(*context, output.get());

  const double kOutputPortIndex{0};
  const auto& scene_msg = output->get_data(kOutputPortIndex)->get_value<ignition::msgs::Scene>();

  // The geometry models should have been updated to include the pose of the
  // updated pose models.
  EXPECT_EQ(updated_pose_models.models_size(), scene_msg.model_size());

  for (const ::ignition::msgs::Model& updated_pose_model : updated_pose_models.models()) {
    const google::protobuf::internal::RepeatedPtrIterator<const ignition::msgs::Model>& matching_scene_model =
        std::find_if(scene_msg.model().begin(), scene_msg.model().end(),
                     [updated_pose_model](const ::ignition::msgs::Model& scene_model) {
                       return scene_model.id() == updated_pose_model.id();
                     });

    ASSERT_FALSE(matching_scene_model == scene_msg.model().end());
    EXPECT_TRUE(test::CheckProtobufMsgEquality(updated_pose_model, *matching_scene_model));
  }
}

}  // namespace delphyne
