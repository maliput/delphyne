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

#include "backend/scene_system.h"

using drake::systems::AbstractValue;
using drake::systems::Context;
using drake::systems::PublishEvent;

namespace delphyne {
namespace backend {

SceneSystem::SceneSystem(const std::string& topic_name) : topic_(topic_name) {
  // TODO(caguero): In the future, this system won't publish the scene message
  // directly. Instead, it will declare an output port that will contain the
  // scene message. Later, we'll use a separate system for publishing.
  DeclareAbstractInputPort();
  publisher_ = node_.Advertise<ignition::msgs::Scene>(topic_);
}

SceneSystem::~SceneSystem() {}

void SceneSystem::DoPublish(
    const Context<double>& context,
    const std::vector<const PublishEvent<double>*>&) const {
  // Check if it's time to update the scene.
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = now - last_scene_update_;
  if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() <
      kScenePeriodMs_) {
    return;
  }

  // It's time to update the scene!
  last_scene_update_ = now;

  const AbstractValue* input = EvalAbstractInput(context, 0);
  const auto& viewer_draw = input->GetValue<drake::lcmt_viewer_draw>();

  // Populate the list of models.
  ignition::msgs::Scene scene_msg;
  ignition::msgs::Model_V models;
  lcmToIgn(viewer_draw, &models);
  for (int i = 0; i < models.models_size(); ++i) {
    auto newModel = scene_msg.add_model();
    newModel->CopyFrom(models.models(i));
  }

  // TODO(caguero): Populate the rest of the scene fields, such as lights.
  // See https://github.com/ToyotaResearchInstitute/delphyne/issues/204

  // Publishes onto the specified ign-transport channel.
  publisher_.Publish(scene_msg);
}

}  // namespace backend
}  // namespace delphyne
