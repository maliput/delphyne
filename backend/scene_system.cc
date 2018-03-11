// Copyright 2017 Toyota Research Institute

#include <ignition/common/Time.hh>

#include "backend/scene_system.h"
#include "backend/time_conversion.h"

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

  ignition::msgs::Scene scene_msg;

  // Stamp the message.
  auto t = drake::ExtractDoubleOrThrow(context.get_time());
  *scene_msg.mutable_header()->mutable_stamp() = SecsToIgnitionTime(t);

  // Populate the list of models.
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
