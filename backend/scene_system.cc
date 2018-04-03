// Copyright 2017 Toyota Research Institute

#include "backend/scene_system.h"

namespace delphyne {
namespace backend {

SceneSystem::SceneSystem() {
  DeclareAbstractInputPort();
  DeclareAbstractOutputPort(&SceneSystem::CalcSceneMessage);
}

void SceneSystem::CalcSceneMessage(
    const drake::systems::Context<double>& context,
    ignition::msgs::Scene* scene_message) const {
  DELPHYNE_DEMAND(scene_message != nullptr);

  // Clears old scene state from the previous CalcSceneMessage call.
  // @see DeclareAbstractOutputPort
  scene_message->Clear();

  const drake::systems::AbstractValue* input = EvalAbstractInput(context, 0);
  const auto& models = input->GetValue<ignition::msgs::Model_V>();

  scene_message->mutable_header()->mutable_stamp()->CopyFrom(
      models.header().stamp());

  for (int i = 0; i < models.models_size(); ++i) {
    ignition::msgs::Model* new_model = scene_message->add_model();
    new_model->CopyFrom(models.models(i));
  }

  // TODO(caguero): Populate the rest of the scene fields, such as lights.
  // See https://github.com/ToyotaResearchInstitute/delphyne/issues/204
}

}  // namespace backend
}  // namespace delphyne
