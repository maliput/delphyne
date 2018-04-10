// Copyright 2017 Toyota Research Institute

#include "backend/scene_system.h"

#include <algorithm>

#include <ignition/common/Console.hh>

namespace delphyne {
namespace backend {

SceneSystem::SceneSystem() {
  geometry_models_input_port_index = DeclareAbstractInputPort().get_index();
  updated_pose_models_input_port_index = DeclareAbstractInputPort().get_index();

  DeclareAbstractOutputPort(&SceneSystem::CalcSceneMessage);
}

void SceneSystem::GetScene(ignition::msgs::Scene* scene) const {}

void SceneSystem::CalcSceneMessage(
    const drake::systems::Context<double>& context,
    ignition::msgs::Scene* scene_message) const {
  DELPHYNE_DEMAND(scene_message != nullptr);

  // Clears old scene state from the previous CalcSceneMessage call.
  // @see DeclareAbstractOutputPort
  scene_message->Clear();

  const drake::systems::AbstractValue* geometry_input =
      EvalAbstractInput(context, geometry_models_input_port_index);
  const auto& geometry_models =
      geometry_input->GetValue<ignition::msgs::Model_V>();

  const drake::systems::AbstractValue* updated_pose_input =
      EvalAbstractInput(context, updated_pose_models_input_port_index);
  const auto& updated_pose_models =
      updated_pose_input->GetValue<ignition::msgs::Model_V>();

  // The scene timestamp is that of the poses update.
  scene_message->mutable_header()->mutable_stamp()->CopyFrom(
      updated_pose_models.header().stamp());

  // The scene is then built from the geometry models, which are updated using
  // the updated poses.
  for (const ignition::msgs::Model& geometry_model : geometry_models.models()) {
    ignition::msgs::Model* updated_geometry_model = scene_message->add_model();
    updated_geometry_model->CopyFrom(geometry_model);

    // Finds the updated pose model for the model.
    const google::protobuf::internal::RepeatedPtrIterator<
        const ignition::msgs::Model>& matching_updated_pose_model =
        std::find_if(updated_pose_models.models().begin(),
                     updated_pose_models.models().end(),
                     [updated_geometry_model](
                         const ::ignition::msgs::Model& updated_pose_model) {
                       return updated_pose_model.id() ==
                              updated_geometry_model->id();
                     });

    if (matching_updated_pose_model == updated_pose_models.models().end()) {
      ignerr << "No updated pose for model id " << updated_geometry_model->id()
             << std::endl;
      continue;
    }

    // Updates each model link.
    for (ignition::msgs::Link& updated_geometry_link :
         *updated_geometry_model->mutable_link()) {
      // Find the corresponding link inside the updated pose model.
      const google::protobuf::internal::RepeatedPtrIterator<
          const ignition::msgs::Link>& matching_updated_pose_link =
          std::find_if(matching_updated_pose_model->link().begin(),
                       matching_updated_pose_model->link().end(),
                       [updated_geometry_link](
                           const ::ignition::msgs::Link& updated_pose_link) {
                         return updated_pose_link.name() ==
                                updated_geometry_link.name();
                       });

      if (matching_updated_pose_link ==
          matching_updated_pose_model->link().end()) {
        ignerr << "No updated pose for link name "
               << updated_geometry_link.name() << std::endl;
        continue;
      }

      // Gets the pose and applies it to the link of the geometry model being
      // updated.
      updated_geometry_link.mutable_pose()->mutable_position()->CopyFrom(
          matching_updated_pose_link->pose().position());
      updated_geometry_link.mutable_pose()->mutable_orientation()->CopyFrom(
          matching_updated_pose_link->pose().orientation());
    }
  }

  // TODO(caguero): Populate the rest of the scene fields, such as lights.
  // See https://github.com/ToyotaResearchInstitute/delphyne/issues/204
}

}  // namespace backend
}  // namespace delphyne
