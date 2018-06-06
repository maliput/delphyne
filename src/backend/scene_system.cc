// Copyright 2017 Toyota Research Institute

#include "backend/scene_system.h"

#include <algorithm>

#include <ignition/common/Console.hh>

#include "backend/system.h"

namespace delphyne {

template <class T>
using ProtobufIterator = google::protobuf::internal::RepeatedPtrIterator<T>;

SceneSystem::SceneSystem() {
  geometry_models_input_port_index_ = DeclareAbstractInputPort().get_index();
  updated_pose_models_input_port_index_ =
      DeclareAbstractInputPort().get_index();

  DeclareAbstractOutputPort(&SceneSystem::CalcSceneMessage);
}

void SceneSystem::CalcSceneMessage(
    const drake::systems::Context<double>& context,
    ignition::msgs::Scene* scene_message) const {
  DELPHYNE_DEMAND(scene_message != nullptr);

  // Clears old scene state from the previous CalcSceneMessage call.
  // @see DeclareAbstractOutputPort
  scene_message->Clear();

  const drake::systems::AbstractValue* geometry_input =
      EvalAbstractInput(context, geometry_models_input_port_index_);
  const auto& geometry_models =
      geometry_input->GetValue<ignition::msgs::Model_V>();

  const drake::systems::AbstractValue* updated_pose_input =
      EvalAbstractInput(context, updated_pose_models_input_port_index_);
  const auto& updated_pose_models =
      updated_pose_input->GetValue<ignition::msgs::Model_V>();

  // The scene timestamp is that of the poses update.
  scene_message->mutable_header()->mutable_stamp()->CopyFrom(
      updated_pose_models.header().stamp());

  // All of the geometry models are added to the scene.
  for (const ignition::msgs::Model& geometry_model : geometry_models.models()) {
    scene_message->add_model()->CopyFrom(geometry_model);
  }

  // And those models for which a pose update exists are then updated.
  for (const ignition::msgs::Model& updated_pose_model :
       updated_pose_models.models()) {
    // Finds the matching scene model.
    const ProtobufIterator<ignition::msgs::Model>& matching_scene_model =
        std::find_if(
            scene_message->mutable_model()->begin(),
            scene_message->mutable_model()->end(),
            [&updated_pose_model](const ::ignition::msgs::Model& scene_model) {
              return scene_model.id() == updated_pose_model.id();
            });

    if (matching_scene_model == scene_message->mutable_model()->end()) {
      ignerr << "No geometry model for updated pose with model id "
             << updated_pose_model.id() << std::endl;
      continue;
    }

    // Updates the model itself.
    matching_scene_model->set_name(updated_pose_model.name());
    matching_scene_model->mutable_pose()->CopyFrom(updated_pose_model.pose());

    // Updates each model link.
    for (const ignition::msgs::Link& updated_pose_link :
         updated_pose_model.link()) {
      // Finds the corresponding link inside the scene model.
      const ProtobufIterator<ignition::msgs::Link>& matching_scene_link =
          std::find_if(
              matching_scene_model->mutable_link()->begin(),
              matching_scene_model->mutable_link()->end(),
              [&updated_pose_link](const ::ignition::msgs::Link& scene_link) {
                return scene_link.name() == updated_pose_link.name();
              });

      if (matching_scene_link == matching_scene_model->mutable_link()->end()) {
        ignerr << "No geometry link for updated pose with model id "
               << updated_pose_model.id() << " and link name "
               << updated_pose_link.name() << std::endl;
        continue;
      }

      // Applies the pose to the scene link.
      matching_scene_link->mutable_pose()->mutable_position()->CopyFrom(
          updated_pose_link.pose().position());
      matching_scene_link->mutable_pose()->mutable_orientation()->CopyFrom(
          updated_pose_link.pose().orientation());
    }
  }

  // TODO(caguero): Populate the rest of the scene fields, such as lights.
  // See https://github.com/ToyotaResearchInstitute/delphyne/issues/204
}

}  // namespace delphyne
