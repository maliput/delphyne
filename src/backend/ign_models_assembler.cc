// Copyright 2018 Toyota Research Institute

#include "backend/ign_models_assembler.h"

#include <map>

#include <drake/common/eigen_types.h>
#include <drake/systems/rendering/pose_bundle.h>

namespace delphyne {

IgnModelsAssembler::IgnModelsAssembler() {
  models_input_port_index_ =
      DeclareAbstractInputPort(drake::systems::kUseDefaultName, drake::Value<ignition::msgs::Model_V>()).get_index();
  states_input_port_index_ = DeclareAbstractInputPort(drake::systems::kUseDefaultName,
                                                      drake::Value<drake::systems::rendering::PoseBundle<double>>())
                                 .get_index();

  DeclareAbstractOutputPort(&IgnModelsAssembler::CalcAssembledIgnModelVMessage);
}

void IgnModelsAssembler::CalcAssembledIgnModelVMessage(const drake::systems::Context<double>& context,
                                                       ignition::msgs::Model_V* output_models) const {
  // Ensures provided output models message is cleared.
  output_models->Clear();

  // Retrieves models and states inputs.
  const ignition::msgs::Model_V* input_models =
      this->template EvalInputValue<ignition::msgs::Model_V>(context, models_input_port_index_);

  const drake::systems::rendering::PoseBundle<double>* input_states =
      this->template EvalInputValue<drake::systems::rendering::PoseBundle<double>>(context, states_input_port_index_);

  // Copies timestamp from input to output.
  output_models->mutable_header()->mutable_stamp()->CopyFrom(input_models->header().stamp());

  // Reverses the pose bundle index to model ID mapping
  // for convenience.
  std::map<int, int> index_per_model_id;
  for (int i = 0; i < input_states->get_num_poses(); ++i) {
    const int id = input_states->get_model_instance_id(i);
    index_per_model_id[id] = i;
  }

  // Iterates over input models, copying them over to the output while
  // stamping the model based on the model ID to pose mapping.
  for (const ignition::msgs::Model& input_model : input_models->models()) {
    const int index = index_per_model_id[input_model.id()];
    ignition::msgs::Model* output_model = output_models->add_models();
    output_model->CopyFrom(input_model);
    output_model->set_name(input_states->get_name(index));
    ignition::msgs::Pose* ign_pose = output_model->mutable_pose();
    const drake::Isometry3<double>& pose = input_states->get_pose(index);
    const drake::Vector3<double>& position = pose.translation();
    ign_pose->mutable_position()->set_x(position.x());
    ign_pose->mutable_position()->set_y(position.y());
    ign_pose->mutable_position()->set_z(position.z());
    const drake::Quaternion<double> orientation(pose.linear());
    ign_pose->mutable_orientation()->set_x(orientation.x());
    ign_pose->mutable_orientation()->set_y(orientation.y());
    ign_pose->mutable_orientation()->set_z(orientation.z());
    ign_pose->mutable_orientation()->set_w(orientation.w());
  }
}

}  // namespace delphyne
