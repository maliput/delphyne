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
    const drake::Isometry3<double>& pose = input_states->get_transform(index).GetAsIsometry3();
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
