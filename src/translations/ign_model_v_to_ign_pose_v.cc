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

#include "translations/ign_model_v_to_ign_pose_v.h"

namespace delphyne {

void IgnModelVToIgnPoseV::DoIgnToDrakeTranslation(const ignition::msgs::Model_V& ign_model_v,
                                                  ignition::msgs::Pose_V* ign_pose_v) const {
  // Clears state from the previous call.
  // @see IgnToDrake::DoIgnToDrakeTranslation
  ign_pose_v->Clear();

  // Add model and link poses
  for (int i = 0; i < ign_model_v.models_size(); ++i) {
    const ::ignition::msgs::Model& robot_model = ign_model_v.models(i);

    // Add model pose, using the model index as its pose id
    {
      ignition::msgs::Pose* pose = ign_pose_v->add_pose();
      pose->CopyFrom(robot_model.pose());
      pose->set_id(i);
    }

    // Add link poses, whose ids should already be set
    for (int l = 0; l < robot_model.link_size(); ++l) {
      ignition::msgs::Pose* pose = ign_pose_v->add_pose();
      pose->CopyFrom(robot_model.link(l).pose());
    }
  }
}

}  // namespace delphyne
