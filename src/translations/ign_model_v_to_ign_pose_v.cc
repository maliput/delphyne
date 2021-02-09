// Copyright 2018 Toyota Research Institute

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
