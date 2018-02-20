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

#include "backend/test/helpers.h"

#include <string>

#include <drake/lcmt_viewer_draw.hpp>

#include <gtest/gtest.h>

#include <ignition/msgs.hh>

#include "backend/system.h"

namespace delphyne {
namespace test {

drake::lcmt_viewer_draw BuildPreloadedDrawMsg() {
  drake::lcmt_viewer_draw msg;
  msg.timestamp = 0;
  msg.num_links = 1;
  msg.link_name.resize(msg.num_links);
  msg.link_name[0] = "box";
  msg.robot_num.resize(1);
  msg.robot_num[0] = 1;
  msg.position.resize(1);
  msg.position[0].resize(3);
  msg.position[0][0] = 0.0;
  msg.position[0][1] = 0.0;
  msg.position[0][2] = 0.0;
  msg.quaternion.resize(1);
  msg.quaternion[0].resize(4);
  msg.quaternion[0][0] = 0.0;
  msg.quaternion[0][1] = 0.0;
  msg.quaternion[0][2] = 0.0;
  msg.quaternion[0][3] = 1.0;
  return msg;
}

ignition::msgs::Model_V BuildPreloadedModelVMsg() {
  ignition::msgs::Model_V robot_models;
  robot_models.mutable_header()->mutable_stamp()->set_sec(123);
  robot_models.mutable_header()->mutable_stamp()->set_nsec(456000000);

  for (int i = 0; i < 3; ++i) {
    ::ignition::msgs::Model* model = robot_models.add_models();
    model->set_id(i);

    for (int j = 0; j < 2; ++j) {
      ::ignition::msgs::Link* link = model->add_link();

      ::ignition::msgs::Pose* pose = link->mutable_pose();
      ignition::msgs::Vector3d* position = pose->mutable_position();
      ignition::msgs::Quaternion* orientation = pose->mutable_orientation();

      link->set_name(std::to_string(i) + std::to_string(j));

      position->set_x(i);
      position->set_y(j + 5.0);
      position->set_z(i + 10.0);

      orientation->set_w(i);
      orientation->set_x(j + 5.0);
      orientation->set_y(i + 10.0);
      orientation->set_z(j + 15.0);
    }
  }

  return robot_models;
}

bool AssertLinkNumberEquivalence(const drake::lcmt_viewer_draw& lcm_msg,
                                 const ignition::msgs::Model_V& ign_models) {
  int ign_links = 0;
  for (int i = 0; i < ign_models.models_size(); ++i) {
    const ::ignition::msgs::Model model = ign_models.models(i);
    ign_links += model.link_size();
  }

  return lcm_msg.num_links == ign_links;
}

::testing::AssertionResult CheckPositionTranslation(
    const ignition::msgs::Pose& pose, const drake::lcmt_viewer_draw& lcm_msg,
    int lcm_index, double tolerance) {
  // Asserts that the tolerance is not negative, aborts otherwise.
  DELPHYNE_ASSERT(tolerance >= 0);
  DELPHYNE_ASSERT(lcm_index >= 0);

  std::string error_message;
  bool fails(false);

  double delta = std::abs(pose.position().x() - lcm_msg.position[lcm_index][0]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Values for position are different at x.\n";
  }
  delta = std::abs(pose.position().y() - lcm_msg.position[lcm_index][1]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Values for position are different at y.\n";
  }
  delta = std::abs(pose.position().z() - lcm_msg.position[lcm_index][2]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Values for position are different at z.\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CheckOrientationTranslation(
    const ignition::msgs::Pose& pose, const drake::lcmt_viewer_draw& lcm_msg,
    int lcm_index, double tolerance) {
  DELPHYNE_ASSERT(tolerance >= 0);
  DELPHYNE_ASSERT(lcm_index >= 0);

  std::string error_message;
  bool fails(false);

  double delta =
      std::abs(pose.orientation().w() - lcm_msg.quaternion[lcm_index][0]);
  if (delta > tolerance) {
    fails = true;
    error_message =
        error_message + "Values for orientation are different at w.\n";
  }
  delta = std::abs(pose.orientation().x() - lcm_msg.quaternion[lcm_index][1]);
  if (delta > tolerance) {
    fails = true;
    error_message =
        error_message + "Values for orientation are different at x.\n";
  }
  delta = std::abs(pose.orientation().y() - lcm_msg.quaternion[lcm_index][2]);
  if (delta > tolerance) {
    fails = true;
    error_message =
        error_message + "Values for orientation are different at y.\n";
  }
  delta = std::abs(pose.orientation().z() - lcm_msg.quaternion[lcm_index][3]);
  if (delta > tolerance) {
    fails = true;
    error_message =
        error_message + "Values for orientation are different at z.\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess();
}

namespace {
::testing::AssertionResult AssertModelsEquivalence(
    const ignition::msgs::Model& model, const drake::lcmt_viewer_draw& lcm_msg,
    int i) {
  DELPHYNE_ASSERT(i >= 0);

  bool failure = false;

  // Asserts there is a corresponding ignition link for the LCM link.
  bool found = false;
  std::string error_msg;
  ignition::msgs::Link link;
  for (int j = 0; j < model.link_size(); ++j) {
    if (model.link(j).name() == lcm_msg.link_name[i]) {
      link = model.link(j);
      found = true;
    }
  }
  if (!found) {
    error_msg =
        "Couldn't find a matching link name in the translated ignition model "
        "for its corresponding lcm message.\n";
    failure = true;
  }

  // Gets the pose and compares the values.
  const ignition::msgs::Pose pose = link.pose();

  const double kPositionTolerance(0.0);
  const double kOrientationTolerance(0.0);

  ::testing::AssertionResult position_check_result(
      CheckPositionTranslation(pose, lcm_msg, i, kPositionTolerance));

  ::testing::AssertionResult orientation_check_result(
      CheckOrientationTranslation(pose, lcm_msg, i, kOrientationTolerance));

  // If any of the two checks above failed, set failure flag.
  failure |= (!position_check_result || !orientation_check_result);

  if (failure) {
    // Appends the error messages from both, the position check and
    // the orientation check into the AssertionResult.
    error_msg += position_check_result.message();
    error_msg += orientation_check_result.message();
    return ::testing::AssertionFailure() << error_msg;
  }
  return ::testing::AssertionSuccess();
}
}  // namespace

::testing::AssertionResult CheckMsgTranslation(
    const drake::lcmt_viewer_draw& lcm_msg,
    const ignition::msgs::Model_V& ign_models) {
  if (!AssertLinkNumberEquivalence(lcm_msg, ign_models)) {
    return ::testing::AssertionFailure()
           << "Non-matching number of links "
              "between the LCM and Model_V models.\n";
  }

  std::string error_msg;
  bool failure = false;
  for (int i = 0; i < lcm_msg.num_links; i++) {
    // Checks there is a corresponding ignition model for the LCM link.
    bool found = false;
    ignition::msgs::Model model;
    for (int j = 0; j < ign_models.models_size(); ++j) {
      if (ign_models.models(j).id() == int64_t(lcm_msg.robot_num[i])) {
        model = ign_models.models(j);
        found = true;
        break;
      }
    }
    if (!found) {
      error_msg +=
          "Couldn't find a valid ignition model in the Model_V for the given "
          "LCM link\n";
      failure = true;
    }

    const ::testing::AssertionResult models_equivalence_check =
        AssertModelsEquivalence(model, lcm_msg, i);
    if (!models_equivalence_check) {
      error_msg += models_equivalence_check.message();
      error_msg += "Models are not equivalent.\n";
      failure = true;
    }
  }
  if (failure) {
    return ::testing::AssertionFailure() << error_msg;
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CheckMsgTranslation(
    const drake::lcmt_viewer_draw& lcm_msg,
    const ignition::msgs::Scene& scene) {
  bool failure = false;
  std::string error_msg;
  for (int i = 0; i < lcm_msg.num_links; i++) {
    // Checks there is a corresponding ignition model for the LCM link.
    bool found(false);
    ignition::msgs::Model model;
    for (int j = 0; j < scene.model_size(); ++j) {
      if (scene.model(j).id() == int64_t(lcm_msg.robot_num[i])) {
        model = scene.model(j);
        found = true;
        break;
      }
    }
    if (!found) {
      error_msg +=
          "Couldn't find a valid ignition model in the Scene for the given LCM "
          "link\n";
      failure = true;
    }
    const ::testing::AssertionResult models_equivalence_check =
        AssertModelsEquivalence(model, lcm_msg, i);
    if (!models_equivalence_check) {
      error_msg += models_equivalence_check.message();
      error_msg += "Models are not equivalent.\n";
      failure = true;
    }
  }
  if (failure) {
    return ::testing::AssertionFailure() << error_msg;
  }
  return ::testing::AssertionSuccess();
}

}  // namespace test
}  // namespace delphyne
