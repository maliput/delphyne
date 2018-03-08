// Copyright 2017 Toyota Research Institute

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
