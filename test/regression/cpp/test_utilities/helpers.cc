// Copyright 2017 Toyota Research Institute

#include "test_utilities/helpers.h"

#include <cstdlib>
#include <exception>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <drake/common/eigen_types.h>
#include <drake/lcmt_viewer_draw.hpp>
#include <drake/lcmt_viewer_geometry_data.hpp>
#include <drake/math/rigid_transform.h>
#include <drake/systems/rendering/pose_bundle.h>
#include <ignition/msgs.hh>

#include "delphyne/macros.h"

namespace delphyne {
namespace test {

// The preloaded messages have kPreloadedModels models, each with
// kPreloadedLinks links, each with kPreloadedGeometries geometries.
const int kPreloadedModels{3};
const int kPreloadedLinks{2};
const int kPreloadedGeometries{4};

drake::lcmt_viewer_draw BuildPreloadedDrawMsg() {
  drake::lcmt_viewer_draw lcm_msg;
  lcm_msg.timestamp = 123456;

  for (int i = 0; i < kPreloadedModels; ++i) {
    for (int j = 0; j < kPreloadedLinks; ++j) {
      lcm_msg.link_name.push_back(std::to_string(i) + std::to_string(j));
      lcm_msg.robot_num.push_back(i);

      const std::vector<float> position{static_cast<float>(i), static_cast<float>(j + 5.0),
                                        static_cast<float>(i + 10.0)};
      lcm_msg.position.push_back(position);

      const std::vector<float> quaternion{static_cast<float>(j), static_cast<float>(i + 5.0),
                                          static_cast<float>(j + 10.0), static_cast<float>(i + 15.0)};
      lcm_msg.quaternion.push_back(quaternion);
    }
  }

  lcm_msg.num_links = kPreloadedModels * kPreloadedLinks;

  return lcm_msg;
}

drake::lcmt_viewer_load_robot BuildPreloadedLoadRobotMsg() {
  drake::lcmt_viewer_load_robot lcm_msg;

  for (int i = 0; i < kPreloadedModels; ++i) {
    for (int j = 0; j < kPreloadedLinks; ++j) {
      drake::lcmt_viewer_link_data link;

      link.robot_num = i;
      link.name = std::to_string(i) + std::to_string(j);

      for (int k = 0; k < kPreloadedGeometries; ++k) {
        drake::lcmt_viewer_geometry_data geometry;

        // Pre-fill the float_data, only some of them will be used, depending
        // on the actual geometry.
        geometry.float_data = std::vector<float>{static_cast<float>(i), static_cast<float>(j), static_cast<float>(k)};

        // We currently support 4 types of geometries, so only those are
        // generated.
        switch ((i + j + k) % 4) {
          case 0:
            geometry.type = drake::lcmt_viewer_geometry_data::BOX;
            geometry.num_float_data = 3;
            break;

          case 1:
            geometry.type = drake::lcmt_viewer_geometry_data::SPHERE;
            geometry.num_float_data = 1;
            break;

          case 2:
            geometry.type = drake::lcmt_viewer_geometry_data::CYLINDER;
            geometry.num_float_data = 2;
            break;

          case 3:
            geometry.type = drake::lcmt_viewer_geometry_data::MESH;
            geometry.string_data = "mesh";
            geometry.num_float_data = 3;
            break;

          default:
            throw std::logic_error("Unhandled geometry type");
            break;
        }

        geometry.position[0] = i;
        geometry.position[1] = j + 5.0;
        geometry.position[2] = k + 10.0;

        geometry.quaternion[0] = j;
        geometry.quaternion[1] = i + 5.0;
        geometry.quaternion[2] = j + 10.0;
        geometry.quaternion[3] = k + 15.0;

        geometry.color[0] = j;
        geometry.color[1] = i + 5.0;
        geometry.color[2] = j + 10.0;
        geometry.color[3] = k + 15.0;

        link.geom.push_back(geometry);
      }

      link.num_geom = link.geom.size();
      lcm_msg.link.push_back(link);
    }
  }

  lcm_msg.num_links = lcm_msg.link.size();

  return lcm_msg;
}

ignition::msgs::Model_V BuildPreloadedModelVMsg() {
  ignition::msgs::Model_V robot_models;
  robot_models.mutable_header()->mutable_stamp()->set_sec(123);
  robot_models.mutable_header()->mutable_stamp()->set_nsec(456000000);
  size_t currentLinkId = kPreloadedModels;

  for (int i = 0; i < kPreloadedModels; ++i) {
    ::ignition::msgs::Model* model = robot_models.add_models();
    model->set_id(i);
    model->set_name("none");

    ::ignition::msgs::Pose* model_pose = model->mutable_pose();
    model_pose->set_id(model->id());

    ::ignition::msgs::Vector3d* model_position = model_pose->mutable_position();
    model_position->set_x(i);
    model_position->set_y(i + 5.0);
    model_position->set_z(i + 10.0);

    ::ignition::msgs::Quaternion* model_orientation = model_pose->mutable_orientation();
    model_orientation->set_w(i);
    model_orientation->set_x(i + 5.0);
    model_orientation->set_y(i + 10.0);
    model_orientation->set_z(i + 15.0);

    for (int j = 0; j < kPreloadedLinks; ++j) {
      ::ignition::msgs::Link* link = model->add_link();
      link->set_name(std::to_string(i) + std::to_string(j));
      link->set_id(currentLinkId++);

      ::ignition::msgs::Pose* pose = link->mutable_pose();
      pose->set_id(link->id());

      ignition::msgs::Vector3d* position = pose->mutable_position();
      position->set_x(i);
      position->set_y(j + 5.0);
      position->set_z(i + 10.0);

      ignition::msgs::Quaternion* orientation = pose->mutable_orientation();
      orientation->set_w(j);
      orientation->set_x(i + 5.0);
      orientation->set_y(j + 10.0);
      orientation->set_z(i + 15.0);
    }
  }

  return robot_models;
}

using drake::AngleAxis;
using drake::Isometry3;
using drake::Translation3;
using drake::Vector3;
using drake::math::RigidTransform;
using drake::systems::rendering::PoseBundle;

PoseBundle<double> BuildPreloadedPoseBundle() {
  PoseBundle<double> model_states(kPreloadedModels);
  for (int i = 0; i < kPreloadedModels; ++i) {
    model_states.set_model_instance_id(i, i);
    model_states.set_name(i, std::string("model") + std::to_string(i));
    Isometry3<double> model_pose = AngleAxis<double>(i, Vector3<double>::UnitZ()) * Translation3<double>(i, i, i);
    model_states.set_transform(i, RigidTransform<double>(model_pose));
  }
  return model_states;
}

::testing::AssertionResult CheckLcmArrayToVector3dEquivalence(const float lcm_position[],
                                                              const ignition::msgs::Vector3d& ign_position,
                                                              double tolerance) {
  DELPHYNE_DEMAND(lcm_position != nullptr);
  std::string error_message;
  bool fails(false);
  double delta = std::abs(ign_position.x() - lcm_position[0]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Values for Vector3d are different at x coordinate. " +
                    "lcm_position[0]: " + std::to_string(lcm_position[0]) +
                    " vs. ign_position.x(): " + std::to_string(ign_position.x()) + ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_position.y() - lcm_position[1]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Values for Vector3d are different at y coordinate. " +
                    "lcm_position[1]: " + std::to_string(lcm_position[1]) +
                    " vs. ign_position.y(): " + std::to_string(ign_position.y()) + ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_position.z() - lcm_position[2]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Values for Vector3d are different at z coordinate. " +
                    "lcm_position[2]: " + std::to_string(lcm_position[2]) +
                    " vs. ign_position.z(): " + std::to_string(ign_position.z()) + ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CheckLcmArrayToQuaternionEquivalence(const float lcm_orientation[],
                                                                const ignition::msgs::Quaternion& ign_orientation,
                                                                double tolerance) {
  DELPHYNE_DEMAND(lcm_orientation != nullptr);
  std::string error_message;
  bool fails(false);
  double delta = std::abs(ign_orientation.w() - lcm_orientation[0]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Values for Quaternion are different at w coordinate. " +
                    "lcm_orientation[0]: " + std::to_string(lcm_orientation[0]) +
                    " vs. ign_orientation.w(): " + std::to_string(ign_orientation.w()) +
                    ", diff = " + std::to_string(delta) + ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_orientation.x() - lcm_orientation[1]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Values for Quaternion are different at x coordinate. " +
                    "lcm_orientation[1]: " + std::to_string(lcm_orientation[1]) +
                    " vs. ign_orientation.x(): " + std::to_string(ign_orientation.x()) +
                    ", diff = " + std::to_string(delta) + ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_orientation.y() - lcm_orientation[2]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Values for Quaternion are different at y coordinate. " +
                    "lcm_orientation[2]: " + std::to_string(lcm_orientation[2]) +
                    " vs. ign_orientation.y(): " + std::to_string(ign_orientation.y()) +
                    ", diff = " + std::to_string(delta) + ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_orientation.z() - lcm_orientation[3]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Values for Quaternion are different at z coordinate. " +
                    "lcm_orientation[3]: " + std::to_string(lcm_orientation[3]) +
                    " vs. ign_orientation.z(): " + std::to_string(ign_orientation.z()) +
                    ", diff = " + std::to_string(delta) + ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CheckLcmArrayToColorEquivalence(const float lcm_color[],
                                                           const ignition::msgs::Color& ign_color, double tolerance) {
  DELPHYNE_DEMAND(lcm_color != nullptr);
  std::string error_message;
  bool fails(false);
  double delta = std::abs(ign_color.r() - lcm_color[0]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Values for Color are different at r coordinate. " +
                    "lcm_color[0]: " + std::to_string(lcm_color[0]) +
                    " vs. ign_color.r(): " + std::to_string(ign_color.r()) + ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_color.g() - lcm_color[1]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Values for Color are different at g coordinate. " +
                    "lcm_color[1]: " + std::to_string(lcm_color[1]) +
                    " vs. ign_color.g(): " + std::to_string(ign_color.g()) + ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_color.b() - lcm_color[2]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Values for Color are different at b coordinate. " +
                    "lcm_color[2]: " + std::to_string(lcm_color[2]) +
                    " vs. ign_color.b(): " + std::to_string(ign_color.b()) + ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_color.a() - lcm_color[3]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Values for Color are different at a coordinate. " +
                    "lcm_color[3]: " + std::to_string(lcm_color[3]) +
                    " vs. ign_color.a(): " + std::to_string(ign_color.a()) + ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CheckGeometryTypeEquivalence(int8_t lcm_geometry_type,
                                                        ignition::msgs::Geometry::Type ign_geometry_type) {
  // Map between ignition Geometry and drake lcmt_viewer_geometry_data
  // types, since both are represented by different integer values.
  static const std::map<ignition::msgs::Geometry::Type, int8_t> ign_to_lcm_geometry_map = {
      {ignition::msgs::Geometry_Type_BOX, drake::lcmt_viewer_geometry_data::BOX},
      {ignition::msgs::Geometry_Type_SPHERE, drake::lcmt_viewer_geometry_data::SPHERE},
      {ignition::msgs::Geometry_Type_CYLINDER, drake::lcmt_viewer_geometry_data::CYLINDER},
      {ignition::msgs::Geometry_Type_MESH, drake::lcmt_viewer_geometry_data::MESH}};
  // If cannot find matching geometry in the map, fails prematurelly.
  if (ign_to_lcm_geometry_map.find(ign_geometry_type) == ign_to_lcm_geometry_map.end()) {
    return ::testing::AssertionFailure();
  }
  // Otherwise, checks equivalence between geometries.
  if (ign_to_lcm_geometry_map.find(ign_geometry_type)->second == lcm_geometry_type) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure();
}

namespace {
// @brief Asserts that the number of links found in an lcmt_viewer_draw message
// is equal to the ones found in an ignition Model_V one.
//
// @param lcm_msg The lcmt_viewer_draw message.
// @param ign_models The Model_V message against which to compare.
// @return true if the number of links coincides and false otherwise.
bool AssertLinkNumberEquivalence(const drake::lcmt_viewer_draw& lcm_msg, const ignition::msgs::Model_V& ign_models) {
  int ign_links = 0;
  for (int i = 0; i < ign_models.models_size(); ++i) {
    ign_links += ign_models.models(i).link_size();
  }

  return lcm_msg.num_links == ign_links;
}

// @brief Asserts that each link has a unique Id that does not match a model Id.
//
// @param ign_models The Model_V message to check.
// @return true if each model and its links each have a unique Id.
bool AssertUniqueModelAndLinkIds(const ignition::msgs::Model_V& ign_models) {
  std::unordered_set<size_t> ids;
  for (int m = 0; m < ign_models.models_size(); ++m) {
    if (ids.find(m) == ids.end()) {
      ids.insert(m);
    } else {
      // model id is not unique
      return false;
    }

    const ignition::msgs::Model& model = ign_models.models(m);
    for (int l = 0; l < model.link_size(); ++l) {
      size_t linkId = model.link(l).id();
      if (ids.find(linkId) == ids.end()) {
        ids.insert(linkId);
      } else {
        // link id is not unique
        return false;
      }
    }
  }

  return true;
}

// @brief Asserts that each link has a unique Id that does not match a model Id.
//
// @param ign_scene The Scene message to check.
// @return true if each model and its links each have a unique Id.
bool AssertUniqueModelAndLinkIds(const ignition::msgs::Scene& ign_scene) {
  std::unordered_set<size_t> ids;
  for (int m = 0; m < ign_scene.model_size(); ++m) {
    if (ids.find(m) == ids.end()) {
      ids.insert(m);
    } else {
      // model id is not unique
      return false;
    }

    const ignition::msgs::Model& model = ign_scene.model(m);
    for (int l = 0; l < model.link_size(); ++l) {
      size_t linkId = model.link(l).id();
      if (ids.find(linkId) == ids.end()) {
        ids.insert(linkId);
      } else {
        // link id is not unique
        return false;
      }
    }
  }

  return true;
}

// @brief Asserts that each pose has a unique Id.
//
// @param ign_poses The Pose_V message to check.
// @return success if each pose has a unique Id.
::testing::AssertionResult AssertUniquePoseIds(const ignition::msgs::Pose_V& ign_poses) {
  std::unordered_set<size_t> ids;
  for (int p = 0; p < ign_poses.pose_size(); ++p) {
    const ignition::msgs::Pose pose = ign_poses.pose(p);
    if (ids.find(pose.id()) == ids.end()) {
      ids.insert(pose.id());
    } else {
      // pose id is not unique
      return ::testing::AssertionFailure() << "pose id [" + std::to_string(pose.id()) + "] is not unique\n";
    }
  }

  return ::testing::AssertionSuccess();
}

// @brief Asserts that an ignition Model message is equivalent to an
// lcmt_viewer_draw one.
//
// @param model The ignition Model message.
// @param lcm_msg The lcmt_viewer_draw message against which to compare.
// @param i Index to a particular element of the position and quaternion arrays
// in the lcmt_viewer_draw message.
// @return AssertionSuccess if equivalence found, AssertionFailure otherwise.
::testing::AssertionResult AssertModelsEquivalence(const ignition::msgs::Model& model,
                                                   const drake::lcmt_viewer_draw& lcm_msg, int i) {
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

  const ::testing::AssertionResult position_check_result(
      CheckLcmArrayToVector3dEquivalence(lcm_msg.position[i].data(), pose.position(), kPositionTolerance));

  const ::testing::AssertionResult orientation_check_result(
      CheckLcmArrayToQuaternionEquivalence(lcm_msg.quaternion[i].data(), pose.orientation(), kOrientationTolerance));

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

::testing::AssertionResult CheckMsgTranslation(const drake::lcmt_viewer_draw& lcm_msg,
                                               const ignition::msgs::Model_V& ign_models) {
  if (!AssertLinkNumberEquivalence(lcm_msg, ign_models)) {
    return ::testing::AssertionFailure() << "Non-matching number of links "
                                            "between the LCM and Model_V models.\n";
  }
  if (!AssertUniqueModelAndLinkIds(ign_models)) {
    return ::testing::AssertionFailure() << "Non-unique Id in link or model in Model_V.\n";
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

    const ::testing::AssertionResult models_equivalence_check = AssertModelsEquivalence(model, lcm_msg, i);
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

::testing::AssertionResult CheckMsgTranslation(const drake::lcmt_viewer_load_robot& lcm_msg,
                                               const ignition::msgs::Model_V& ign_models) {
  if (!AssertUniqueModelAndLinkIds(ign_models)) {
    return ::testing::AssertionFailure() << "Non-unique Id in link or model in Model_V.\n";
  }

  const double kTolerance(0.001);

  // To check the translation between models easier, a first pass
  // over the lcm vector is done, grouping the links link id and name.
  std::map<int32_t, std::map<std::string, const drake::lcmt_viewer_link_data*>> grouped_links_by_id;

  for (const drake::lcmt_viewer_link_data& link : lcm_msg.link) {
    const int32_t id = link.robot_num;
    const std::string name = link.name;
    grouped_links_by_id[id][name] = &link;
  }

  // For each of the models in the Model_V
  for (const auto& ign_model : ign_models.models()) {
    for (const auto& ign_link : ign_model.link()) {
      if (grouped_links_by_id.find(ign_model.id()) == grouped_links_by_id.end()) {
        // Fails if none of the map groups matches a given ignition model ID.
        return ::testing::AssertionFailure() << "No match found for an ignition model with id:" << ign_model.id()
                                             << "within the tested lcm_viewer_load_robot message.\n";
      }
      if (grouped_links_by_id[ign_model.id()].find(ign_link.name()) == grouped_links_by_id[ign_model.id()].end()) {
        // Fails if none of the groups of lcm links with same model id matches a
        // given ignition link name.
        return ::testing::AssertionFailure() << "No match found for an ignition link with name:" << ign_link.name()
                                             << "within the tested lcm_viewer_load_robot message.\n";
      }
      // Creates a reference to the lcm link matching id
      // and name for legibility purposes.
      auto& lcm_link = grouped_links_by_id[ign_model.id()][ign_link.name()];
      // For each Visual within an ignition Link, there must be a 1 to 1
      // corresponance with the lcm vector of links in the same order.
      for (int i = 0; i < ign_link.visual_size(); ++i) {
        auto& ign_visual = ign_link.visual(i);
        const ::testing::AssertionResult are_positions_equivalent =
            CheckLcmArrayToVector3dEquivalence(lcm_link->geom[i].position, ign_visual.pose().position(), kTolerance);
        if (!are_positions_equivalent) {
          return ::testing::AssertionFailure() << are_positions_equivalent.message() << "\n";
        }
        const ::testing::AssertionResult are_quaternions_equivalent = CheckLcmArrayToQuaternionEquivalence(
            lcm_link->geom[i].quaternion, ign_visual.pose().orientation(), kTolerance);
        if (!are_quaternions_equivalent) {
          return ::testing::AssertionFailure() << are_quaternions_equivalent.message() << "\n";
        }
        // colors are not propagated for meshes
        if (lcm_link->geom[i].type != drake::lcmt_viewer_geometry_data::MESH) {
          const ::testing::AssertionResult are_colors_equivalent =
              CheckLcmArrayToColorEquivalence(lcm_link->geom[i].color, ign_visual.material().diffuse(), kTolerance);
          if (!are_colors_equivalent) {
            return ::testing::AssertionFailure() << are_colors_equivalent.message() << "\n";
          }
        }
        const ::testing::AssertionResult resultGeometryType =
            CheckGeometryTypeEquivalence(lcm_link->geom[i].type, ign_visual.geometry().type());
        if (!resultGeometryType) {
          return ::testing::AssertionFailure() << resultGeometryType.message() << "\n";
        }
      }
    }
  }
  // A fully matching translation is assumed.
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CheckMsgTranslation(const drake::lcmt_viewer_draw& lcm_msg,
                                               const ignition::msgs::Scene& scene) {
  if (!AssertUniqueModelAndLinkIds(scene)) {
    return ::testing::AssertionFailure() << "Non-unique Id in link or model in Model_V.\n";
  }

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
    const ::testing::AssertionResult models_equivalence_check = AssertModelsEquivalence(model, lcm_msg, i);
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

::testing::AssertionResult CheckMsgTranslation(const ignition::msgs::Model_V& ign_models,
                                               const ignition::msgs::Pose_V& ign_poses) {
  if (!AssertUniqueModelAndLinkIds(ign_models)) {
    return ::testing::AssertionFailure() << "Non-unique Id in link or model in Model_V.\n";
  }
  if (!AssertUniquePoseIds(ign_poses)) {
    return ::testing::AssertionFailure() << "Non-unique Id in Pose_V.\n";
  }

  // map of object Id to pose
  std::unordered_map<size_t, ignition::math::Pose3d> idToPose;

  // populate the map from the Model_V message
  for (int m = 0; m < ign_models.models_size(); ++m) {
    const ignition::msgs::Model& model = ign_models.models(m);
    idToPose[model.id()] = ignition::msgs::Convert(model.pose());

    for (int l = 0; l < model.link_size(); ++l) {
      const ignition::msgs::Link& link = model.link(l);
      idToPose[link.id()] = ignition::msgs::Convert(link.pose());
    }
  }

  if (static_cast<int>(idToPose.size()) != ign_poses.pose_size()) {
    return ::testing::AssertionFailure() << "Different number of entities in Model_V and Pose_V\n";
  }

  std::string error_msg;
  bool failure = false;

  // check that the Values of the Pose_V message match the content in the map
  for (int p = 0; p < ign_poses.pose_size(); ++p) {
    const ignition::msgs::Pose pose = ign_poses.pose(p);
    if (idToPose.find(pose.id()) == idToPose.end()) {
      error_msg += "Pose_V has entry with Id [" + std::to_string(pose.id()) + "] that is not found in Model_V\n";
      failure = true;
    } else {
      if (ignition::msgs::Convert(pose) != idToPose[pose.id()]) {
        error_msg += "Pose with Id [" + std::to_string(pose.id()) + "] does not match value in Model_V\n";
        failure = true;
      }
    }
  }

  if (failure) {
    return ::testing::AssertionFailure() << error_msg;
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CheckProtobufMsgEquality(const google::protobuf::MessageLite& lhs,
                                                    const google::protobuf::MessageLite& rhs) {
  return ((lhs.GetTypeName() == rhs.GetTypeName()) && (lhs.SerializeAsString() == rhs.SerializeAsString()))
             ? ::testing::AssertionSuccess()
             : ::testing::AssertionFailure();
}

std::string MakeTemporaryDirectory(const std::string& template_path) {
  char template_array[template_path.length() + 1];
  template_path.copy(template_array, template_path.length());
  template_array[template_path.length()] = '\0';
  return mkdtemp(template_array);
}

}  // namespace test
}  // namespace delphyne
