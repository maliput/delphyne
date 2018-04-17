// Copyright 2017 Toyota Research Institute

#include "test/regression/cpp/helpers.h"

#include <exception>
#include <string>
#include <vector>

#include "backend/system.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "helpers.h"
#include "ignition/msgs.hh"

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

      const std::vector<float> position{static_cast<float>(i),
                                        static_cast<float>(j + 5.0),
                                        static_cast<float>(i + 10.0)};
      lcm_msg.position.push_back(position);

      const std::vector<float> quaternion{
          static_cast<float>(j), static_cast<float>(i + 5.0),
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
        geometry.float_data =
            std::vector<float>{static_cast<float>(i), static_cast<float>(j),
                               static_cast<float>(k)};

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

  for (int i = 0; i < kPreloadedModels; ++i) {
    ::ignition::msgs::Model* model = robot_models.add_models();
    model->set_id(i);

    for (int j = 0; j < kPreloadedLinks; ++j) {
      ::ignition::msgs::Link* link = model->add_link();
      link->set_name(std::to_string(i) + std::to_string(j));

      ::ignition::msgs::Pose* pose = link->mutable_pose();

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
    const drake::lcmt_viewer_load_robot& lcm_msg,
    const ignition::msgs::Model_V& ign_models) {
  // Map between ignition Geometry and drake lcmt_viewer_geometry_data
  // types, since both are represented by different integer values.
  std::map<ignition::msgs::Geometry::Type, int8_t> ign_to_lcm_geometry_map = {
      {ignition::msgs::Geometry_Type_BOX,
       drake::lcmt_viewer_geometry_data::BOX},
      {ignition::msgs::Geometry_Type_SPHERE,
       drake::lcmt_viewer_geometry_data::SPHERE},
      {ignition::msgs::Geometry_Type_CYLINDER,
       drake::lcmt_viewer_geometry_data::CYLINDER},
      {ignition::msgs::Geometry_Type_MESH,
       drake::lcmt_viewer_geometry_data::MESH}};

  int32_t num_geometries_matched = 0;
  int32_t num_geometries_total = 0;

  // Sum up all the geometries existent in the message.
  for (int n = 0; n < lcm_msg.num_links; ++n) {
    num_geometries_total += lcm_msg.link[n].num_geom;
  }
  for (const auto& ign_model : ign_models.models()) {
    for (const auto& ign_link : ign_model.link()) {
      for (int n = 0; n < lcm_msg.num_links; ++n) {
        drake::lcmt_viewer_link_data lcm_link = lcm_msg.link[n];
        if (static_cast<uint32_t>(lcm_link.robot_num) == ign_model.id()) {
          if (lcm_link.name == ign_link.name()) {
            for (int i = 0; i < lcm_link.num_geom; ++i) {
              for (const auto& ign_visual : ign_link.visual()) {
                if ((lcm_link.geom[i].position[0] ==
                     ign_visual.pose().position().x()) &&
                    (lcm_link.geom[i].position[1] ==
                     ign_visual.pose().position().y()) &&
                    (lcm_link.geom[i].position[2] ==
                     ign_visual.pose().position().z()) &&
                    (lcm_link.geom[i].quaternion[0] ==
                     ign_visual.pose().orientation().w()) &&
                    (lcm_link.geom[i].quaternion[1] ==
                     ign_visual.pose().orientation().x()) &&
                    (lcm_link.geom[i].quaternion[2] ==
                     ign_visual.pose().orientation().y()) &&
                    (lcm_link.geom[i].quaternion[3] ==
                     ign_visual.pose().orientation().z()) &&
                    (lcm_link.geom[i].color[0] ==
                     ign_visual.material().diffuse().r()) &&
                    (lcm_link.geom[i].color[1] ==
                     ign_visual.material().diffuse().g()) &&
                    (lcm_link.geom[i].color[2] ==
                     ign_visual.material().diffuse().b()) &&
                    (lcm_link.geom[i].color[3] ==
                     ign_visual.material().diffuse().a()) &&
                    (lcm_link.geom[i].type ==
                     ign_to_lcm_geometry_map[ign_visual.geometry().type()])) {
                  ++num_geometries_matched;
                }
              }
            }
          }
        }
      }
    }
  }
  // If a fully matching ignition geometry is found for each lcm visual,
  // we assume the lcm message is contained within its ignition counterpart.
  if (num_geometries_matched == num_geometries_total) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure();
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

::testing::AssertionResult CheckProtobufMsgEquality(
    const google::protobuf::MessageLite& lhs,
    const google::protobuf::MessageLite& rhs) {
  return ((lhs.GetTypeName() == rhs.GetTypeName()) &&
          (lhs.SerializeAsString() == rhs.SerializeAsString()))
             ? ::testing::AssertionSuccess()
             : ::testing::AssertionFailure();
}

}  // namespace test
}  // namespace delphyne
