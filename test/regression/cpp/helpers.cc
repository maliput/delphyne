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

::testing::AssertionResult CheckLcmArrayToVector3DEquivalence(
    const float lcm_position[], ignition::msgs::Vector3d ign_position,
    double tolerance) {
  DELPHYNE_ASSERT(lcm_position != nullptr);
  std::string error_message;
  bool fails(false);
  double delta = std::abs(ign_position.x() - lcm_position[0]);
  if (delta > tolerance) {
    fails = true;
    error_message =
        error_message + "Values for Vector3D are different at x coordinate. " +
        "lcm_position[0]: " + std::to_string(lcm_position[0]) +
        " vs. ign_position.x(): " + std::to_string(ign_position.x()) +
        ", diff = " + std::to_string(delta) + ", tolerance = " +
        std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_position.y() - lcm_position[1]);
  if (delta > tolerance) {
    fails = true;
    error_message =
        error_message + "Values for Vector3D are different at y coordinate. " +
        "lcm_position[1]: " + std::to_string(lcm_position[1]) +
        " vs. ign_position.y(): " + std::to_string(ign_position.y()) +
        ", diff = " + std::to_string(delta) + ", tolerance = " +
        std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_position.z() - lcm_position[2]);
  if (delta > tolerance) {
    fails = true;
    error_message =
        error_message + "Values for Vector3D are different at z coordinate. " +
        "lcm_position[2]: " + std::to_string(lcm_position[2]) +
        " vs. ign_position.z(): " + std::to_string(ign_position.z()) +
        ", diff = " + std::to_string(delta) + ", tolerance = " +
        std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CheckLcmArrayToQuaternionEquivalence(
    const float lcm_orientation[], ignition::msgs::Quaternion ign_orientation,
    double tolerance) {
  std::string error_message;
  bool fails(false);
  double delta = std::abs(ign_orientation.w() - lcm_orientation[0]);
  if (delta > tolerance) {
    fails = true;
    error_message =
        error_message +
        "Values for Quaternion are different at w coordinate. " +
        "lcm_orientation[0]: " + std::to_string(lcm_orientation[0]) +
        " vs. ign_orientation.w(): " + std::to_string(ign_orientation.w()) +
        ", diff = " + std::to_string(delta) + ", tolerance = " +
        std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_orientation.x() - lcm_orientation[1]);
  if (delta > tolerance) {
    fails = true;
    error_message =
        error_message +
        "Values for Quaternion are different at x coordinate. " +
        "lcm_orientation[1]: " + std::to_string(lcm_orientation[1]) +
        " vs. ign_orientation.x(): " + std::to_string(ign_orientation.x()) +
        ", diff = " + std::to_string(delta) + ", tolerance = " +
        std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_orientation.y() - lcm_orientation[2]);
  if (delta > tolerance) {
    fails = true;
    error_message =
        error_message +
        "Values for Quaternion are different at y coordinate. " +
        "lcm_orientation[2]: " + std::to_string(lcm_orientation[2]) +
        " vs. ign_orientation.y(): " + std::to_string(ign_orientation.y()) +
        ", diff = " + std::to_string(delta) + ", tolerance = " +
        std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_orientation.z() - lcm_orientation[3]);
  if (delta > tolerance) {
    fails = true;
    error_message =
        error_message +
        "Values for Quaternion are different at z coordinate. " +
        "lcm_orientation[3]: " + std::to_string(lcm_orientation[3]) +
        " vs. ign_orientation.z(): " + std::to_string(ign_orientation.z()) +
        ", diff = " + std::to_string(delta) + ", tolerance = " +
        std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CheckLcmArrayToColorEquivalence(
    const float lcm_color[], ignition::msgs::Color ign_color,
    double tolerance) {
  std::string error_message;
  bool fails(false);
  double delta = std::abs(ign_color.r() - lcm_color[0]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message +
                    "Values for Color are different at r coordinate. " +
                    "lcm_color[0]: " + std::to_string(lcm_color[0]) +
                    " vs. ign_color.r(): " + std::to_string(ign_color.r()) +
                    ", diff = " + std::to_string(delta) + ", tolerance = " +
                    std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_color.g() - lcm_color[1]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message +
                    "Values for Color are different at g coordinate. " +
                    "lcm_color[1]: " + std::to_string(lcm_color[1]) +
                    " vs. ign_color.g(): " + std::to_string(ign_color.g()) +
                    ", diff = " + std::to_string(delta) + ", tolerance = " +
                    std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_color.b() - lcm_color[2]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message +
                    "Values for Color are different at b coordinate. " +
                    "lcm_color[2]: " + std::to_string(lcm_color[2]) +
                    " vs. ign_color.b(): " + std::to_string(ign_color.b()) +
                    ", diff = " + std::to_string(delta) + ", tolerance = " +
                    std::to_string(tolerance) + "\n";
  }
  delta = std::abs(ign_color.a() - lcm_color[3]);
  if (delta > tolerance) {
    fails = true;
    error_message = error_message +
                    "Values for Color are different at a coordinate. " +
                    "lcm_color[3]: " + std::to_string(lcm_color[3]) +
                    " vs. ign_color.a(): " + std::to_string(ign_color.a()) +
                    ", diff = " + std::to_string(delta) + ", tolerance = " +
                    std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult CheckGeometryTypeEquivalence(
    int8_t lcm_geometry_type,
    ignition::msgs::Geometry::Type ign_geometry_type) {
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
  if (ign_to_lcm_geometry_map[ign_geometry_type] == lcm_geometry_type) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure();
}

namespace {
bool AssertLinkNumberEquivalence(const drake::lcmt_viewer_draw& lcm_msg,
                                 const ignition::msgs::Model_V& ign_models) {
  int ign_links = 0;
  for (int i = 0; i < ign_models.models_size(); ++i) {
    const ::ignition::msgs::Model model = ign_models.models(i);
    ign_links += model.link_size();
  }

  return lcm_msg.num_links == ign_links;
}

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
      CheckLcmArrayToVector3DEquivalence(lcm_msg.position[i].data(),
                                         pose.position(), kPositionTolerance));

  ::testing::AssertionResult orientation_check_result(
      CheckLcmArrayToQuaternionEquivalence(lcm_msg.quaternion[i].data(),
                                           pose.orientation(),
                                           kOrientationTolerance));

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
  const double kTolerance(0.001);

  // To check the translation between models easier, a first pass
  // over the lcm vector is done, grouping the links link id.
  std::map<int32_t, std::vector<const drake::lcmt_viewer_link_data*>>
      grouped_links_by_id;

  for (const drake::lcmt_viewer_link_data& link : lcm_msg.link) {
    const int32_t id = link.robot_num;
    if (grouped_links_by_id.count(id) == 0) {
      grouped_links_by_id[id] = {};
    }
    grouped_links_by_id[id].push_back(&link);
  }

  // For each of the models in the Model_V
  for (const auto& ign_model : ign_models.models()) {
    // Looks for an ID match in the map of lcm links.
    if (grouped_links_by_id.find(ign_model.id()) == grouped_links_by_id.end()) {
      // Fails if none of the groups of lcm links match a given ignition model id.
      return ::testing::AssertionFailure();
    }
    // On each match found
    else {
      auto sameIdLcmLinks = grouped_links_by_id[ign_model.id()];
      // For each link within the group
      for (const auto& lcm_link : sameIdLcmLinks) {
        for (const auto& ign_link : ign_model.link()) {
          // Looks for a link with the same name.
          // If there is a match in name as well as in ID
          // then a valid candidate to compare values exists.
          if (lcm_link->name == ign_link.name()) {
            // There must be a correspondance of 1 to 1 between
            // the position of the elements in the vector of ignition
            // visuals and the vector of lcm geometries.
            int i = 0;
            for (const auto& ign_visual : ign_link.visual()) {
              // If no match is found, raises a failure.
              if (!CheckLcmArrayToVector3DEquivalence(
                      lcm_link->geom[i].position, ign_visual.pose().position(),
                      kTolerance)) {
                return ::testing::AssertionFailure();
              }
              if (!CheckLcmArrayToQuaternionEquivalence(
                      lcm_link->geom[i].quaternion,
                      ign_visual.pose().orientation(), kTolerance)) {
                return ::testing::AssertionFailure();
              }
              if (!CheckLcmArrayToColorEquivalence(
                      lcm_link->geom[i].color, ign_visual.material().diffuse(),
                      kTolerance)) {
                return ::testing::AssertionFailure();
              }
              if (!CheckGeometryTypeEquivalence(lcm_link->geom[i].type,
                                                ign_visual.geometry().type())) {
                return ::testing::AssertionFailure();
              }
              i++;
            }
          }
        }
      }
    }
  }
  // Otherwise, a fully matching translation is assumed.
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
