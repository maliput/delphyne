// Copyright 2019 Toyota Research Institute
#include "backend/geometry_utilities.h"

#include <algorithm>
#include <string>

#include <drake/geometry/scene_graph.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <maliput/math/roll_pitch_yaw.h>
#include <maliput/math/vector.h>
#include <maliput/utility/generate_urdf.h>

#include "delphyne/macros.h"

namespace delphyne {
namespace {

// Creates and registers traffic lights bulbs in a given @p plant.
// The bulb geometry is described as a rigid body in a predefined cylinder shape.
//
// @param unique_bulb_id A unique ID for the bulb to be used as name.
// @param position Position of the bulb.
// @param rpy Orientation of the bulg.
// @param bulb_color Color of the bulb.
// @param plant Is the plant where the bulb will be registered.
void CreateAndRegisterBulb(const std::string& unique_bulb_id, const maliput::math::Vector3& position,
                           const maliput::math::RollPitchYaw& rpy, const maliput::api::rules::BulbColor& bulb_color,
                           drake::multibody::MultibodyPlant<double>& plant) {
  static const Eigen::Vector4d kGreen(0.0, 1.0, 0.0, 1.0);
  static const Eigen::Vector4d kYellow(1.0, 1.0, 0.0, 1.0);
  static const Eigen::Vector4d kRed(1.0, 0.0, 0.0, 1.0);

  auto get_color = [](const maliput::api::rules::BulbColor& bulb_color) {
    switch (bulb_color) {
      case maliput::api::rules::BulbColor::kRed:
        return kRed;
      case maliput::api::rules::BulbColor::kYellow:
        return kYellow;
      case maliput::api::rules::BulbColor::kGreen:
        return kGreen;
      default:
        throw std::runtime_error("Unknown bulb color");
    }
  };

  auto& rigid_body = plant.AddRigidBody(unique_bulb_id, drake::multibody::SpatialInertia<double>());
  // Visual for the Cylinder.
  plant.RegisterVisualGeometry(
      rigid_body,
      // The cylinder should be rotated half pi radians as the default orientation is along the z-axis.
      drake::math::RigidTransformd(
          drake::math::RollPitchYaw{rpy.roll_angle(), rpy.pitch_angle() + M_PI_2, rpy.yaw_angle()},
          Eigen::Vector3d(position.x(), position.y(), position.z())),
      drake::geometry::Cylinder(0.18, 0.05), "visual", get_color(bulb_color));
}

}  // namespace

drake::lcmt_viewer_load_robot BuildLoadMessageForRoad(const maliput::api::RoadGeometry& road_geometry,
                                                      const maliput::utility::ObjFeatures& features) {
  drake::geometry::SceneGraph<double> scene_graph;
  drake::multibody::MultibodyPlant<double> plant(0.0);
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  std::string filename = road_geometry.id().string();
  std::transform(filename.begin(), filename.end(), filename.begin(), [](char ch) { return ch == ' ' ? '_' : ch; });
  maliput::utility::GenerateUrdfFile(&road_geometry, "/tmp", filename, features);
  drake::multibody::Parser parser(&plant);
  parser.AddModelFromFile("/tmp/" + filename + ".urdf");
  plant.Finalize();
  drake::lcmt_viewer_load_robot load_message = BuildLoadMessage(scene_graph);
  for (drake::lcmt_viewer_link_data& link : load_message.link) {
    DELPHYNE_VALIDATE(link.geom.size() == 1, std::runtime_error, "Expected one geometry in link");
    link.name = link.name.substr(link.name.rfind("::") + 2);
  }
  return load_message;
}

drake::lcmt_viewer_load_robot BuildLoadMessageForTrafficLights(
    const std::vector<const maliput::api::rules::TrafficLight*>& traffic_lights) {
  drake::geometry::SceneGraph<double> scene_graph;
  drake::multibody::MultibodyPlant<double> plant(0.0);
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  // Iterates over all traffic lights and creates bulbs for each one.
  // Traffic lights' pose are defined in the inertial frame while
  // both bulb groups and bulbs are relative to the traffic light frame.
  // So some transformations are needed to convert the bulb pose.
  for (const auto& traffic_light : traffic_lights) {
    const auto traffic_light_position = traffic_light->position_road_network();
    const auto traffic_light_orientation = traffic_light->orientation_road_network();
    for (const auto& bulb_group : traffic_light->bulb_groups()) {
      const auto bulb_group_position = bulb_group->position_traffic_light();
      const auto bulb_group_orientation = bulb_group->orientation_traffic_light();
      for (const auto& bulb : bulb_group->bulbs()) {
        const auto bulb_position = bulb->position_bulb_group();
        const auto bulb_orientation = bulb->orientation_bulb_group();
        const auto bulb_position_world_frame = traffic_light_position + bulb_group_position + bulb_position;
        const auto bulb_orientation_world_frame = maliput::api::Rotation::FromQuat(maliput::math::Quaternion{
            traffic_light_orientation.matrix() * bulb_group_orientation.matrix() * bulb_orientation.matrix()});
        CreateAndRegisterBulb(bulb->unique_id().string(), bulb_position_world_frame.xyz(),
                              bulb_orientation_world_frame.rpy(), bulb->color(), plant);
      }
    }
  }

  plant.Finalize();

  drake::lcmt_viewer_load_robot load_message = BuildLoadMessage(scene_graph);
  for (drake::lcmt_viewer_link_data& link : load_message.link) {
    DELPHYNE_VALIDATE(link.geom.size() == 1, std::runtime_error, "Expected one geometry in link");
    link.name = link.name.substr(link.name.rfind("::") + 2);
  }
  return load_message;
}

}  // namespace delphyne
