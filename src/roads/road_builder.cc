/**
 * @file src/roads/road_builder.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "delphyne/roads/road_builder.h"

#include <map>

#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>
#include <maliput/common/assertion_error.h>
#include <maliput/math/vector.h>
#include <maliput/plugin/maliput_plugin_manager.h>
#include <maliput/plugin/road_network_loader.h>

#include "delphyne/macros.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {
namespace roads {

namespace api = maliput::api;

/*****************************************************************************
** Implementation
*****************************************************************************/

std::unique_ptr<maliput::api::RoadNetwork> CreateRoadNetwork(
    const std::string& road_network_plugin_name, const std::map<std::string, std::string>& loader_parameters) {
  // The MaliputPluginManager instance must be alive until the program is terminated, otherwise
  // the RoadNetwork instance could lead to undesired behavior.
  static maliput::plugin::MaliputPluginManager manager{};
  const maliput::plugin::MaliputPlugin* plugin =
      manager.GetPlugin(maliput::plugin::MaliputPlugin::Id{road_network_plugin_name});
  DELPHYNE_VALIDATE(plugin != nullptr, maliput::common::assertion_error,
                    road_network_plugin_name + " plugin can't be obtained.");

  maliput::plugin::RoadNetworkLoaderPtr rn_loader_ptr =
      plugin->ExecuteSymbol<maliput::plugin::RoadNetworkLoaderPtr>(maliput::plugin::RoadNetworkLoader::GetEntryPoint());
  // Use smart pointers to gracefully manage heap allocation.
  std::unique_ptr<maliput::plugin::RoadNetworkLoader> road_network_loader{
      reinterpret_cast<maliput::plugin::RoadNetworkLoader*>(rn_loader_ptr)};
  // Generates the maliput::api::RoadNetwork.
  return (*road_network_loader)(loader_parameters);
}

std::unique_ptr<maliput::api::RoadNetwork> CreateDragway(const std::string& name, int num_lanes, double length,
                                                         double lane_width, double shoulder_width,
                                                         double maximum_height, double linear_tolerance,
                                                         double angular_tolerance) {
  const std::map<std::string, std::string> parameters{
      {"road_geometry_id", name},
      {"linear_tolerance", std::to_string(linear_tolerance)},
      {"angular_tolerance", std::to_string(angular_tolerance)},
      {"num_lanes", std::to_string(num_lanes)},
      {"length", std::to_string(length)},
      {"lane_width", std::to_string(lane_width)},
      {"shoulder_width", std::to_string(shoulder_width)},
      {"maximum_height", std::to_string(maximum_height)},
      {"inertial_to_backend_frame_translation", "{0., 0., 0.}"},
  };
  return CreateRoadNetwork("maliput_dragway", parameters);
}

std::unique_ptr<maliput::api::RoadNetwork> CreateMultilaneFromFile(const std::string& file_path) {
  const std::map<std::string, std::string> parameters{
      {"road_network_source", "yaml"},
      {"yaml_file", file_path},
  };
  return CreateRoadNetwork("maliput_multilane", parameters);
}

std::unique_ptr<maliput::api::RoadNetwork> CreateMultilaneFromDescription(const std::string& yaml_description) {
  const std::map<std::string, std::string> parameters{
      {"road_network_source", "yaml"},
      {"yaml_description", yaml_description},
  };
  return CreateRoadNetwork("maliput_multilane", parameters);
}

std::unique_ptr<maliput::api::RoadNetwork> CreateOnRamp() {
  const std::map<std::string, std::string> parameters{
      {"road_network_source", "on_ramp_merge"},
  };
  return CreateRoadNetwork("maliput_multilane", parameters);
}

std::unique_ptr<maliput::api::RoadNetwork> CreateMalidriveFromXodr(const std::string& name,
                                                                   const std::string& file_path,
                                                                   double linear_tolerance, double angular_tolerance) {
  return CreateMalidriveRoadNetworkFromXodr(name, file_path, {/*road_rulebook_file_path*/},
                                            {/*traffic_light_book_path*/}, {/*phase_ring_path*/}, linear_tolerance,
                                            angular_tolerance);
}

std::unique_ptr<maliput::api::RoadNetwork> CreateMalidriveRoadNetworkFromXodr(
    const std::string& name, const std::string& file_path, const std::string& road_rulebook_file_path,
    const std::string& traffic_light_book_path, const std::string& phase_ring_path, double linear_tolerance,
    double angular_tolerance) {
  static constexpr double kScaleLength{1.};
  std::map<std::string, std::string> road_network_configuration{
      {"road_geometry_id", name},
      {"opendrive_file", file_path},
      {"linear_tolerance", std::to_string(linear_tolerance)},
      {"angular_tolerance", std::to_string(angular_tolerance)},
      {"scale_length", std::to_string(kScaleLength)},
  };

  if (!road_rulebook_file_path.empty()) {
    road_network_configuration.emplace("road_rule_book", road_rulebook_file_path);
  }
  if (!traffic_light_book_path.empty()) {
    road_network_configuration.emplace("traffic_light_book", traffic_light_book_path);
  }
  if (!phase_ring_path.empty()) {
    road_network_configuration.emplace("phase_ring_book", phase_ring_path);
  }

  return CreateRoadNetwork("maliput_malidrive", road_network_configuration);
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace roads
}  // namespace delphyne
