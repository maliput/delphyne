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
#include <maliput/math/vector.h>
#include <maliput_dragway/road_geometry.h>
#include <maliput_malidrive/builder/road_geometry_configuration.h>
#include <maliput_malidrive/builder/road_network_builder.h>
#include <maliput_malidrive/builder/road_network_configuration.h>
#include <maliput_malidrive/constants.h>
#include <maliput_malidrive/loader/loader.h>
#include <maliput_multilane/loader.h>
#include <maliput_multilane/multilane_onramp_merge.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {
namespace roads {

namespace api = maliput::api;

/*****************************************************************************
** Implementation
*****************************************************************************/

std::unique_ptr<const maliput::api::RoadGeometry> CreateDragway(const std::string& name, int num_lanes, double length,
                                                                double lane_width, double shoulder_width,
                                                                double maximum_height, double linear_tolerance,
                                                                double angular_tolerance) {
  maliput::api::RoadGeometryId id(name);
  return std::make_unique<const maliput::dragway::RoadGeometry>(id, num_lanes, length, lane_width, shoulder_width,
                                                                maximum_height, linear_tolerance, angular_tolerance,
                                                                maliput::math::Vector3{0., 0., 0.});
}

std::unique_ptr<const maliput::api::RoadGeometry> CreateMultilaneFromFile(const std::string& file_path) {
  return maliput::multilane::LoadFile(maliput::multilane::BuilderFactory(), file_path);
}

std::unique_ptr<const maliput::api::RoadGeometry> CreateOnRamp() {
  return maliput::multilane::MultilaneOnrampMerge().BuildOnramp();
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
  std::map<std::string, std::string> road_network_configuration{
      {"road_geometry_id", name},
      {"opendrive_file", file_path},
      {"linear_tolerance", std::to_string(linear_tolerance)},
      {"angular_tolerance", std::to_string(angular_tolerance)},
      {"scale_length", std::to_string(malidrive::constants::kScaleLength)},
  };

  if (!road_rulebook_file_path.empty()) {
    road_network_configuration.emplace("road_rule_book", road_rulebook_file_path);
  }
  if (!traffic_light_book_path.empty()) {
    road_network_configuration.emplace("traffic_light_book", traffic_light_book_path);
  }
  if (!phase_ring_path.empty()) {
    road_network_configuration.emplace("phase_ring_path", phase_ring_path);
  }

  return malidrive::loader::Load<malidrive::builder::RoadNetworkBuilder>(road_network_configuration);
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace roads
}  // namespace delphyne
