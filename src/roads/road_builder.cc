/**
 * @file src/roads/road_builder.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "delphyne/roads/road_builder.h"

#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>
#include <maliput_dragway/road_geometry.h>
#include <maliput_multilane/loader.h>
#include <maliput_multilane/multilane_onramp_merge.h>

#include <malidrive/builder/road_geometry_configuration.h>
#include <malidrive/builder/road_network_configuration.h>
#include <malidrive/constants.h>
#include <malidrive/loader/loader.h>

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
                                                                maximum_height, linear_tolerance, angular_tolerance);
}

std::unique_ptr<const maliput::api::RoadGeometry> CreateMultilaneFromFile(const std::string& file_path) {
  return maliput::multilane::LoadFile(maliput::multilane::BuilderFactory(), file_path);
}

std::unique_ptr<const maliput::api::RoadGeometry> CreateOnRamp() {
  return maliput::multilane::MultilaneOnrampMerge().BuildOnramp();
}

std::unique_ptr<maliput::api::RoadNetwork> CreateMalidriveFromFile(const std::string& name,
                                                                   const std::string& file_path,
                                                                   const std::string& road_rulebook_file_path,
                                                                   const std::string& traffic_light_book_path,
                                                                   const std::string& phase_ring_path) {
  std::optional<std::string> road_rulebook =
      road_rulebook_file_path.empty() ? std::nullopt : std::optional<std::string>(road_rulebook_file_path);
  std::optional<std::string> traffic_light_book =
      traffic_light_book_path.empty() ? std::nullopt : std::optional<std::string>(traffic_light_book_path);
  std::optional<std::string> phase_ring =
      phase_ring_path.empty() ? std::nullopt : std::optional<std::string>(phase_ring_path);
  malidrive::builder::RoadNetworkConfiguration road_network_configuration{
      malidrive::builder::RoadGeometryConfiguration{
          maliput::api::RoadGeometryId(name), file_path, malidrive::constants::kLinearTolerance,
          malidrive::constants::kAngularTolerance, malidrive::constants::kScaleLength,
          malidrive::InertialToLaneMappingConfig(malidrive::constants::kExplorationRadius,
                                                 malidrive::constants::kNumIterations)},
      road_rulebook, traffic_light_book, phase_ring};
  return malidrive::loader::Load(road_network_configuration, malidrive::WorldToOpenDriveTransform::Identity());
}

std::unique_ptr<maliput::api::RoadNetwork> CreateMalidriveFromXodr(const std::string& name,
                                                                   const std::string& file_path) {
  const double kLinearTolerance{1e-3};
  const double kAngularTolerance{1e-3};
  malidrive::builder::RoadNetworkConfiguration road_network_configuration{malidrive::builder::RoadGeometryConfiguration{
      maliput::api::RoadGeometryId(name), file_path, kLinearTolerance, kAngularTolerance,
      malidrive::constants::kScaleLength,
      malidrive::InertialToLaneMappingConfig(malidrive::constants::kExplorationRadius,
                                             malidrive::constants::kNumIterations)}};
  return malidrive::loader::MalidriveLoad(road_network_configuration, malidrive::WorldToOpenDriveTransform::Identity());
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace roads
}  // namespace delphyne
