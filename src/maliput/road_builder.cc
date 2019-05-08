/**
 * @file src/maliput/road_builder.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "delphyne/maliput/road_builder.h"

#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/automotive/maliput/api/road_network.h>
#include <drake/automotive/maliput/dragway/road_geometry.h>
#include <drake/automotive/maliput/multilane/loader.h>
#include <drake/automotive/multilane_onramp_merge.h>

#include <malidrive/constants.h>
#include <malidrive/loader.h>
#include <malidrive/road_geometry_configuration.h>
#include <malidrive/road_network_configuration.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {
namespace maliput {

namespace api = drake::maliput::api;

/*****************************************************************************
** Implementation
*****************************************************************************/

std::unique_ptr<const drake::maliput::api::RoadGeometry> CreateDragway(
    const std::string& name, int num_lanes, double length, double lane_width,
    double shoulder_width, double maximum_height, double linear_tolerance,
    double angular_tolerance) {
  drake::maliput::api::RoadGeometryId id(name);
  return std::make_unique<const drake::maliput::dragway::RoadGeometry>(
      id, num_lanes, length, lane_width, shoulder_width, maximum_height,
      linear_tolerance, angular_tolerance);
}

std::unique_ptr<const drake::maliput::api::RoadGeometry>
CreateMultilaneFromFile(const std::string& file_path) {
  return drake::maliput::multilane::LoadFile(
      drake::maliput::multilane::BuilderFactory(), file_path);
}

std::unique_ptr<const drake::maliput::api::RoadGeometry> CreateOnRamp() {
  return drake::automotive::MultilaneOnrampMerge().BuildOnramp();
}

std::unique_ptr<const drake::maliput::api::RoadNetwork>
CreateMalidriveFromFile(const std::string& name, const std::string& file_path) {
  malidrive::RoadNetworkConfiguration road_network_configuration{
    malidrive::RoadGeometryConfiguration{
    drake::maliput::api::RoadGeometryId(name),
    file_path,
    malidrive::Constants::kLinearTolerance,
    malidrive::Constants::kAngularTolerance,
    malidrive::Constants::kScaleLength,
    malidrive::InertialToLaneMappingConfig(
        malidrive::Constants::kExplorationRadius,
        malidrive::Constants::kNumIterations)
    },
    drake::nullopt,
    drake::nullopt,
    drake::nullopt
  };
  return malidrive::Load(
      road_network_configuration,
      malidrive::WorldToOpenDriveTransform::Identity());
}


/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace maliput
}  // namespace delphyne
