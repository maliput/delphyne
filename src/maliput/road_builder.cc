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
#include <drake/automotive/maliput/dragway/road_geometry.h>
#include <drake/automotive/maliput/monolane/loader.h>
#include <drake/automotive/maliput/multilane/loader.h>
#include <drake/automotive/monolane_onramp_merge.h>

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
  auto id = drake::maliput::api::RoadGeometryId(name);
  return std::make_unique<const drake::maliput::dragway::RoadGeometry>(
      id, num_lanes, length, lane_width, shoulder_width, maximum_height,
      linear_tolerance, angular_tolerance);
}

std::unique_ptr<const drake::maliput::api::RoadGeometry>
CreateMultilaneFromFile(const std::string& file_path) {
  return drake::maliput::multilane::LoadFile(
      drake::maliput::multilane::BuilderFactory(), file_path);
}

std::unique_ptr<const drake::maliput::api::RoadGeometry> CreateMonolaneFromFile(
    const std::string& file_path) {
  return drake::maliput::monolane::LoadFile(file_path);
}

std::unique_ptr<const drake::maliput::api::RoadGeometry> CreateOnRamp() {
  return drake::automotive::MonolaneOnrampMerge().BuildOnramp();
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace maliput
}  // namespace delphyne
