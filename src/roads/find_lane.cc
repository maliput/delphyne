/**
 * @file src/roads/find_lane.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "delphyne/roads/find_lane.h"

#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/segment.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {
namespace roads {

/*****************************************************************************
** Implementation
*****************************************************************************/

// TODO(daniel.stonier) return optional once we have c++17
const maliput::api::Lane* FindLane(const maliput::api::LaneId& lane_id,
                                   const maliput::api::RoadGeometry& road_geometry) {
  return road_geometry.ById().GetLane(lane_id);
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace roads
}  // namespace delphyne
