/**
 * @file include/delphyne/maliput/find_lane.h
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
****************************************************************************/

#pragma once

#include <drake/automotive/maliput/api/lane.h>
#include <drake/automotive/maliput/api/road_geometry.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {
namespace maliput {



/*****************************************************************************
** Methods
*****************************************************************************/

/**
 * @brief Find the lane corresponding to the unique name identifier
 *
 * @param lane_id: unique lane identifier (based on a string)
 * @param road_geometry: road geometry to search in
 * @return a pointer to the lane, null if not found
 *
 * TODO(daniel.stonier): use optional once we have c++17
 */
const drake::maliput::api::Lane* FindLane(
    const drake::maliput::api::LaneId& lane_id,
    const drake::maliput::api::RoadGeometry& road_geometry);

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace maliput
} // namespace delphyne
