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

/// @brief Find the lane inside the specified road geometry by id.
///
/// @param lane_id: a maliput-style identifier based on
/// @ref drake::automotive::api::TypeSpecificIdentifier<class Lane>
/// "TypeSpecificIdentifier"
///
/// @param road_geometry[in] Search over this road geometry.
/// @return A pointer to the lane, null if not found.
const drake::maliput::api::Lane* FindLane(
    const drake::maliput::api::LaneId& lane_id,
    const drake::maliput::api::RoadGeometry& road_geometry);

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace maliput
}  // namespace delphyne
