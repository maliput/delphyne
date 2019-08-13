/**
 * @file include/delphyne/roads/find_lane.h
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
****************************************************************************/

#pragma once

#include <maliput/api/lane.h>
#include <maliput/api/road_geometry.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {
namespace roads {

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
const maliput::api::Lane* FindLane(const maliput::api::LaneId& lane_id,
                                   const maliput::api::RoadGeometry& road_geometry);

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace roads
}  // namespace delphyne
