/**
 * @file src/maliput/find_lane.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <drake/automotive/maliput/api/junction.h>
#include <drake/automotive/maliput/api/lane.h>
#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/automotive/maliput/api/segment.h>

#include "delphyne/maliput/find_lane.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {
namespace maliput {

namespace api = drake::maliput::api;

/*****************************************************************************
** Implementation
*****************************************************************************/

// TODO(daniel.stonier) if frequently used, cache a map of lane id's -> lane
// lane pointers.

const api::Lane* FindLane(const api::LaneId& lane_id,
                          const api::RoadGeometry& road_geometry) {
  for (int i = 0; i < road_geometry.num_junctions(); ++i) {
    const api::Junction* junction = road_geometry.junction(i);
    for (int j = 0; j < junction->num_segments(); ++j) {
      const api::Segment* segment = junction->segment(j);
      for (int k = 0; k < segment->num_lanes(); ++k) {
        const api::Lane* lane = segment->lane(k);
        if (lane->id() == lane_id) {
          return lane;
        }
      }
    }
  }
  return nullptr;
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace maliput
}  // namespace delphyne
