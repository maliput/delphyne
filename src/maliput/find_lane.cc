/**
 * @file src/maliput/find_lane.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/segment.h>

#include "delphyne/maliput/find_lane.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {
namespace roads {

/*****************************************************************************
** Implementation
*****************************************************************************/

// TODO(daniel.stonier) cache a map of lane id's -> lane pointers?
// TODO(daniel.stonier) return optional once we have c++17
const maliput::api::Lane* FindLane(const maliput::api::LaneId& lane_id,
                          const maliput::api::RoadGeometry& road_geometry) {
  for (int i = 0; i < road_geometry.num_junctions(); ++i) {
    const maliput::api::Junction* junction = road_geometry.junction(i);
    for (int j = 0; j < junction->num_segments(); ++j) {
      const maliput::api::Segment* segment = junction->segment(j);
      for (int k = 0; k < segment->num_lanes(); ++k) {
        const maliput::api::Lane* lane = segment->lane(k);
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

}  // namespace roads
}  // namespace delphyne
