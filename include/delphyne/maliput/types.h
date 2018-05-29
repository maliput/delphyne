/**
 * @file include/delphyne/maliput/types.hpp
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
****************************************************************************/

#pragma once

#include <memory>

#include <drake/automotive/maliput/api/lane.h>
#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/automotive/maliput/dragway/road_geometry.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {
namespace maliput {

/*****************************************************************************
** Interfaces
*****************************************************************************/

typedef drake::maliput::dragway::RoadGeometry Dragway;
typedef drake::maliput::api::Lane Lane;
typedef drake::maliput::api::LaneId LaneId;
typedef drake::maliput::api::RoadGeometry RoadGeometry;

typedef std::unique_ptr<Dragway> DragwayPtr;
typedef std::unique_ptr<RoadGeometry> RoadGeometryPtr;

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace maliput
}  // namespace delphyne
