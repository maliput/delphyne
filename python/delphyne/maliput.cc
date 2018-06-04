/**
 * @file python/delphyne/maliput.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <memory>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <drake/automotive/maliput/api/road_geometry.h>

// public headers
#include "delphyne/maliput/find_lane.h"
#include "delphyne/maliput/road_builder.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace py = pybind11;

namespace {

/*****************************************************************************
** Implementation
*****************************************************************************/

PYBIND11_MODULE(maliput, m) {
  py::module::import("pydrake.maliput.api");

  m.def("create_dragway", &delphyne::maliput::CreateDragway);

//          [](const std::string& name, int num_lanes, double length,
//             double lane_width, double shoulder_width, double maximum_height,
//             double linear_tolerance, double angular_tolerance) {
//            return make_unique<drake::maliput::api::RoadGeometry>(
//                road_id, num_lanes, length, lane_width, shoulder_width,
//                maximum_height, linear_tolerance, angular_tolerance);
//          }, py::arg("road_id"), py::arg("num_lanes"), py::arg("length"),
//          py::arg("lane_width"), py::arg("shoulder_width"),
//          py::arg("maximum_height"), py::arg("linear_tolerance"),
//          py::arg("angular_tolerance"));

}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace
