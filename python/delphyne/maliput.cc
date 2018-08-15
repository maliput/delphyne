/**
 * @file python/delphyne/maliput.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <limits>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <drake/automotive/maliput/api/lane.h>
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

  // Most of the required maliput types (e.g. Lane, LaneId, RoadGeometry)
  // already have bindings in pydrake. Take advantage of these or help
  // contribute to them so others can take advantage of them as well.

  m.def("find_lane", &delphyne::maliput::FindLane,
        "Find the lane inside the specified road geometry by id",
        py::arg("lane_id"), py::arg("road_geometry"));

  m.def("create_dragway", &delphyne::maliput::CreateDragway,
        "Create a simple multi-lane dragway strip", py::arg("name"),
        py::arg("num_lanes"), py::arg("length"), py::arg("lane_width"),
        py::arg("shoulder_width"), py::arg("maximum_height"),
        py::arg("linear_tolerance") = std::numeric_limits<double>::epsilon(),
        py::arg("angular_tolerance") = std::numeric_limits<double>::epsilon());

  m.def(
      "create_multilane_from_file", &delphyne::maliput::CreateMultilaneFromFile,
      "Load a multilane road geometry from file (yaml)", py::arg("file_path"));

  m.def("create_monolane_from_file", &delphyne::maliput::CreateMonolaneFromFile,
        "Load a monolane road geometry from file (yaml)", py::arg("file_path"));

  m.def("create_on_ramp", &delphyne::maliput::CreateOnRamp,
        "Create the exemplar highway on-ramp");
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace
