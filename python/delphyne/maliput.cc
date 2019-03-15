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
#include <drake/automotive/maliput/utility/generate_obj.h>

// public headers
#include "delphyne/maliput/find_lane.h"
#include "delphyne/maliput/road_builder.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace py = pybind11;

using drake::maliput::utility::ObjFeatures;

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

  m.def("create_on_ramp", &delphyne::maliput::CreateOnRamp,
        "Create the exemplar highway on-ramp");

  m.def("create_malidrive_from_file",
        &delphyne::maliput::CreateMalidriveFromFile,
        "Load an OpenDrive road geometry from file (xodr)",
        py::arg("name"), py::arg("file_path"));

  py::class_<ObjFeatures>(m, "ObjFeatures")
      .def(py::init<>())
      .def_readwrite("max_grid_unit", &ObjFeatures::max_grid_unit)
      .def_readwrite("min_grid_resolution", &ObjFeatures::min_grid_resolution)
      .def_readwrite("draw_elevation_bounds",
                     &ObjFeatures::draw_elevation_bounds)
      .def_readwrite("draw_stripes", &ObjFeatures::draw_stripes)
      .def_readwrite("draw_arrows", &ObjFeatures::draw_arrows)
      .def_readwrite("draw_lane_haze", &ObjFeatures::draw_lane_haze)
      .def_readwrite("draw_branch_points", &ObjFeatures::draw_branch_points)
      .def_readwrite("stripe_width", &ObjFeatures::stripe_width)
      .def_readwrite("stripe_elevation", &ObjFeatures::stripe_elevation)
      .def_readwrite("arrow_elevation", &ObjFeatures::arrow_elevation)
      .def_readwrite("lane_haze_elevation", &ObjFeatures::lane_haze_elevation)
      .def_readwrite("branch_point_elevation",
                     &ObjFeatures::branch_point_elevation)
      .def_readwrite("branch_point_height", &ObjFeatures::branch_point_height);
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace
