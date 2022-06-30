// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2017-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*****************************************************************************
** Includes
*****************************************************************************/

#include <limits>

#include <maliput/api/lane.h>
#include <maliput/api/road_geometry.h>
#include <maliput/utility/generate_obj.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// public headers
#include "delphyne/roads/find_lane.h"
#include "delphyne/roads/road_builder.h"
#include "delphyne/roads/road_network_wrapper.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace py = pybind11;

using maliput::utility::ObjFeatures;

namespace {

/*****************************************************************************
** Implementation
*****************************************************************************/

PYBIND11_MODULE(roads, m) {
  py::module::import("maliput.api");

  // Most of the required maliput types (e.g. Lane, LaneId, RoadGeometry)
  // already have bindings in maliput_py. Take advantage of these or help
  // contribute to them so others can take advantage of them as well.

  m.def("find_lane", &delphyne::roads::FindLane, "Find the lane inside the specified road geometry by id",
        py::arg("lane_id"), py::arg("road_geometry"));

  m.def("create_dragway", &delphyne::roads::CreateDragway, "Create a simple multi-lane dragway strip", py::arg("name"),
        py::arg("num_lanes"), py::arg("length"), py::arg("lane_width"), py::arg("shoulder_width"),
        py::arg("maximum_height"), py::arg("linear_tolerance") = std::numeric_limits<double>::epsilon(),
        py::arg("angular_tolerance") = std::numeric_limits<double>::epsilon());

  m.def("create_multilane_from_file", &delphyne::roads::CreateMultilaneFromFile,
        "Load a multilane road geometry from file (yaml)", py::arg("file_path"));

  m.def("create_on_ramp", &delphyne::roads::CreateOnRamp, "Create the exemplar highway on-ramp");

  m.def("create_malidrive_from_xodr", &delphyne::roads::CreateMalidriveFromXodr,
        "Load an OpenDrive road geometry from file (xodr).", py::arg("name"), py::arg("file_path"),
        py::arg("linear_tolerance") = 1e-3, py::arg("angular_tolerance") = 1e-3);

  m.def("create_malidrive_road_network_from_xodr", &delphyne::roads::CreateMalidriveRoadNetworkFromXodr,
        "Load an full RoadNetwork based on malidrive backend", py::arg("name"), py::arg("file_path"),
        py::arg("rule_registry_file_path"), py::arg("road_rulebook_file_path"), py::arg("traffic_light_book_path"),
        py::arg("phase_ring_path"), py::arg("intersection_book_path"), py::arg("linear_tolerance") = 1e-3,
        py::arg("angular_tolerance") = 1e-3);

  py::class_<ObjFeatures>(m, "ObjFeatures")
      .def(py::init<>())
      .def_readwrite("max_grid_unit", &ObjFeatures::max_grid_unit)
      .def_readwrite("min_grid_resolution", &ObjFeatures::min_grid_resolution)
      .def_readwrite("draw_elevation_bounds", &ObjFeatures::draw_elevation_bounds)
      .def_readwrite("draw_stripes", &ObjFeatures::draw_stripes)
      .def_readwrite("draw_arrows", &ObjFeatures::draw_arrows)
      .def_readwrite("draw_lane_haze", &ObjFeatures::draw_lane_haze)
      .def_readwrite("draw_branch_points", &ObjFeatures::draw_branch_points)
      .def_readwrite("stripe_width", &ObjFeatures::stripe_width)
      .def_readwrite("stripe_elevation", &ObjFeatures::stripe_elevation)
      .def_readwrite("arrow_elevation", &ObjFeatures::arrow_elevation)
      .def_readwrite("lane_haze_elevation", &ObjFeatures::lane_haze_elevation)
      .def_readwrite("branch_point_elevation", &ObjFeatures::branch_point_elevation)
      .def_readwrite("branch_point_height", &ObjFeatures::branch_point_height);

  py::class_<delphyne::roads::RoadNetworkWrapper>(m, "RoadNetworkWrapper")
      .def("get", [](const delphyne::roads::RoadNetworkWrapper* self) { return self->operator->(); });
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace
