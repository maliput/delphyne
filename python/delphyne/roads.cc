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
#include "delphyne/roads/road_network.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace py = pybind11;

using maliput::utility::ObjFeatures;

namespace {

/*****************************************************************************
** Implementation
*****************************************************************************/

std::unique_ptr<delphyne::roads::RoadNetwork> create_malidrive_road_network_from_xodr(
    const std::string& name, const std::string& file_path, const std::string& rule_registry_file_path,
    const std::string& road_rulebook_file_path, const std::string& traffic_light_book_path,
    const std::string& phase_ring_path, const std::string& intersection_book_path, double linear_tolerance,
    double angular_tolerance) {
  return std::make_unique<delphyne::roads::RoadNetwork>(delphyne::roads::CreateMalidriveRoadNetworkFromXodr(
      name, file_path, rule_registry_file_path, road_rulebook_file_path, traffic_light_book_path, phase_ring_path,
      intersection_book_path, linear_tolerance, angular_tolerance));
}

std::unique_ptr<delphyne::roads::RoadNetwork> create_malidrive_from_xodr(const std::string& name,
                                                                         const std::string& file_path,
                                                                         double linear_tolerance,
                                                                         double angular_tolerance) {
  return std::make_unique<delphyne::roads::RoadNetwork>(
      delphyne::roads::CreateMalidriveFromXodr(name, file_path, linear_tolerance, angular_tolerance));
}

std::unique_ptr<delphyne::roads::RoadNetwork> create_dragway(const std::string& name, int num_lanes, double length,
                                                             double lane_width, double shoulder_width,
                                                             double maximum_height, double linear_tolerance,
                                                             double angular_tolerance) {
  return std::make_unique<delphyne::roads::RoadNetwork>(delphyne::roads::CreateDragway(
      name, num_lanes, length, lane_width, shoulder_width, maximum_height, linear_tolerance, angular_tolerance));
}

std::unique_ptr<delphyne::roads::RoadNetwork> create_multilane_from_file(const std::string& file_path) {
  return std::make_unique<delphyne::roads::RoadNetwork>(delphyne::roads::CreateMultilaneFromFile(file_path));
}

std::unique_ptr<delphyne::roads::RoadNetwork> create_on_ramp() {
  return std::make_unique<delphyne::roads::RoadNetwork>(delphyne::roads::CreateOnRamp());
}

PYBIND11_MODULE(roads, m) {
  py::module::import("maliput.api");

  // Most of the required maliput types (e.g. Lane, LaneId, RoadGeometry)
  // already have bindings in maliput_py. Take advantage of these or help
  // contribute to them so others can take advantage of them as well.

  m.def("find_lane", &delphyne::roads::FindLane, "Find the lane inside the specified road geometry by id",
        py::arg("lane_id"), py::arg("road_geometry"));

  m.def("create_dragway", &create_dragway, "Create a simple multi-lane dragway strip", py::arg("name"),
        py::arg("num_lanes"), py::arg("length"), py::arg("lane_width"), py::arg("shoulder_width"),
        py::arg("maximum_height"), py::arg("linear_tolerance") = std::numeric_limits<double>::epsilon(),
        py::arg("angular_tolerance") = std::numeric_limits<double>::epsilon());

  m.def("create_multilane_from_file", &create_multilane_from_file, "Load a multilane road geometry from file (yaml)",
        py::arg("file_path"));

  m.def("create_on_ramp", &create_on_ramp, "Create the exemplar highway on-ramp");

  m.def("create_malidrive_from_xodr", &create_malidrive_from_xodr, "Load an OpenDrive road geometry from file (xodr).",
        py::arg("name"), py::arg("file_path"), py::arg("linear_tolerance") = 1e-3, py::arg("angular_tolerance") = 1e-3);

  m.def("create_malidrive_road_network_from_xodr", &create_malidrive_road_network_from_xodr,
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

  py::class_<delphyne::roads::RoadNetwork>(m, "RoadNetwork")
      .def("road_geometry", &delphyne::roads::RoadNetwork::road_geometry, py::return_value_policy::reference_internal)
      .def("rulebook", &delphyne::roads::RoadNetwork::rulebook, py::return_value_policy::reference_internal)
      .def("traffic_light_book", &delphyne::roads::RoadNetwork::traffic_light_book,
           py::return_value_policy::reference_internal)
      .def("intersection_book", &delphyne::roads::RoadNetwork::intersection_book,
           py::return_value_policy::reference_internal)
      .def("phase_ring_book", &delphyne::roads::RoadNetwork::phase_ring_book,
           py::return_value_policy::reference_internal)
      .def("right_of_way_rule_state_provider", &delphyne::roads::RoadNetwork::right_of_way_rule_state_provider,
           py::return_value_policy::reference_internal)
      .def("phase_provider", &delphyne::roads::RoadNetwork::phase_provider, py::return_value_policy::reference_internal)
      .def("rule_registry", &delphyne::roads::RoadNetwork::rule_registry, py::return_value_policy::reference_internal)
      .def("discrete_value_rule_state_provider", &delphyne::roads::RoadNetwork::discrete_value_rule_state_provider,
           py::return_value_policy::reference_internal)
      .def("range_value_rule_state_provider", &delphyne::roads::RoadNetwork::range_value_rule_state_provider,
           py::return_value_policy::reference_internal)
      .def("Contains",
           py::overload_cast<const maliput::api::RoadPosition&>(&delphyne::roads::RoadNetwork::Contains, py::const_),
           py::arg("road_position"))
      .def("Contains",
           py::overload_cast<const maliput::api::LaneId&>(&delphyne::roads::RoadNetwork::Contains, py::const_),
           py::arg("lane_id"));
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace
