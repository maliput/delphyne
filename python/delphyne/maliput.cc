// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
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
#include <sstream>

#include <maliput/api/lane.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>
#include <maliput/math/math.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace py = pybind11;

namespace {

/*****************************************************************************
** Implementation
*****************************************************************************/

// TODO(@francocipollone): Removes bindings once maliput_py's bindings can be used here directly.
//                         These bindings are inspired on
//                         https://github.com/maliput/maliput_py/blob/0d91d3aec3fbe87c86d432c895eac7e4053d17bd/src/bindings/api_py.cc
PYBIND11_MODULE(maliput, m) {
  py::class_<maliput::api::InertialPosition>(m, "InertialPosition")
      .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("z"))
      .def("__eq__", &maliput::api::InertialPosition::operator==)
      .def("xyz", &maliput::api::InertialPosition::xyz, py::return_value_policy::reference_internal)
      .def("length", &maliput::api::InertialPosition::length)
      .def("Distance", &maliput::api::InertialPosition::Distance, py::arg("inertial_position"))
      .def("x", &maliput::api::InertialPosition::x)
      .def("set_x", &maliput::api::InertialPosition::set_x)
      .def("y", &maliput::api::InertialPosition::y)
      .def("set_y", &maliput::api::InertialPosition::set_y)
      .def("z", &maliput::api::InertialPosition::z)
      .def("set_z", &maliput::api::InertialPosition::set_z);

  py::class_<maliput::api::LanePosition>(m, "LanePosition")
      .def(py::init<double, double, double>(), py::arg("s"), py::arg("r"), py::arg("h"))
      .def("srh", &maliput::api::LanePosition::srh, py::return_value_policy::reference_internal)
      .def("s", &maliput::api::LanePosition::s)
      .def("set_s", &maliput::api::LanePosition::set_s)
      .def("r", &maliput::api::LanePosition::r)
      .def("set_r", &maliput::api::LanePosition::set_r)
      .def("h", &maliput::api::LanePosition::h)
      .def("set_h", &maliput::api::LanePosition::set_h);

  py::class_<maliput::api::LanePositionResult>(m, "LanePositionResult")
      .def(py::init<>())
      .def(py::init<const maliput::api::LanePosition&, const maliput::api::InertialPosition&, double>(),
           py::arg("lane_position"), py::arg("nearest_position"), py::arg("distance"))
      .def_readwrite("lane_position", &maliput::api::LanePositionResult::lane_position)
      .def_readwrite("nearest_position", &maliput::api::LanePositionResult::nearest_position)
      .def_readwrite("distance", &maliput::api::LanePositionResult::distance);

  py::class_<maliput::api::Rotation>(m, "Rotation")
      .def(py::init<>())
      .def("quat", &maliput::api::Rotation::quat, py::return_value_policy::reference_internal)
      .def("rpy", &maliput::api::Rotation::rpy);

  py::class_<maliput::api::RoadNetwork>(m, "RoadNetwork")
      .def("road_geometry", &maliput::api::RoadNetwork::road_geometry, py::return_value_policy::reference_internal);

  py::class_<maliput::api::RoadGeometry>(m, "RoadGeometry")
      .def("id", &maliput::api::RoadGeometry::id)
      .def("ById", &maliput::api::RoadGeometry::ById, py::return_value_policy::reference_internal)
      .def("junction", &maliput::api::RoadGeometry::junction, py::return_value_policy::reference_internal)
      .def("num_junctions", &maliput::api::RoadGeometry::num_junctions);

  py::class_<maliput::api::RoadGeometry::IdIndex>(m, "RoadGeometry.IdIndex")
      .def("GetLane", &maliput::api::RoadGeometry::IdIndex::GetLane, py::arg("id"),
           py::return_value_policy::reference_internal)
      .def("GetLanes", &maliput::api::RoadGeometry::IdIndex::GetLanes, py::return_value_policy::reference_internal)
      .def("GetSegment", &maliput::api::RoadGeometry::IdIndex::GetSegment, py::arg("id"),
           py::return_value_policy::reference_internal)
      .def("GetJunction", &maliput::api::RoadGeometry::IdIndex::GetJunction, py::arg("id"),
           py::return_value_policy::reference_internal);

  py::class_<maliput::api::JunctionId>(m, "JunctionId")
      .def(py::init<std::string>())
      .def(py::detail::hash(py::self))
      .def("string", &maliput::api::JunctionId::string)
      .def("__eq__", &maliput::api::JunctionId::operator==)
      .def("__repr__", [](const maliput::api::JunctionId& id) { return id.string(); });

  py::class_<maliput::api::Junction>(m, "Junction")
      .def("num_segments", &maliput::api::Junction::num_segments)
      .def("segment", &maliput::api::Junction::segment, py::return_value_policy::reference_internal)
      .def("id", &maliput::api::Junction::id, py::return_value_policy::reference_internal)
      .def("road_geometry", &maliput::api::Junction::road_geometry, py::return_value_policy::reference_internal);

  py::class_<maliput::api::SegmentId>(m, "SegmentId")
      .def(py::init<std::string>())
      .def(py::detail::hash(py::self))
      .def("string", &maliput::api::SegmentId::string)
      .def("__eq__", &maliput::api::SegmentId::operator==)
      .def("__repr__", [](const maliput::api::SegmentId& id) { return id.string(); });

  py::class_<maliput::api::Segment>(m, "Segment")
      .def("num_lanes", &maliput::api::Segment::num_lanes)
      .def("lane", &maliput::api::Segment::lane, py::return_value_policy::reference_internal)
      .def("junction", &maliput::api::Segment::num_lanes, py::return_value_policy::reference_internal)
      .def("id", &maliput::api::Segment::id, py::return_value_policy::reference_internal);

  py::class_<maliput::api::LaneId>(m, "LaneId")
      .def(py::init<std::string>())
      .def(py::detail::hash(py::self))
      .def("string", &maliput::api::LaneId::string)
      .def("__eq__", &maliput::api::LaneId::operator==)
      .def("__repr__", [](const maliput::api::LaneId& id) { return id.string(); });

  py::class_<maliput::api::Lane>(m, "Lane")
      .def("id", &maliput::api::Lane::id)
      .def("length", &maliput::api::Lane::length)
      .def("ToInertialPosition", &maliput::api::Lane::ToInertialPosition)
      .def("GetOrientation", &maliput::api::Lane::GetOrientation);

  py::class_<maliput::api::UniqueId>(m, "UniqueId")
      .def(py::init<const std::string&>())
      .def(py::detail::hash(py::self))
      .def("string", &maliput::api::UniqueId::string)
      .def("__repr__", &maliput::api::UniqueId::string)
      .def("__eq__", &maliput::api::UniqueId::operator==)
      .def("__ne__", &maliput::api::UniqueId::operator!=);

  py::class_<maliput::math::Vector3>(m, "Vector3")
      .def(py::init<double, double, double>())
      .def("__getitem__", py::overload_cast<std::size_t>(&maliput::math::Vector3::operator[]), py::is_operator())
      .def("__eq__", [](const maliput::math::Vector3& a, const maliput::math::Vector3& b) { return a == b; })
      .def("__ne__", [](const maliput::math::Vector3& a, const maliput::math::Vector3& b) { return a != b; })
      .def("__str__",
           [](const maliput::math::Vector3& self) {
             std::stringstream ss;
             ss << self;
             return ss.str();
           })
      .def("size", &maliput::math::Vector3::size)
      .def("x", py::overload_cast<>(&maliput::math::Vector3::x))
      .def("y", py::overload_cast<>(&maliput::math::Vector3::y))
      .def("z", py::overload_cast<>(&maliput::math::Vector3::z));

  py::class_<maliput::math::RollPitchYaw>(m, "RollPitchYaw")
      .def(py::init<double, double, double>())
      .def("__str__",
           [](const maliput::math::RollPitchYaw& self) {
             std::stringstream ss;
             ss << self.vector();
             return ss.str();
           })
      .def("ToQuaternion", &maliput::math::RollPitchYaw::ToQuaternion)
      .def("roll_angle", py::overload_cast<>(&maliput::math::RollPitchYaw::roll_angle))
      .def("pitch_angle", py::overload_cast<>(&maliput::math::RollPitchYaw::pitch_angle))
      .def("yaw_angle", py::overload_cast<>(&maliput::math::RollPitchYaw::yaw_angle));
}

}  // namespace
