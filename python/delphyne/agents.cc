/**
 * @file python/delphyne/agents.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <vector>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "agents/mobil_car.h"
#include "agents/rail_car.h"
#include "agents/simple_car.h"
#include "agents/trajectory_agent.h"
#include "delphyne/mi6/agent_base.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace py = pybind11;

namespace {

/*****************************************************************************
** Implementation
*****************************************************************************/

PYBIND11_MODULE(agents, m) {
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.maliput.api");

  py::class_<delphyne::Agent>(m, "AgentBase")
      .def("name", &delphyne::Agent::name);

  py::class_<delphyne::MobilCar, delphyne::Agent>(m, "MobilCar")
      .def(py::init<const std::string&, bool, double, double, double, double,
                    const drake::maliput::api::RoadGeometry&>(),
           "Construct and configure a mobil car", py::arg("name"),
           py::arg("direction_of_travel"), py::arg("x"), py::arg("y"),
           py::arg("heading"), py::arg("speed"), py::arg("road_geometry"));

  py::class_<delphyne::RailCar, delphyne::Agent>(m, "RailCar")
      .def(py::init<const std::string&, const drake::maliput::api::Lane&, bool,
                    double, double, double, double,
                    const drake::maliput::api::RoadGeometry&>(),
           "Construct and configure a rail car", py::arg("name"),
           py::arg("lane"), py::arg("direction_of_travel"),
           py::arg("longitudinal_position"), py::arg("lateral_offset"),
           py::arg("speed"), py::arg("nominal_speed"),
           py::arg("road_geometry"));

  py::class_<delphyne::SimpleCar, delphyne::Agent>(m, "SimpleCar")
      .def(py::init<const std::string&, double, double, double, double>(),
           "Construct and configure a simple car", py::arg("name"),
           py::arg("x"), py::arg("y"), py::arg("heading"), py::arg("speed"));

  py::class_<delphyne::TrajectoryAgent, delphyne::Agent>(m, "TrajectoryAgent")
      .def(py::init<const std::string&, const std::vector<double>&,
                    const std::vector<double>&,
                    const std::vector<std::vector<double>>&>(),
           "Construct and configure a trajectory agent", py::arg("name"),
           py::arg("times"), py::arg("headings"), py::arg("translations"));
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace
