/**
 * @file /delphyne/python/bindings/agents.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <vector>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "delphyne/agent_base.h"

#include "agents/mobil_car.h"
#include "agents/rail_car.h"
#include "agents/simple_car.h"
#include "agents/trajectory_agent.h"

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

  py::class_<delphyne::Agent>(m, "AgentBase");

  py::class_<delphyne::MobilCar, delphyne::Agent>(m, "MobilCar")
      .def(py::init<const std::string&, const bool&, const double&,
                    const double&, const double&, const double&>());

  py::class_<delphyne::RailCar, delphyne::Agent>(m, "RailCar")
      .def(py::init<const std::string&, const drake::maliput::api::Lane&,
                    const bool&, const double&, const double&, const double&,
                    const double&>());

  py::class_<delphyne::SimpleCar, delphyne::Agent>(m, "SimpleCar")
      .def(py::init<const std::string&, const double&, const double&,
                    const double&, const double&>());

  py::class_<delphyne::TrajectoryAgent, delphyne::Agent>(m, "TrajectoryAgent")
      .def(py::init<const std::string&, const std::vector<double>&,
                    const std::vector<double>&,
                    const std::vector<std::vector<double>>&>());
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace
