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

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "agents/mobil_car.h"
#include "agents/rail_car.h"
#include "agents/simple_car.h"
#include "agents/trajectory_agent.h"
#include "delphyne/mi6/agent_base.h"
#include "delphyne/mi6/agent_base_blueprint.h"
#include "delphyne/mi6/agent_simulation.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace py = pybind11;

using delphyne::Agent;
using delphyne::RailCar;
using delphyne::AgentBlueprint;
using delphyne::AgentSimulation;
using delphyne::BasicAgentBlueprint;
using delphyne::SimpleCarBlueprint;
using delphyne::RailCarBlueprint;
using delphyne::MobilCarBlueprint;
using delphyne::TrajectoryAgentBlueprint;

namespace {

/*****************************************************************************
** Implementation
*****************************************************************************/

PYBIND11_MODULE(agents, m) {
  py::module::import("pydrake.common.eigen_geometry");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.maliput.api");

  py::class_<Agent>(m, "Agent")
      .def("name", &Agent::name)
      .def("get_pose", &Agent::GetPose)
      .def("get_velocity", &Agent::GetVelocity);

  py::class_<RailCar, Agent>(m, "RailCar").def("set_speed", &RailCar::SetSpeed);

  py::class_<AgentBlueprint>(m, "AgentBlueprint")
      .def("get_agent",
           [](AgentBlueprint* self, const AgentSimulation& simulation) {
             return &self->GetAgent(simulation);
           })
      .def("get_mutable_agent", &AgentBlueprint::GetMutableAgent);

  py::class_<SimpleCarBlueprint, AgentBlueprint>(m, "SimpleCarBlueprint")
      .def(py::init<const std::string&, double, double, double, double>(),
           "Construct and configure a simple car", py::arg("name"),
           py::arg("x"), py::arg("y"), py::arg("heading"), py::arg("speed"));

  py::class_<MobilCarBlueprint, AgentBlueprint>(m, "MobilCarBlueprint")
      .def(py::init<const std::string&, bool, double, double, double, double>(),
           "Construct and configure a mobil car", py::arg("name"),
           py::arg("direction_of_travel"), py::arg("x"), py::arg("y"),
           py::arg("heading"), py::arg("speed"));

  py::class_<RailCarBlueprint, AgentBlueprint>(m, "RailCarBlueprint")
      .def(py::init<const std::string&, const drake::maliput::api::Lane&, bool,
                    double, double, double, double>(),
           "Construct and configure a rail car", py::arg("name"),
           py::arg("lane"), py::arg("direction_of_travel"),
           py::arg("longitudinal_position"), py::arg("lateral_offset"),
           py::arg("speed"), py::arg("nominal_speed"));

  py::class_<TrajectoryAgentBlueprint, AgentBlueprint>(
      m, "TrajectoryAgentBlueprint")
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
