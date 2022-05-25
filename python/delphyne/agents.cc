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

#include <string>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "agents/mobil_car.h"
#include "agents/rail_car.h"
#include "agents/rule_rail_car.h"
#include "agents/simple_car.h"
#include "agents/trajectory_agent.h"
#include "agents/unicycle_car.h"
#include "delphyne/mi6/agent_base.h"
#include "delphyne/mi6/agent_base_blueprint.h"
#include "delphyne/mi6/agent_simulation.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace py = pybind11;

using delphyne::Agent;
using delphyne::AgentBlueprint;
using delphyne::AgentSimulation;
using delphyne::BasicAgentBlueprint;
using delphyne::MobilCarBlueprint;
using delphyne::RailCar;
using delphyne::RailCarBlueprint;
using delphyne::RuleRailCar;
using delphyne::RuleRailCarBlueprint;
using delphyne::SimpleCarBlueprint;
using delphyne::TrajectoryAgentBlueprint;
using delphyne::UnicycleCarAgent;
using delphyne::UnicycleCarBlueprint;

namespace {

/*****************************************************************************
** Implementation
*****************************************************************************/

PYBIND11_MODULE(agents, m) {
  py::module::import("maliput.api");

  py::class_<Agent>(m, "Agent")
      .def("name", &Agent::name)
      .def("get_pose_rotation", [](const Agent* self) { return self->GetPose().get_rotation().coeffs(); })
      .def("get_pose_translation", [](const Agent* self) { return self->GetPose().get_translation().vector(); })
      .def("get_velocity", &Agent::GetVelocity);

  py::class_<RailCar, Agent>(m, "RailCar").def("set_speed", &RailCar::SetSpeed);

  py::class_<UnicycleCarAgent, Agent>(m, "UnicycleCarAgent")
      .def("set_acceleration", &UnicycleCarAgent::SetAcceleration)
      .def("set_angular_rate", &UnicycleCarAgent::SetAngularRate);

  // clang-format off
  py::class_<AgentBlueprint>(m, "AgentBlueprint")
      .def("get_agent",
           [](AgentBlueprint* self, const AgentSimulation& simulation) { return &self->GetAgent(simulation); },
           py::return_value_policy::reference_internal)
      .def("get_mutable_agent", &AgentBlueprint::GetMutableAgent, py::return_value_policy::reference_internal);
  // clang-format on
  py::class_<SimpleCarBlueprint, AgentBlueprint>(m, "SimpleCarBlueprint")
      .def(py::init<const std::string&, double, double, double, double>(), "Construct and configure a simple car",
           py::arg("name"), py::arg("x"), py::arg("y"), py::arg("heading"), py::arg("speed"));

  py::class_<MobilCarBlueprint, AgentBlueprint>(m, "MobilCarBlueprint")
      .def(py::init<const std::string&, bool, double, double, double, double>(), "Construct and configure a mobil car",
           py::arg("name"), py::arg("direction_of_travel"), py::arg("x"), py::arg("y"), py::arg("heading"),
           py::arg("speed"));

  py::class_<RailCarBlueprint, AgentBlueprint>(m, "RailCarBlueprint")
      .def(py::init<const std::string&, const ::maliput::api::Lane&, bool, double, double, double, double>(),
           "Construct and configure a rail car", py::arg("name"), py::arg("lane"), py::arg("direction_of_travel"),
           py::arg("longitudinal_position"), py::arg("lateral_offset"), py::arg("speed"), py::arg("nominal_speed"));

  py::class_<RuleRailCarBlueprint, AgentBlueprint>(m, "RuleRailCarBlueprint")
      .def(py::init<const std::string&, const ::maliput::api::Lane&, bool, double, double, double, double>(),
           "Construct and configure a rule rail car", py::arg("name"), py::arg("lane"), py::arg("direction_of_travel"),
           py::arg("longitudinal_position"), py::arg("lateral_offset"), py::arg("speed"), py::arg("nominal_speed"));

  py::class_<TrajectoryAgentBlueprint, AgentBlueprint>(m, "TrajectoryAgentBlueprint")
      .def(py::init<const std::string&, const std::vector<double>&, const std::vector<double>&,
                    const std::vector<std::vector<double>>&>(),
           "Construct and configure a trajectory agent", py::arg("name"), py::arg("times"), py::arg("headings"),
           py::arg("translations"));

  py::class_<UnicycleCarBlueprint, AgentBlueprint>(m, "UnicycleCarBlueprint")
      .def(py::init<const std::string&, double, double, double, double>(), "Construct and configure a unicycle car",
           py::arg("name"), py::arg("x"), py::arg("y"), py::arg("heading"), py::arg("speed"));
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace
