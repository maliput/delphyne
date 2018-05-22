/**
 * @file /delphyne/src/agents/bindings.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "agents/trajectory_agent.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace py = pybind11;

namespace {

/*****************************************************************************
** Implementation
*****************************************************************************/

PYBIND11_MODULE(agent_bindings, m) {
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.maliput.api");

  py::class_<delphyne::TrajectoryAgent>(
      m, "TrajectoryAgent")
      .def(py::init<>());
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}
