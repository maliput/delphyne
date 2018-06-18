// Copyright 2018 Toyota Research Institute

#pragma once

/*****************************************************************************
** Includes
*****************************************************************************/

#include <map>
#include <memory>
#include <string>

#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/framework_common.h>  // OutputPortIndex
#include <drake/systems/framework/input_port_descriptor.h>

#include "delphyne/types.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/// @brief A generic bundle containing and describing a diagram.
///
/// There's no easy way to introspect on the input/output ports of a
/// diagram or even enumerate the indices of the port since they are
/// dynamically generated at build time, so this passes along the
/// indices with the diagram so that it can be encapsulated inside other
/// diagrams and wired up with that diagram's systems.
///
/// To avoid zombie diagram bundles, this class should be exclusively
/// used with unique pointers. Copy/Move/Assign capabilities are
/// disabled.
///
/// @tparam One of double, delphyne::AutoDiff or delphyne::Symbolic.
template <typename T>
struct DiagramBundle {
  DiagramBundle() = default;
  DiagramBundle(const DiagramBundle<T>& other) = delete;
  DiagramBundle<T>& operator=(const DiagramBundle<T>&) = delete;

  std::unique_ptr<drake::systems::Diagram<T>> diagram{};
  std::map<std::string, drake::systems::OutputPortIndex> outputs;
  std::map<std::string, drake::systems::InputPortIndex> inputs;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
