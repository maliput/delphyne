// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
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

#pragma once

/*****************************************************************************
** Includes
*****************************************************************************/

#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include <drake/common/drake_copyable.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/framework_common.h>  // OutputPortIndex
#include <drake/systems/framework/input_port.h>
#include <drake/systems/framework/output_port.h>

#include "delphyne/macros.h"
#include "delphyne/types.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/

namespace detail {

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
template <typename Base, typename T>
class NamedPortSystem : public Base {
 public:
  static_assert(std::is_base_of<drake::systems::System<T>, Base>::value, "Only a System can have their ports mapped.");

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NamedPortSystem);

  template <typename... Args>
  explicit NamedPortSystem(Args... args) : Base(std::forward<Args>(args)...) {}

  void set_input_names(std::map<std::string, drake::systems::InputPortIndex> inputs_mapping) {
    inputs_mapping_ = inputs_mapping;
  }

  void set_output_names(std::map<std::string, drake::systems::OutputPortIndex> outputs_mapping) {
    outputs_mapping_ = outputs_mapping;
  }

  const drake::systems::InputPort<T>& get_input_port(int port_index) const {
    // Caveat: `System::get_input_port` is not virtual.
    return Base::get_input_port(port_index);
  }

  const drake::systems::InputPort<T>& get_input_port(const std::string& port_name) const {
    DELPHYNE_VALIDATE(inputs_mapping_.count(port_name) != 0, std::runtime_error,
                      "Input port \"" + port_name + "\" could not be found.");
    return get_input_port(inputs_mapping_.at(port_name));
  }

  const drake::systems::OutputPort<T>& get_output_port(int port_index) const {
    // Caveat: `System::get_output_port` is not virtual.
    return Base::get_output_port(port_index);
  }

  const drake::systems::OutputPort<T>& get_output_port(const std::string& port_name) const {
    DELPHYNE_VALIDATE(outputs_mapping_.count(port_name) != 0, std::runtime_error,
                      "Output port \"" + port_name + "\" could not be found.");
    return this->get_output_port(outputs_mapping_.at(port_name));
  }

 private:
  std::map<std::string, drake::systems::InputPortIndex> inputs_mapping_{};
  std::map<std::string, drake::systems::OutputPortIndex> outputs_mapping_{};
};

}  // namespace detail

template <typename T>
class DiagramBundle : public detail::NamedPortSystem<drake::systems::Diagram<T>, T> {
 public:
  explicit DiagramBundle(drake::systems::DiagramBuilder<T>* builder) { builder->BuildInto(this); }
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
