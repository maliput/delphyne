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

#include <memory>
#include <set>
#include <string>

#include <drake/common/eigen_types.h>
#include <drake/systems/framework/diagram_builder.h>
#include <maliput/api/road_network.h>

#include "delphyne/macros.h"
#include "delphyne/mi6/agent_base.h"
#include "delphyne/mi6/agent_diagram_builder.h"
#include "delphyne/mi6/agent_simulation.h"
#include "delphyne/mi6/diagram_bundle.h"
#include "delphyne/types.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/

// Forward declaration
template <typename T>
class AgentBase;

// Forward declaration
template <typename T>
class AgentSimulationBase;

/// The abstract blueprint class for agents in Delphyne.
///
/// This is the abstract class that all Delphyne agent blueprints must
/// inherit from. Concrete implementations are required to implement the
/// DoBuildInto() method, to build each agent sensor-planner-control systems
/// into a a simulation's diagram.
///
/// @tparam T One of double, delphyne::AutoDiff or delphyne::Symbolic.
template <typename T>
class AgentBaseBlueprint {
 public:
  /// Diagram builder type for this agent blueprint.
  using DiagramBuilder = AgentDiagramBuilder<T>;

  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(AgentBaseBlueprint)

  /// Constructs a blueprint for an agent with the given name.
  ///
  /// @param name The name for the agent, must be unique in any
  ///             given simulation.
  explicit AgentBaseBlueprint(const std::string& name) : name_(name) {}

  virtual ~AgentBaseBlueprint() = default;

  /// Returns a reference to the agent associated with this blueprint
  /// inside the given @p simulation
  ///
  /// @param[in] simulation Simulation instance where the agent lives.
  /// @see AgentSimulationBase<T>::GetAgentByName()
  virtual const AgentBase<T>& GetAgent(const AgentSimulationBase<T>& simulation) const {
    return simulation.GetAgentByName(this->name());
  }

  /// Returns a mutable reference to the agent associated with this blueprint
  /// inside the given @p simulation
  ///
  /// @param[in] simulation Simulation instance where the agent lives.
  /// @see AgentSimulationBase<T>::GetMutableAgentByName()
  virtual AgentBase<T>* GetMutableAgent(AgentSimulationBase<T>* simulation) {
    return simulation->GetMutableAgentByName(this->name());
  }

  /// Builds the agen Agent's Diagram representation into the given
  /// @p builder of the containing simulation Diagram.
  /// @param road_network A maliput::api::RoadNetwork pointer.
  /// @param builder The builder for the simulation Diagram.
  /// @returns Ownership of the agent just built.
  /// @throws std::runtime_error if builder is nullptr.
  std::unique_ptr<AgentBase<T>> BuildInto(maliput::api::RoadNetwork* road_network,
                                          drake::systems::DiagramBuilder<T>* builder) {
    DELPHYNE_VALIDATE(builder != nullptr, std::runtime_error, "Given diagram builder is null");
    return DoBuildInto(road_network, builder);
  }

  /// Gets the name for the agent.
  const std::string& name() const { return name_; }

  // Gets a mutable reference to the @p agent Diagram representation.
  typename AgentBase<T>::Diagram* GetMutableDiagram(AgentBase<T>* agent) const { return agent->diagram_; }

  // Gets a mutable reference to the @p agent geometry IDs.
  std::set<drake::geometry::GeometryId>* GetMutableGeometryIDs(AgentBase<T>* agent) const {
    return &agent->geometry_ids_;
  }

 private:
  // Builds the Diagram representation for the agent.
  virtual std::unique_ptr<AgentBase<T>> DoBuildInto(maliput::api::RoadNetwork* road_network,
                                                    drake::systems::DiagramBuilder<T>* builder) const = 0;

  // The name for the agent to be built.
  std::string name_{};
};

/// An abstract but typed blueprint class for agents in Delphyne.
///
/// Concrete implementations are required to implement the DoBuildAgentInto()
/// method, yielding ownership of a constructed agent of type @p A.
///
/// @tparam T One of double, delphyne::AutoDiff or delphyne::Symbolic.
/// @tparam A An AgentBase<T> derived class.
template <typename T, class A>
class TypedAgentBaseBlueprint : public AgentBaseBlueprint<T> {
 public:
  static_assert(std::is_base_of<AgentBase<T>, A>::value, "Class is not an AgentBase derived class.");
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(TypedAgentBaseBlueprint)

  using AgentBaseBlueprint<T>::AgentBaseBlueprint;

  // Leveraging return type covariance support for references.
  const A& GetAgent(const AgentSimulationBase<T>& simulation) const override {
    return dynamic_cast<const A&>(AgentBaseBlueprint<T>::GetAgent(simulation));
  }

  // Leveraging return type covariance support for pointers.
  A* GetMutableAgent(AgentSimulationBase<T>* simulation) override {
    return dynamic_cast<A*>(AgentBaseBlueprint<T>::GetMutableAgent(simulation));
  }

 private:
  std::unique_ptr<AgentBase<T>> DoBuildInto(maliput::api::RoadNetwork* road_network,
                                            drake::systems::DiagramBuilder<T>* builder) const final {
    return DoBuildAgentInto(road_network, builder);
  }

  // DoBuildInto() variation to cope with the lack of support
  // for type covariance when dealing with smart pointers.
  virtual std::unique_ptr<A> DoBuildAgentInto(maliput::api::RoadNetwork* road_network,
                                              drake::systems::DiagramBuilder<T>* builder) const = 0;
};

/// A simplified abstract and typed blueprint for agents.
///
/// @tparam T One of double, delphyne::AutoDiff or delphyne::Symbolic.
/// @tparam A An AgentBase<T> derived class.
template <typename T, class A>
class BasicTypedAgentBaseBlueprint : public TypedAgentBaseBlueprint<T, A> {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(BasicTypedAgentBaseBlueprint)

  using TypedAgentBaseBlueprint<T, A>::TypedAgentBaseBlueprint;

 private:
  std::unique_ptr<A> DoBuildAgentInto(maliput::api::RoadNetwork* road_network,
                                      drake::systems::DiagramBuilder<T>* builder) const override {
    return std::make_unique<A>(builder->AddSystem(DoBuildDiagram(road_network)));
  }

  // Builds a Diagram representation for the agent.
  // @param road_network The road network for the simulation
  //                      the agent is to be built into. May be
  //                      nullptr.
  // @returns Ownership of the agent's Diagram representation.
  virtual std::unique_ptr<typename AgentBase<T>::Diagram> DoBuildDiagram(
      maliput::api::RoadNetwork* road_network) const = 0;
};

/*****************************************************************************
** Typedefs
*****************************************************************************/

using AgentBlueprint = AgentBaseBlueprint<double>;
using AutoDiffAgentBlueprint = AgentBaseBlueprint<AutoDiff>;
using SymbolicAgentBlueprint = AgentBaseBlueprint<Symbolic>;

template <class A>
using TypedAgentBlueprint = TypedAgentBaseBlueprint<double, A>;
template <class A>
using AutoDiffTypedAgentBlueprint = TypedAgentBaseBlueprint<AutoDiff, A>;
template <class A>
using SymbolicTypedAgentBlueprint = TypedAgentBaseBlueprint<Symbolic, A>;

template <class A>
using BasicTypedAgentBlueprint = BasicTypedAgentBaseBlueprint<double, A>;
template <class A>
using BasicAutoDiffTypedAgentBlueprint = BasicTypedAgentBaseBlueprint<AutoDiff, A>;
template <class A>
using BasicSymbolicTypedAgentBlueprint = BasicTypedAgentBaseBlueprint<Symbolic, A>;

using BasicAgentBlueprint = BasicTypedAgentBaseBlueprint<double, AgentBase<double>>;
using BasicAutoDiffAgentBlueprint = BasicTypedAgentBaseBlueprint<AutoDiff, AgentBase<AutoDiff>>;
using BasicSymbolicAgentBlueprint = BasicTypedAgentBaseBlueprint<Symbolic, AgentBase<Symbolic>>;

}  // namespace delphyne
