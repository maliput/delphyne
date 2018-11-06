// Copyright 2018 Toyota Research Institute

#pragma once

/*****************************************************************************
** Includes
*****************************************************************************/

#include <memory>
#include <set>
#include <string>

#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/common/eigen_types.h>
#include <drake/systems/framework/diagram_builder.h>

#include "delphyne/macros.h"
#include "delphyne/mi6/agent_base.h"
#include "delphyne/mi6/agent_diagram_builder.h"
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

/// The abstract blueprint class for agents in Delphyne!
///
/// This is the abstract class that all Delphyne agent blueprints must
/// inherit from. Concrete implementations are required to implement the
/// DoBuildInto() method, to build each agent sensor-planner-control systems
/// into a a simulation's diagram.
///
/// @tparam One of double, delphyne::AutoDiff or delphyne::Symbolic.
///
/// Instantiated templates for the following types are provided:
/// - double
template <typename T>
class AgentBaseBlueprint {
 public:
  /// Diagram builder type for this agent blueprint.
  using DiagramBuilder = AgentDiagramBuilder<T>;

  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(AgentBaseBlueprint)

  /// Constructs a blueprint for an agent with the given name and
  /// located at the given initial pose.
  ///
  /// @param name The name for the agent, must be unique in any
  ///             given simulation.
  /// @param initial_world_pose The initial pose of the agent.
  explicit AgentBaseBlueprint(const std::string& name,
                              const drake::Isometry3<T>& initial_world_pose)
      : name_(name), initial_world_pose_(initial_world_pose) {
  }

  /// Constructs a blueprint for an agent with the given name and
  /// located at the origin.
  ///
  /// @param name The name for the agent, must be unique in any
  ///             given simulation.
  explicit AgentBaseBlueprint(const std::string& name)
      : AgentBaseBlueprint(name, drake::Isometry3<T>::Identity()) {
  }

  virtual ~AgentBaseBlueprint() = default;

  /// Builds the agen Agent's Diagram representation into the given
  /// @p builder of the containing simulation Diagram.
  /// @param builder The builder for the simulation Diagram.
  /// @returns Ownership of the agent just built.
  /// @throws std::runtime_error if builder is nullptr.
  std::unique_ptr<AgentBase<T>> BuildInto(
      const drake::maliput::api::RoadGeometry* road_geometry,
      drake::systems::DiagramBuilder<T>* builder) {
    DELPHYNE_VALIDATE(builder != nullptr, std::runtime_error,
                      "Given diagram builder is null");
    return DoBuildInto(road_geometry, builder);
  }

  /// Gets the name for the agent.
  const std::string& name() const { return name_; }

  /// Gets the initial world pose for the agent.
  const drake::Isometry3<T>& GetInitialWorldPose() const {
    return initial_world_pose_;
  }

  /// Sets the initial world pose for the agent.
  void SetInitialWorldPose(const drake::Isometry3<T>& pose) {
    initial_world_pose_ = pose;
  }

 private:
  // Builds the Diagram representation for the agent.
  virtual std::unique_ptr<AgentBase<T>> DoBuildInto(
      const drake::maliput::api::RoadGeometry* road_geometry,
      drake::systems::DiagramBuilder<T>* builder) const = 0;

  // The name for the agent to be built.
  std::string name_{};

  // The initial world pose for the agent to be built.
  drake::Isometry3<T> initial_world_pose_{};
};

/// A simplified abstract blueprint for agents.
///
/// @tparam One of double, delphyne::AutoDiff or delphyne::Symbolic.
///
/// Instantiated templates for the following types are provided:
/// - double
template <typename T>
class SimpleAgentBaseBlueprint : public AgentBaseBlueprint<T> {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleAgentBaseBlueprint)

  using AgentBaseBlueprint<T>::AgentBaseBlueprint;

 private:
  std::unique_ptr<AgentBase<T>> DoBuildInto(
      const drake::maliput::api::RoadGeometry* road_geometry,
      drake::systems::DiagramBuilder<T>* builder) const override {
    return std::make_unique<AgentBase<T>>(
        builder->AddSystem(DoBuildDiagram(road_geometry)));
  }

  // Builds a Diagram representation for the agent.
  // @param road_geometry The road geometry for the simulation
  //                      the agent is to be built into. May be
  //                      nullptr.
  // @returns Ownership of the agent's Diagram representation.
  virtual std::unique_ptr<typename AgentBase<T>::Diagram> DoBuildDiagram(
      const drake::maliput::api::RoadGeometry* road_geometry) const = 0;
};


/*****************************************************************************
** Typedefs
*****************************************************************************/

using AgentBlueprint = AgentBaseBlueprint<double>;
using AutoDiffAgentBlueprint = AgentBaseBlueprint<AutoDiff>;
using SymbolicAgentBlueprint = AgentBaseBlueprint<Symbolic>;

using SimpleAgentBlueprint = SimpleAgentBaseBlueprint<double>;
using SimpleAutoDiffAgentBlueprint = SimpleAgentBaseBlueprint<AutoDiff>;
using SimpleSymbolicAgentBlueprint = SimpleAgentBaseBlueprint<Symbolic>;

}  // namespace delphyne
