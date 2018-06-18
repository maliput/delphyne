// Copyright 2017 Toyota Research Institute

#pragma once

/*****************************************************************************
** Includes
*****************************************************************************/

#include <memory>
#include <set>
#include <string>

#include <drake/common/eigen_types.h>
#include <drake/geometry/geometry_ids.h>

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

/// @brief The parent of all agents in delphyne!
///
/// This is the abstract class that all delphyne agents must inherit from.
/// Concrete implementations are required to implement the BuildDiagram()
/// method which packages this agent's sensor-planner-control systems
/// into a single diagram for encapsulation into the simulator's main
/// diagram.
///
/// ::DiagramBuilder and ::DiagramBundle can be used to assist in the
/// diagram construction. In particular, refer to the documentation of
/// ::DiagramBuilder for information about the agent diagram structure
/// and convenience methods.
///
/// @tparam One of double, delphyne::AutoDiff or delphyne::Symbolic.
template <typename T>
class AgentBase {
 public:
  /// Type for packaging diagram and indices together in a bundle.
  using DiagramBundle = delphyne::DiagramBundle<T>;
  /// Specific builder for this agent. Use inside BuildDiagram().
  using DiagramBuilder = AgentDiagramBuilder<T>;

  /// @brief Constructor initialising common agent parameters.
  ///
  /// @param name[in]: Convenient descriptive name for the agent
  /// (must be unique in any given simulation).
  explicit AgentBase(const std::string& name) : name_(name) {}
  virtual ~AgentBase() = default;

  /// @brief The agent diagram builder
  ///
  /// This diagram encloses the systems needed for the agent to run and
  /// will ultimately be encapsulated in the simulator's diagram.
  ///
  /// @return DiagramBundle : The diagram along with the relevant input
  /// and output indices for an agent diagram that allow it to be
  /// wired up with systems in the simulator diagram.
  virtual std::unique_ptr<DiagramBundle> BuildDiagram() const = 0;

  /// @brief Name accessor
  const std::string& name() const { return name_; }

  /// @brief Accessor to the geometry ids
  const std::set<drake::geometry::GeometryId>& geometry_ids() const {
    return geometry_ids_;
  }

  /// @brief Mutable accessor to the geometry ids
  std::set<drake::geometry::GeometryId>& mutable_geometry_ids() {
    return geometry_ids_;
  }

  /// @brief Accessor for the initial world pose of the agent.
  const drake::Isometry3<double>& initial_world_pose() const {
    return initial_world_pose_;
  }

  /// Checks whether this agent is the source for the given
  /// @p geometry_id (i.e. has registered the geometry associated
  /// with that id) or not.
  virtual bool is_source_of(
      const drake::geometry::GeometryId& geometry_id) const {
    return (geometry_ids_.count(geometry_id) != 0);
  }

 protected:
  std::string name_;
  // TODO(daniel.stonier) stop using this, make use of an
  // initial value on the pose output (used by geometry settings for
  // the collision subsystem)
  drake::Isometry3<double> initial_world_pose_;

 private:
  // TODO(daniel.stonier) dubious whether we should have
  // simulator specific machinery here
  std::set<drake::geometry::GeometryId> geometry_ids_{};
};

/*****************************************************************************
** Typedefs
*****************************************************************************/

using Agent = AgentBase<double>;
using AutoDiffAgent = AgentBase<AutoDiff>;
using SymbolicAgent = AgentBase<Symbolic>;

}  // namespace delphyne
