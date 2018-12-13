// Copyright 2018 Toyota Research Institute

#pragma once

/*****************************************************************************
** Includes
*****************************************************************************/

#include <memory>
#include <set>
#include <string>

#include <drake/common/eigen_types.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/output_port.h>
#include <drake/systems/framework/value.h>
#include <drake/systems/framework/vector_base.h>
#include <drake/systems/rendering/frame_velocity.h>
#include <drake/systems/rendering/pose_vector.h>

#include "delphyne/macros.h"
#include "delphyne/mi6/agent_base_blueprint.h"
#include "delphyne/mi6/diagram_bundle.h"
#include "delphyne/types.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/

// Forward declaration.
template <typename T>
class AgentBaseBlueprint;

/// The most basic agent in Delphyne.
///
/// @tparam One of double, delphyne::AutoDiff or delphyne::Symbolic.
template <typename T>
class AgentBase {
 public:
  /// Diagram type for this agent.
  using Diagram = DiagramBundle<T>;

  /// Constructs basic agent, associated with the given @p diagram.
  ///
  /// @param diagram A reference to the Diagram representation
  ///                for the agent, owned by the simulation.
  /// @throws std::runtime_error if @p diagram is nullptr.
  explicit AgentBase(Diagram* diagram) : diagram_(diagram), context_(nullptr) {
    DELPHYNE_VALIDATE(diagram != nullptr, std::runtime_error,
                      "Invalid null diagram representation given");
  }

  virtual ~AgentBase() = default;

  /// Resets this agent's Context to @p context.
  /// @param context A reference to the agent's Diagram
  ///                representation Context, owned by the
  ///                simulation.
  /// @throws std::runtime_error if @p context is nullptr.
  void SetContext(drake::systems::Context<T>* context) {
    DELPHYNE_VALIDATE(context != nullptr, std::runtime_error,
                      "Invalid null context given");
    context_ = context;
  }

  /// Gets a reference to the agent's Context.
  const drake::systems::Context<T>& GetContext() const {
    DELPHYNE_VALIDATE(context_ != nullptr, std::runtime_error,
                      "This agent has no simulation context yet");
    return *context_;
  }

  /// Gets a mutable reference to the agent's Context.
  drake::systems::Context<T>* GetMutableContext() const { return context_; }

  /// Gets a reference to the agent's Diagram representation.
  const Diagram& GetDiagram() const { return *diagram_; }

  /// Gets the agent pose in the simulation.
  /// @throws std::runtime_error if this agent lacks Context.
  drake::Isometry3<T> GetPose() const {
    constexpr const char* const kPosePortName = "pose";
    const drake::systems::OutputPort<T>& pose_output_port =
        GetDiagram().get_output_port(kPosePortName);
    std::unique_ptr<drake::systems::AbstractValue> port_value =
        pose_output_port.Allocate();
    pose_output_port.Calc(GetContext(), port_value.get());
    // TODO(hidmic): figure out why type assertions fail if
    // trying to get the port value with the correct type in
    // a single step (presumably related to the fact that
    // BasicVector defines Clone() but its subclasses only
    // inherit it, see drake::is_cloneable).
    using drake::systems::BasicVector;
    using drake::systems::rendering::PoseVector;
    const BasicVector<T>& base_vector =
        port_value->template GetValueOrThrow<BasicVector<T>>();
    const PoseVector<T>& pose_vector =
        dynamic_cast<const PoseVector<T>&>(base_vector);
    return pose_vector.get_isometry();
  }

  /// Gets the agent twist in the simulation.
  /// @throws std::runtime_error if this agent lacks Context.
  drake::TwistVector<T> GetVelocity() const {
    constexpr const char* const kVelocityPortName = "velocity";
    const drake::systems::OutputPort<T>& vel_output_port =
        GetDiagram().get_output_port(kVelocityPortName);
    std::unique_ptr<drake::systems::AbstractValue> port_value =
        vel_output_port.Allocate();
    vel_output_port.Calc(GetContext(), port_value.get());
    // TODO(hidmic): figure out why type assertions fail if
    // trying to get the port value with the correct type in
    // a single step (presumably related to the fact that
    // BasicVector defines Clone() but its subclasses only
    // inherit it, see drake::is_cloneable).
    using drake::systems::BasicVector;
    using drake::systems::rendering::FrameVelocity;
    const BasicVector<T>& base_vector =
        port_value->template GetValueOrThrow<BasicVector<T>>();
    const FrameVelocity<T>& frame_velocity =
        dynamic_cast<const FrameVelocity<T>&>(base_vector);
    return frame_velocity.get_velocity().get_coeffs();
  }

  /// Gets the agent name.
  const std::string& name() const { return GetDiagram().get_name(); }

  /// Gets a reference to the agent's geometry IDs.
  const std::set<drake::geometry::GeometryId>& GetGeometryIDs() const {
    return geometry_ids_;
  }

  /// Checks whether this agent is the source of the given
  /// @p geometry_id (i.e. has registered the geometry associated
  /// with that id) or not.
  virtual bool IsSourceOf(
      const drake::geometry::GeometryId& geometry_id) const {
    return (geometry_ids_.count(geometry_id) != 0);
  }

 private:
  // Give the associated blueprint access to the agent internals.
  friend AgentBaseBlueprint<T>;

  // A reference to this agent's Diagram representation in simulation.
  Diagram* diagram_{nullptr};

  // A reference to this agent's Context in simulation.
  drake::systems::Context<T>* context_{nullptr};

  // The set of geometry IDs registered for this agent's
  // collision geometries.
  std::set<drake::geometry::GeometryId> geometry_ids_{};
};

/*****************************************************************************
** Typedefs
*****************************************************************************/

using Agent = AgentBase<double>;
using AutoDiffAgent = AgentBase<AutoDiff>;
using SymbolicAgent = AgentBase<Symbolic>;

}  // namespace delphyne
