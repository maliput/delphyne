// Copyright 2017 Toyota Research Institute

#pragma once

/*****************************************************************************
** Includes
*****************************************************************************/

#include <memory>
#include <set>
#include <string>

#include <drake/common/eigen_types.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/output_port.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/value.h>
#include <drake/systems/framework/vector_base.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/rendering/pose_vector.h>
#include <drake/systems/rendering/frame_velocity.h>
#include <drake/geometry/geometry_ids.h>

#include "delphyne/macros.h"
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
  /// Specific diagram for this agent. As returned by BuildDiagram().
  using Diagram = DiagramBundle<T>;
  /// Specific builder for this agent. To be used inside BuildDiagram().
  using DiagramBuilder = AgentDiagramBuilder<T>;

  /// @brief Constructor initialising common agent parameters.
  ///
  /// @param name[in]: Convenient descriptive name for the agent
  /// (must be unique in any given simulation).
  explicit AgentBase(const std::string& name) : name_(name) {}
  virtual ~AgentBase() = default;

  /// Builds this Agent's Diagram representation into the given
  /// @p builder of the containing simulation Diagram.
  /// @param builder The builder for the simulation Diagram.
  /// @returns A reference to this Agent's Diagram representation.
  /// @throws std::runtime_error if this Agent has been built into
  ///                            a simulation via BuildInto() already.
  Diagram* BuildInto(drake::systems::DiagramBuilder<T>* builder) {
    DELPHYNE_VALIDATE(diagram_ == nullptr, std::runtime_error,
                      "This agent is already associated with a simulation!");
    diagram_ = builder->AddSystem(BuildDiagram());
    return diagram_;
  }

  /// Grabs this Agent's Context from the @p context of the containing
  /// simulation @p diagram.
  /// @param diagram The simulation Diagram.
  /// @param context The simulation Context.
  /// @throws std::runtime_error if this Agent has not been built into
  ///                            any simulation via BuildInto() yet.
  void GrabContextFrom(const drake::systems::Diagram<T>& diagram,
                       drake::systems::Context<T>* context) {
    DELPHYNE_VALIDATE(diagram_ != nullptr, std::runtime_error,
                      "This agent is not associated to any simulation!");
    context_ = &(diagram.GetMutableSubsystemContext(*diagram_, context));
  }

  /// Gets the Agent's pose in the simulation.
  /// @throws std::runtime_error if this Agent has not been built into
  ///                            any simulation via BuildInto() yet.
  /// @throws std::runtime_error if this Agent has not been given a Context
  ///                            via GrabContextFrom() yet.
  drake::Isometry3<T> GetPose() const {
    DELPHYNE_VALIDATE(diagram_ != nullptr, std::runtime_error,
                      "This agent is not associated to any simulation!");
    DELPHYNE_VALIDATE(context_ != nullptr, std::runtime_error,
                      "This agent is not part of a running simulation!");
    const drake::systems::OutputPort<T>& pose_output_port =
        diagram_->get_output_port("pose");
    std::unique_ptr<drake::systems::AbstractValue> port_value =
        pose_output_port.Allocate();
    pose_output_port.Calc(*context_, port_value.get());
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

  /// Gets the Agent's twist in the simulation.
  /// @throws std::runtime_error if this Agent has not been built into
  ///                            any simulation via BuildInto() yet.
  /// @throws std::runtime_error if this Agent has not been given a Context
  ///                            via GrabContextFrom() yet.
  drake::TwistVector<T> GetVelocity() const {
    DELPHYNE_VALIDATE(diagram_ != nullptr, std::runtime_error,
                      "This agent is not associated to any simulation!");
    DELPHYNE_VALIDATE(context_ != nullptr, std::runtime_error,
                      "This agent is not part of a running simulation!");
    const drake::systems::OutputPort<T>& vel_output_port =
        diagram_->get_output_port("velocity");
    std::unique_ptr<drake::systems::AbstractValue> port_value =
        vel_output_port.Allocate();
    vel_output_port.Calc(*context_, port_value.get());
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
  // TODO(daniel.stonier) stop using this, make use of an
  // initial value on the pose output (used by geometry settings for
  // the collision subsystem)
  drake::Isometry3<double> initial_world_pose_;

 private:
  // Builds the Diagram representation for this Agent.
  virtual std::unique_ptr<Diagram> BuildDiagram() const = 0;

  // The name of this Agent.
  std::string name_{};

  // A reference to this Agent's Diagram representation
  // in simulation.
  Diagram* diagram_{};

  // A reference to this Agent's simulation context.
  drake::systems::Context<T>* context_{};

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
