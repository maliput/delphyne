// Copyright 2017 Toyota Research Institute

#pragma once

#include <map>
#include <memory>
#include <string>

#include <drake/automotive/car_vis_applicator.h>
#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/rendering/pose_aggregator.h>

#include "./names.h"

namespace delphyne {
/// @brief The parent of all agents in delphyne!
///
/// This is the abstract class that all delphyne agents must inherit from.
/// Concrete implementations must implement both the 'Configure' method and the
/// 'Initialize' method; see the documentation for those methods for more
/// information.
///
/// @tparam T must generally be a "double-like" type.  Currently, the supported
///           template types are 'double', 'delphyne::AutoDiff', and
///           'delphyne::Symbolic'.
template <typename T>
class AgentBase {
 public:
  /// @brief Constructor initialising common agent parameters.
  ///
  /// @param name: name string for the agent (must be unique in any given
  /// simulation.
  explicit AgentBase(const std::string& name) : name_(name) {}
  virtual ~AgentBase() = default;

  /// @brief Used by the simulator to wire the agent into the simulation.
  ///
  /// @ref delphyne::AutomotiveSimulator "AutomotiveSimulator" will call
  /// this method to perform any simulator-specific configuration
  /// necessary for the agent. This includes generating an id, passing
  /// in the ground truth world information, diagram wiring and setting
  /// up the visuals.
  ///
  /// Creators of scenarios need never to call this method.
  ///
  /// @param id[in] A unique id, provided by the simulator
  /// to be stored by the agent for referential purposes.
  /// @param road_geometry[in] A handle for the agent to the ground truth
  /// road network information.
  /// @param builder[out] The diagram builder, use to wire systems ready for
  /// converting into the simulator's diagram.
  /// @param aggregator[out] Every agent should connect to this and publish
  /// it's state for access by all.
  /// @param car_vis_applicator:
  virtual int Configure(
      int id, const drake::maliput::api::RoadGeometry* road_geometry,
      drake::systems::DiagramBuilder<double>* builder,
      drake::systems::rendering::PoseAggregator<double>* aggregator,
      drake::automotive::CarVisApplicator<double>* car_vis_applicator) = 0;

  /// @brief Prepare the backgroudn context(s) for the agent in this method.
  ///
  /// Derived classes will typically use this to drop variable state on
  /// the context before simulation starts.
  ///
  /// This method is called by the
  /// @ref delphyne::AutomotiveSimulator "AutomotiveSimulator".
  ///
  /// @param[in] context the agent system's context
  ///
  /// @sa AgentBase::get_system()
  virtual int Initialize(drake::systems::Context<T>* context) = 0;

  /// @brief Name accessor
  const std::string& name() { return name_; }
  /// @brief ID accessor
  const int& id() { return id_; }

  /// @brief Accesses the system responsible for the 'defacto' car state.
  ///
  /// Access the system, currently used internally by
  /// @ref delphyne::AutomotiveSimulator "AutomotiveSimulator" to feed
  ///
  /// the Initialize method with the required sub-context.
  ///
  /// @warning: This method is misleading - note that agents will often
  /// be composed of multiple systems. The system extracted here should
  /// always be the one responsible for publishing the SimpleCarState.
  ///
  /// @todo(daniel.stonier): Drop this as soon as we manage to pre-declare
  /// state in the diagram building phase.
  virtual drake::systems::System<T>* get_system() const = 0;

 protected:
  // This id should be set by the simulator who is in charge of ensuring each
  // agent (system) in the simulation receives a unique id that can be passed
  // to register inputs on the pose aggregator
  int id_{0};
  std::string name_;
};

typedef AgentBase<double> Agent;
typedef AgentBase<AutoDiff> AutoDiffAgent;
typedef AgentBase<Symbolic> SymbolicAgent;

}  // namespace delphyne
