// Copyright 2018 Toyota Research Institute

#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/automotive/maliput/api/road_network.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/rendering/pose_bundle.h>

#include <ignition/msgs.hh>

// public headers
#include "delphyne/macros.h"
#include "delphyne/mi6/agent_base.h"
#include "delphyne/protobuf/agent_state_v.pb.h"

namespace delphyne {

// Forward declaration
template <typename T>
class AgentBase;

// Forward declaration.
class SceneSystem;

/// A collision between any two AgentBase instances, along
/// with the global coordinates of the point-of-collision.
/// @tparam T must be a valid Eigen ScalarType.
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
/// - AutoDiff
/// - Symbolic
template <typename T>
struct AgentBaseCollision {
  AgentBaseCollision(const std::pair<AgentBase<T>*, AgentBase<T>*> agents_in,
                     const drake::Vector3<T>& location_in)
      : agents(agents_in), location(location_in) {}

  const std::pair<AgentBase<T>*, AgentBase<T>*> agents;  ///< Pair of agents in
                                                         ///  in collision.
  const drake::Vector3<T> location;  ///< Location of the point-of-collision.
};

using AgentCollision = AgentBaseCollision<double>;
using AutoDiffAgentCollision = AgentBaseCollision<AutoDiff>;
using SymbolicAgentCollision = AgentBaseCollision<Symbolic>;

/// A runnable agent-based simulation, using Drake's system framework
/// as its backbone.
///
/// @tparam T must be a valid Eigen ScalarType.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
/// These are already available to link against in the containing library.
template <typename T>
class AgentSimulationBase {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(AgentSimulationBase)

  /// Constructs a simulation.
  /// @param simulator the Simulator instance to advance this simulation.
  /// @param diagram the Diagram representation of this simulation.
  /// @param agents all the Agents associated with this simulation.
  /// @param road_geometry the RoadGeometry associated with this simulation.
  /// @param scene_graph a reference to the SceneGraph in this simulation,
  ///                    for all sorts of geometrical queries.
  /// @param scene_system a reference to the scene composing system in this
  ///                     simulation
  explicit AgentSimulationBase(
      std::unique_ptr<drake::systems::Simulator<T>> simulator,
      std::unique_ptr<drake::systems::Diagram<T>> diagram,
      std::map<std::string, std::unique_ptr<AgentBase<T>>> agents,
      std::unique_ptr<const drake::maliput::api::RoadGeometry> road_geometry,
      drake::geometry::SceneGraph<T>* scene_graph, SceneSystem* scene_system);

  /// Constructs a simulation.
  /// @param simulator the Simulator instance to advance this simulation.
  /// @param diagram the Diagram representation of this simulation.
  /// @param agents all the Agents associated with this simulation.
  /// @param road_network the RoadNetwork associated with this simulation.
  /// @param scene_graph a reference to the SceneGraph in this simulation,
  ///                    for all sorts of geometrical queries.
  /// @param scene_system a reference to the scene composing system in this
  ///                     simulation
  explicit AgentSimulationBase(
      std::unique_ptr<drake::systems::Simulator<T>> simulator,
      std::unique_ptr<drake::systems::Diagram<T>> diagram,
      std::map<std::string, std::unique_ptr<AgentBase<T>>> agents,
      std::unique_ptr<const drake::maliput::api::RoadNetwork> road_network,
      drake::geometry::SceneGraph<T>* scene_graph, SceneSystem* scene_system);

  /// Returns a reference to the `name`d agent of the given type.
  ///
  /// @param[in] name The name of the agent.
  /// @throws std::runtime_error if no agent with the given `name`
  ///                            is known by the simulator.
  /// @throws std::bad_cast if the agent was found but it is not of
  ///                       the expected type.
  /// @tparam AgentType An AgentBase<T> subclass.
  template <class AgentType>
  const AgentType& GetAgentByName(const std::string& name) {
    static_assert(std::is_base_of<AgentBase<T>, AgentType>::value,
                  "Expected type is not an agent type.");
    return dynamic_cast<const AgentType&>(GetAgentByName(name));
  }

  /// Returns a mutable reference to the `name`d agent of the given type.
  ///
  /// @param[in] name The name of the agent.
  /// @throws std::runtime_error if no agent with the given `name`
  ///                            is known by the simulator.
  /// @throws std::bad_cast if the agent was found but it is not of
  ///                       the expected type.
  /// @tparam AgentBaseType An AgentBase subclass, to be specialized
  ///                       for T.
  template <template <typename U> class AgentBaseType>
  const AgentBaseType<T>& GetAgentByName(const std::string& name) {
    static_assert(std::is_base_of<AgentBase<T>, AgentBaseType<T>>::value,
                  "Expected type is not an agent type.");
    return dynamic_cast<const AgentBaseType<T>&>(GetAgentByName(name));
  }

  /// Returns a reference to the `name`d agent.
  ///
  /// @param[in] name The name of the agent.
  /// @throws std::runtime_error if no agent with the given `name`
  ///                           is known by the simulator.
  const AgentBase<T>& GetAgentByName(const std::string& name) const;

  /// Returns a mutable reference to the `name`d agent of the given
  /// type.
  ///
  /// @param[in] name The name of the agent.
  /// @returns a pointer to the agent or nullptr if it is not of
  ///          the expected type.
  /// @throws std::runtime_error if no agent with the given `name`
  ///                            is known by the simulator.
  /// @tparam AgentType An AgentBase<T> subclass.
  template <class AgentType>
  AgentType* GetMutableAgentByName(const std::string& name) {
    static_assert(std::is_base_of<AgentBase<T>, AgentType>::value,
                  "Expected type is not an agent type.");
    return dynamic_cast<AgentType*>(GetMutableAgentByName(name));
  }

  /// Returns a mutable reference to the `name`d agent of the given
  /// type.
  ///
  /// @param[in] name The name of the agent.
  /// @returns a pointer to the agent or nullptr if it is not of
  ///          the expected type.
  /// @throws std::runtime_error if no agent with the given `name`
  ///                            is known by the simulator.
  /// @tparam AgentBaseType An AgentBase subclass, to be specialized
  ///                       for T.
  template <template <typename U> class AgentBaseType>
  AgentBaseType<T>* GetMutableAgentByName(const std::string& name) {
    static_assert(std::is_base_of<AgentBase<T>, AgentBaseType<T>>::value,
                  "Expected type is not an agent type.");
    return dynamic_cast<AgentBaseType<T>*>(GetMutableAgentByName(name));
  }

  /// Returns a mutable reference to the `name`d agent.
  ///
  /// @param[in] name The name of the agent.
  /// @throws std::runtime_error if no agent with the given `name`
  ///                           is known by the simulator.
  AgentBase<T>* GetMutableAgentByName(const std::string& name);

  /// Returns the simulation scene, full of visuals.
  std::unique_ptr<ignition::msgs::Scene> GetVisualScene();

  /// Returns the current poses of all agents in the simulation.
  drake::systems::rendering::PoseBundle<T> GetCurrentPoses() const;

  /// Returns all agent pairs that are currently in collision.
  ///
  /// @remarks The order in which collision pairs are returned may
  ///          vary with the collision detection backend used and thus
  ///          no order is enforced. DO NOT expect nor rely on any given
  ///          order.
  std::vector<AgentBaseCollision<T>> GetCollisions() const;

  /// Sets the real-time rate for this simulation.
  /// @see systems::Simulator::set_target_realtime_rate
  void SetRealTimeRate(double realtime_rate) {
    simulator_->set_target_realtime_rate(realtime_rate);
  }

  /// Gets the real-time rate for this simulation.
  /// @see systems::Simulator::get_target_realtime_rate
  double GetRealTimeRate() const {
    return simulator_->get_target_realtime_rate();
  }

  /// Advances simulated time by the given @p time_step in seconds.
  void StepBy(const T& time_step);

  /// Returns the current simulation time in seconds.
  /// @see Simulator::Context::get_time.
  const T& GetCurrentTime() const { return GetContext().get_time(); }

  /// Gets a reference to the simulation diagram representation.
  const drake::systems::Diagram<T>& GetDiagram() const { return *diagram_; }

  /// Gets a reference to the simulation context.
  const drake::systems::Context<T>& GetContext() const {
    return simulator_->get_context();
  }

  /// Gets a mutable reference to the simulation context.
  drake::systems::Context<T>* GetMutableContext() {
    return &simulator_->get_mutable_context();
  }

 private:
  // The simulator to advance this simulation in time.
  std::unique_ptr<drake::systems::Simulator<T>> simulator_;
  // The diagram representation for this simulation.
  std::unique_ptr<drake::systems::Diagram<T>> diagram_;
  // The collection of all agents in the simulation, indexed by name.
  std::map<std::string, std::unique_ptr<AgentBase<T>>> agents_;
  // The geometry of the road in this simulation.
  std::unique_ptr<const drake::maliput::api::RoadGeometry> road_geometry_;
  // The network of the road in this simulation.
  std::unique_ptr<const drake::maliput::api::RoadNetwork> road_network_;
  // The scene graph with all collision geometries in this simulation.
  drake::geometry::SceneGraph<T>* scene_graph_;
  // The system that publishes this simulation as a scene for rendering
  // purposes.
  SceneSystem* scene_system_;
  // The scene query associated with the `scene_graph_` in this simulation,
  // useful for geometrical queries (e.g. collision detections).
  std::unique_ptr<drake::AbstractValue> scene_query_;
};

using AgentSimulation = AgentSimulationBase<double>;
using AutoDiffAgentSimulation = AgentSimulationBase<AutoDiff>;
using SymbolicAgentSimulation = AgentSimulationBase<Symbolic>;

}  // namespace delphyne
