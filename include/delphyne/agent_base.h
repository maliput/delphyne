// Copyright 2017 Toyota Research Institute

#pragma once

#include <set>
#include <string>

#include <drake/automotive/car_vis_applicator.h>
#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/rendering/pose_aggregator.h>

#include "delphyne/types.h"

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
  /// in the ground truth world information, diagram wiring, geometry
  /// registration and setting up the visuals.
  ///
  /// Creators of scenarios need never to call this method.
  ///
  /// @param[in] id A unique id, provided by the simulator
  /// to be stored by the agent for referential purposes.
  /// @param[in] road_geometry A handle for the agent to the ground truth
  /// road network information.
  /// @param[out] builder The diagram builder, use to wire systems ready for
  /// converting into the simulator's diagram.
  /// @param[out] scene_graph The global scene graph for agents to register
  /// geometry.
  /// @param[out] aggregator Every agent should connect to this and publish
  /// it's state for access by all.
  /// @param[out] car_vis_applicator The applicator to delegate agent
  /// visualizations to.
  /// @return Non zero on failure, zero on success.
  virtual void Configure(
      int id, const drake::maliput::api::RoadGeometry* road_geometry,
      drake::systems::DiagramBuilder<T>* builder,
      drake::geometry::SceneGraph<T>* scene_graph,
      drake::systems::rendering::PoseAggregator<T>* aggregator,
      drake::automotive::CarVisApplicator<T>* car_vis_applicator) = 0;

  /// Checks whether this agent is the source for the given
  /// @p geometry_id (i.e. has registered the geometry associated
  /// with that id) or not.
  virtual bool is_source_of(
      const drake::geometry::GeometryId& geometry_id) const {
    return (geometry_ids_.count(geometry_id) != 0);
  }

  /// @brief Name accessor
  const std::string& name() { return name_; }
  /// @brief ID accessor
  const int& id() { return id_; }

 protected:
  // This id should be set by the simulator who is in charge of ensuring each
  // agent (system) in the simulation receives a unique id that can be passed
  // to register inputs on the pose aggregator
  int id_{0};
  std::string name_;
  // The set of geometry IDs associated with the geometries registered
  // by this agent upon configuration.
  std::set<drake::geometry::GeometryId> geometry_ids_{};
};

typedef AgentBase<double> Agent;
typedef AgentBase<AutoDiff> AutoDiffAgent;
typedef AgentBase<Symbolic> SymbolicAgent;

}  // namespace delphyne
