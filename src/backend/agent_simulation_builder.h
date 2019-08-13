// Copyright 2018 Toyota Research Institute

#pragma once

#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>
#include <maliput-utilities/generate_obj.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/rendering/pose_aggregator.h>
#include <drake/systems/rendering/pose_bundle.h>
#include <drake/systems/rendering/pose_bundle_to_draw_message.h>

#include <ignition/msgs.hh>

// public headers
#include "delphyne/mi6/agent_base.h"
#include "delphyne/mi6/agent_base_blueprint.h"
#include "delphyne/mi6/agent_simulation.h"
#include "systems/curve2.h"
#include "systems/lane_direction.h"
#include "visualization/car_vis_applicator.h"
#include "visualization/prius_vis.h"
#include "visualization/simple_prius_vis.h"

// private headers
#include "backend/ign_publisher_system.h"
#include "backend/ign_subscriber_system.h"
#include "backend/load_robot_aggregator.h"
#include "backend/scene_system.h"
#include "delphyne/macros.h"
#include "delphyne/protobuf/agent_state_v.pb.h"

namespace delphyne {

/// A builder for agent-based simulations.
///
/// For a system-level architecture diagram, see #5541.
///
/// @tparam One of double, delphyne::AutoDiff or delphyne::Symbolic.
///
/// Instantiated templates for the following types are provided:
/// - double
template <typename T>
class AgentSimulationBaseBuilder {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(AgentSimulationBaseBuilder)

  AgentSimulationBaseBuilder();

  /// Gets the RoadGeometry from this simulation
  const maliput::api::RoadGeometry* GetRoadGeometry() const;

  /// Sets the RoadGeometry for this simulation.
  ///
  /// @param road_geometry The road geometry to use for the simulation.
  const maliput::api::RoadGeometry* SetRoadGeometry(
      std::unique_ptr<const maliput::api::RoadGeometry> road_geometry);

  /// Sets the RoadGeometry for this simulation.
  ///
  /// @param road_geometry The road geometry to use for the simulation.
  /// @param features The road features that will be shown in the simulation.
  /// @see documentation of maliput::utility::ObjFeatures
  const maliput::api::RoadGeometry* SetRoadGeometry(
      std::unique_ptr<const maliput::api::RoadGeometry> road_geometry,
      const maliput::utility::ObjFeatures& features);

  /// Sets the RoadNetwork for this simulation and use its road geometry
  ///
  /// @param road_network The road network to use for the simulation.
  const maliput::api::RoadNetwork* SetRoadNetwork(
      std::unique_ptr<const maliput::api::RoadNetwork> road_network);

  /// Sets the RoadNetwork for this simulation.
  ///
  /// @param road_network The road network to use for the simulation.
  /// @param features The road features that will be shown in the simulation.
  /// @see documentation of maliput::utility::ObjFeatures
  const maliput::api::RoadNetwork* SetRoadNetwork(
      std::unique_ptr<const maliput::api::RoadNetwork> road_network,
      const maliput::utility::ObjFeatures& features);

  /// Constructs a Blueprint in-place and uses it to build and
  /// take ownership of an agent, which is to be wired into the
  /// simulation.
  ///
  /// @param args Blueprint constructor arguments.
  /// @returns A bare pointer to the added AgentBaseBlueprint<T>
  ///          instance, which will remain valid for the lifetime
  ///          of this simulation builder.
  /// @throws std::runtime_error if another agent with the same
  ///                            name has been wired already.
  /// @tparam Blueprint An AgentBaseBlueprint<T> subclass.
  /// @tparam BlueprintArgs Blueprint constructor arguments types.
  template <class Blueprint, typename... BlueprintArgs>
  Blueprint* AddAgent(BlueprintArgs&&... args) {
    return AddAgent(
        std::make_unique<Blueprint>(std::forward<BlueprintArgs>(args)...));
  }

  /// Constructs a Blueprint in-place and uses it to build and
  /// take ownership of an agent, which is to be wired into the
  /// simulation.
  ///
  /// @param args Blueprint constructor arguments.
  /// @returns A bare pointer to the added AgentBaseBlueprint<T>
  ///          instance, which will remain valid for the lifetime
  ///          of this simulation builder.
  /// @throws std::runtime_error if another agent with the same
  ///                            name has been wired already.
  /// @tparam Blueprint An AgentBaseBlueprint subclass, to be
  ///                   specialized for T.
  /// @tparam BlueprintArgs Blueprint constructor arguments types.
  template <template <typename U> class BaseBlueprint,
            typename... BlueprintArgs>
  BaseBlueprint<T>* AddAgent(BlueprintArgs&&... args) {
    return AddAgent(std::make_unique<BaseBlueprint<T>>(
        std::forward<BlueprintArgs>(args)...));
  }

  /// Uses the given @p blueprint to build and take ownership of
  /// an agent, which is to be wired into the simulation. Blueprint
  /// ownership is transferred to the built agent.
  ///
  /// @param blueprint The agent blueprint to be used.
  /// @returns A bare pointer to the added AgentBaseBlueprint<T>
  ///          instance, which will remain valid for the lifetime
  ///          of this simulation builder.
  /// @throws std::runtime_error if blueprint is nullptr.
  /// @throws std::runtime_error if another agent with the same
  ///                            name has been wired already.
  /// @tparam Blueprint An AgentBaseBlueprint<T> subclass.
  template <class Blueprint>
  Blueprint* AddAgent(std::unique_ptr<Blueprint> blueprint) {
    static_assert(std::is_base_of<AgentBaseBlueprint<T>, Blueprint>::value,
                  "Blueprint class is not an AgentBaseBlueprint subclass");
    DELPHYNE_VALIDATE(blueprint != nullptr, std::runtime_error,
                      "Invalid null blueprint was given");
    Blueprint* blueprint_ref = blueprint.get();
    DoAddAgent(blueprint_ref);
    blueprints_.push_back(std::move(blueprint));
    return blueprint_ref;
  }

  /// Builds the simulation.
  ///
  /// Construction of the simulation's Diagram representation is completed
  /// and a Simulator is built and initialized with it. All agents' ownership
  /// is transferred to the simulation instance.
  /// @returns Ownership of the AgentSimulationBase instance just built.
  std::unique_ptr<AgentSimulationBase<T>> Build();

  /// Resets the builder internal state, leaving it ready for
  /// another building procedure.
  void Reset();

  /// Sets the target real-time rate for the simulation to be built.
  /// @param realtime_rate The real-time rate to be set.
  /// @throws std::runtime_error if @p realtime_rate is a negative number.
  void SetTargetRealTimeRate(double realtime_rate) {
    DELPHYNE_VALIDATE(realtime_rate >= 0.0, std::runtime_error,
                      "Real-time rate must be a non-negative number");
    target_realtime_rate_ = realtime_rate;
  }

  /// Gets the target real-time rate for the simulation to be built.
  double GetTargetRealTimeRate() const { return target_realtime_rate_; }

  /// Whether the simulation's integrator should work on a fixed
  /// step basis or not
  bool UsesFixedStepMode() const { return fixed_step_mode_; }

  /// Gets maximum integration step size for the simulation to be built.
  const T& GetMaxStepSize() const { return max_step_size_; }

 private:
  // The rate at which the scene is published over ignition transport to
  // update the scene tree widget tree.
  static constexpr double kSceneTreePublishRateHz{4.0};

  // The name of the ignition transport topic over which the scene tree
  // is published.
  static constexpr const char* kSceneTreeTopicName{"scene"};

  // The rate at which scene updates are published over ignition transport to
  // update the scene rendering.
  static constexpr double kSceneUpdatesPublishRateHz{60.0};

  // The name of the ignition transport topic over which scene updates are
  // published for rendering.
  static constexpr const char* kSceneUpdatesTopicName{
      "visualizer/scene_update"};

  // The name of the ignition transport topic over which agents' states are
  // published.
  static constexpr const char* kAggregatedAgentsStateTopicName{"agents/state"};

  // Adds a generic agent to the simulation. See AddAgent() overloads.
  void DoAddAgent(AgentBaseBlueprint<T>* blueprint);

  // Adds scene publishing systems for the simulation to
  // be built and returns the scene publisher system.
  SceneSystem* AddScenePublishers();

  // Adds state publishing systems for all the agents in
  // the simulation to be built.
  void AddAgentStatePublishers();

  // The target real-time rate for the simulation to be built.
  double target_realtime_rate_{0.0};
  // Whether the simulation's integrator should work on a fixed
  // step basis or not (i.e. adaptive, error-controlled).
  bool fixed_step_mode_{true};
  // Maximum integration step size for the simulation to be built.
  T max_step_size_{0.01};

  // A builder for the simulation's Diagram representation.
  std::unique_ptr<drake::systems::DiagramBuilder<T>> builder_{nullptr};
  // An aggregator system for all AgentBase poses.
  drake::systems::rendering::PoseAggregator<T>* aggregator_{nullptr};
  // A visualization applicator system for agents' visuals, that takes their
  // poses and outputs the poses of their associated visual elements that make
  // up the visualization.
  CarVisApplicator<T>* car_vis_applicator_{nullptr};
  // A scene graph system for agents' collision geometries, necessary for
  // collision detection support.
  drake::geometry::SceneGraph<T>* scene_graph_{nullptr};

  // The sequence of agent IDs for this builder.
  int agent_id_sequence_{0};
  // The collection of all agents for the simulation to be built,
  // indexed by name.
  std::map<std::string, std::unique_ptr<AgentBase<T>>> agents_{};
  // The collection of all agent blueprints used for this simulation,
  // in order of execution.
  std::vector<std::unique_ptr<AgentBaseBlueprint<T>>> blueprints_{};

  // The geometry of the road for the simulation to be built.
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry_{
    nullptr};

  std::unique_ptr<const maliput::api::RoadNetwork> road_network_{
    nullptr};

  // The features of the road for the simulation to be built.
  maliput::utility::ObjFeatures road_features_{};
};

using AgentSimulationBuilder = AgentSimulationBaseBuilder<double>;
using AutoDiffAgentSimulationBuilder = AgentSimulationBaseBuilder<AutoDiff>;
using SymbolicAgentSimulationBuilder = AgentSimulationBaseBuilder<Symbolic>;

}  // namespace delphyne
