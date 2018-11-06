// Copyright 2018 Toyota Research Institute

#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <type_traits>

#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/automotive/maliput/utility/generate_obj.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/rigid_body_tree.h>
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
#include "systems/curve2.h"
#include "systems/lane_direction.h"
#include "visualization/car_vis_applicator.h"
#include "visualization/prius_vis.h"
#include "visualization/simple_prius_vis.h"

// private headers
#include "backend/geometry_wiring.h"
#include "backend/ign_publisher_system.h"
#include "backend/ign_subscriber_system.h"
#include "backend/load_robot_aggregator.h"
#include "backend/scene_system.h"
#include "backend/simulation.h"
#include "delphyne/macros.h"
#include "delphyne/protobuf/agent_state_v.pb.h"

namespace delphyne {

/// A builder for simulations.
///
/// For a system-level architecture diagram, see #5541.
///
/// @tparam One of double, delphyne::AutoDiff or delphyne::Symbolic.
///
/// Instantiated templates for the following types are provided:
/// - double
template <typename T>
class SimulationBaseBuilder {
 public:
  SimulationBaseBuilder();

  /// Sets the RoadGeometry for this simulation.
  ///
  /// @param road_geometry The road geometry to use for the simulation.
  const drake::maliput::api::RoadGeometry* SetRoadGeometry(
      std::unique_ptr<const drake::maliput::api::RoadGeometry> road_geometry);

  /// Sets the RoadGeometry for this simulation.
  ///
  /// @param road_geometry The road geometry to use for the simulation.
  /// @param features The road features that will be shown in the simulation.
  /// @see documentation of drake::maliput::utility::ObjFeatures
  const drake::maliput::api::RoadGeometry* SetRoadGeometry(
      std::unique_ptr<const drake::maliput::api::RoadGeometry> road_geometry,
      const drake::maliput::utility::ObjFeatures& features);

  /// Constructs a Blueprint in-place and uses it to build and
  /// take ownership of an agent, which is to be wired into the
  /// simulation. Blueprint ownership is transferred to the built
  /// agent.
  ///
  /// @param args Blueprint constructor arguments.
  /// @returns A bare pointer to the built AgentBase<T> instance,
  ///          which will remain valid for the lifetime of the
  ///          simulation instance built by this builder.
  /// @throws std::runtime_error if another agent with the same
  ///                            name has been wired already.
  /// @tparam Blueprint An AgentBaseBlueprint<T> subclass.
  /// @tparam BlueprintArgs Blueprint constructor arguments types.
  template<class Blueprint, typename... BlueprintArgs>
  auto AddAgent(BlueprintArgs&&... args) {
    return AddAgent(std::make_unique<Blueprint>(
        std::forward<BlueprintArgs>(args)...));
  }

  /// Constructs a Blueprint in-place and uses it to build and
  /// take ownership of an agent, which is to be wired into the
  /// simulation. Blueprint ownership is transferred to the built
  /// agent.
  ///
  /// @param args Blueprint constructor arguments.
  /// @returns A bare pointer to the built AgentBase<T> instance,
  ///          which will remain valid for the lifetime of the
  ///          simulation instance built by this builder.
  /// @throws std::runtime_error if another agent with the same
  ///                            name has been wired already.
  /// @tparam Blueprint An AgentBaseBlueprint subclass, to be
  ///                   specialized for T.
  /// @tparam BlueprintArgs Blueprint constructor arguments types.
  template<template <typename U> class BaseBlueprint,
           typename... BlueprintArgs>
  auto AddAgent(BlueprintArgs&&... args) {
    return AddAgent(std::make_unique<BaseBlueprint<T>>(
        std::forward<BlueprintArgs>(args)...));
  }

  /// Uses the given @p blueprint to build and take ownership of
  /// an agent, which is to be wired into the simulation. Blueprint
  /// ownership is transferred to the built agent.
  ///
  /// @param blueprint The agent blueprint to be used.
  /// @returns A bare pointer to the built AgentBase<T> instance,
  ///          which will remain valid for the lifetime of the
  ///          simulation instance built by this builder.
  /// @throws std::runtime_error if blueprint is nullptr.
  /// @throws std::runtime_error if another agent with the same
  ///                            name has been wired already.
  /// @tparam Blueprint An AgentBaseBlueprint<T> subclass.
  template <class Blueprint>
  auto AddAgent(std::unique_ptr<Blueprint> blueprint) {
    static_assert(std::is_base_of<AgentBaseBlueprint<T>, Blueprint>::value,
                  "Blueprint class is not an AgentBaseBlueprint subclass");
    DELPHYNE_VALIDATE(blueprint != nullptr, std::runtime_error,
                      "Invalid null blueprint was given");
    auto agent = blueprint->BuildInto(road_geometry_.get(), builder_.get());
    agent->SetBlueprint(std::move(blueprint));

    const int agent_id = agent_id_sequence_++;
    const std::string& agent_name = agent->name();
    DELPHYNE_VALIDATE(agents_.count(agent_name) == 0, std::runtime_error,
                      "An agent named \"" + agent_name + "\" already exists.");

    // Wires up the agent's ports.
    typename AgentBase<T>::Diagram* agent_diagram = agent->GetMutableDiagram();
    drake::systems::rendering::PoseVelocityInputPorts<double> ports =
        aggregator_->AddSinglePoseAndVelocityInput(agent_name, agent_id);
    builder_->Connect(agent_diagram->get_output_port("pose"),
                      ports.pose_input_port);
    builder_->Connect(agent_diagram->get_output_port("velocity"),
                      ports.velocity_input_port);
    builder_->Connect(aggregator_->get_output_port(0),
                      agent_diagram->get_input_port("traffic_poses"));

    // Registers and wires up a Prius geometry for both visuals and collision
    // geometries.

    // TODO(daniel.stonier) this just enforces ... 'everything is a prius'.
    // We'll need a means of having the agents report what visual they have and
    // hooking that up. Also wondering why visuals are in the drake diagram?
    car_vis_applicator_->AddCarVis(
        std::make_unique<SimplePriusVis<T>>(agent_id, agent_name));

    builder_->Connect(
        agent_diagram->get_output_port("pose"),
        WirePriusGeometry(
            agent_name, agent->GetBlueprint().GetInitialWorldPose(),
            builder_.get(), scene_graph_, agent->GetMutableGeometryIDs()));

    auto agent_ref = agent.get();
    agents_[agent_name] = std::move(agent);
    return agent_ref;
  }

  /// Builds the simulation.
  ///
  /// Construction of the simulation's Diagram representation is completed
  /// and a Simulator is built and initialized with it. All agents' ownership
  /// is transferred to the simulation instance.
  /// @returns Ownership of the SimulationBase instance just built.
  std::unique_ptr<SimulationBase<T>> Build();

  /// Resets the builder internal state.
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

  // The geometry of the road for the simulation to be built.
  std::unique_ptr<const drake::maliput::api::RoadGeometry> road_geometry_{nullptr};

  // The world tree representation for the simulation to be built.
  std::unique_ptr<RigidBodyTree<T>> world_tree_{nullptr};
};

using SimulationBuilder = SimulationBaseBuilder<double>;
using AutoDiffSimulationBuilder = SimulationBaseBuilder<AutoDiff>;
using SymbolicSimulationBuilder = SimulationBaseBuilder<Symbolic>;

}  // namespace delphyne
