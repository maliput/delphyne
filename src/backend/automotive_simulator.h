// Copyright 2017 Toyota Research Institute

#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "drake/automotive/car_vis_applicator.h"
#include "drake/automotive/curve2.h"
#include "drake/automotive/gen/maliput_railcar_state.h"
#include "drake/automotive/gen/trajectory_car_state.h"
#include "drake/automotive/idm_controller.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput_railcar.h"
#include "drake/automotive/mobil_planner.h"
#include "drake/automotive/pure_pursuit_controller.h"
#include "drake/automotive/simple_car.h"
#include "drake/automotive/trajectory_car.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/pose_aggregator.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

// public headers
#include "delphyne/agent_base.h"

// private headers
#include "backend/ign_publisher_system.h"
#include "backend/ign_subscriber_system.h"
#include "backend/load_robot_aggregator.h"
#include "backend/scene_system.h"
#include "backend/system.h"

namespace delphyne {

/// AutomotiveSimulator is a helper class for constructing and running
/// automotive-related simulations.
///
/// @tparam T must be a valid Eigen ScalarType.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
/// They are already available to link against in the containing library.
template <typename T>
class AutomotiveSimulator {
 public:
  AutomotiveSimulator();

  /// Returns the DiagramBuilder.
  /// @pre Start() has NOT been called.
  drake::systems::DiagramBuilder<T>* get_builder();

  /// Return the scene.
  std::unique_ptr<ignition::msgs::Scene> GetScene();

  /**
   * @brief Adds an agent to the simulation.
   *
   * The user should have custom constructed this agent from
   * a child-class of @ref delphyne::AgentBase<T> "AgentBase". In
   * turn, the simulator then calls this agent's configure method
   * to perform the necessary system configuration and wiring to
   * ready this agent for use in the simulation.
   *
   * @param agent[in] The user provided agent to add to the simulation.
   * @return A simulator generated unqiue id for the agent.
   */
  int AddAgent(std::unique_ptr<delphyne::AgentBase<T>> agent);

  /// Sets the RoadGeometry for this simulation.
  ///
  /// @pre Start() has NOT been called.
  const drake::maliput::api::RoadGeometry* SetRoadGeometry(
      std::unique_ptr<const drake::maliput::api::RoadGeometry> road);

  /// Returns the System whose name matches @p name.  Throws an exception if no
  /// such system has been added, or multiple such systems have been added.
  //
  /// This is the builder variant of the method.  It can only be used prior to
  /// Start() being called.
  ///
  /// @pre Start() has NOT been called.
  drake::systems::System<T>& GetBuilderSystemByName(std::string name);

  /// Returns the System whose name matches @p name.  Throws an exception if no
  /// such system has been added, or multiple such systems have been added.
  ///
  /// This is the diagram variant of the method, which can only be used after
  /// Start() is called.
  ///
  /// @pre Start() has been called.
  const drake::systems::System<T>& GetDiagramSystemByName(
      std::string name) const;

  /// Builds the Diagram.  No further changes to the diagram may occur after
  /// this has been called.
  ///
  /// @pre Build() has NOT been called.
  void Build();

  /// Returns the System containing the entire AutomotiveSimulator diagram.
  ///
  /// @pre Build() has been called.
  const drake::systems::System<T>& GetDiagram() const { return *diagram_; }

  /// Returns the current poses of all vehicles in the simulation.
  ///
  /// @pre Start() has been called.
  drake::systems::rendering::PoseBundle<T> GetCurrentPoses() const;

  /// Calls Build() on the diagram (if it has not been build already) and
  /// initializes the Simulator.  No further changes to the diagram may occur
  /// after this has been called.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @param realtime_rate This value is passed to
  /// systems::Simulator::set_target_realtime_rate().
  void Start(double realtime_rate = 0.0);

  /// Returns whether the automotive simulator has started.
  bool has_started() const { return simulator_ != nullptr; }

  /// Advances simulated time by the given @p time_step increment in seconds.
  void StepBy(const T& time_step);

  /// Returns the current simulation time in seconds.
  /// @see documentation of Simulator::Context::get_time.
  double get_current_simulation_time() const;

 private:
  // The rate at which the scene is published over ign-transport to update the
  // scene tree widget tree.
  const double kScenePublishPeriodMs = 250.0;

  // Verifies that the provided `name` of an agent is unique among all agents
  // that have been added to the `AutomotiveSimulator`. Throws a
  // std::runtime_error if it is not unique meaning an agent of the same name
  // was already added.
  void CheckNameUniqueness(const std::string& name);

  // Connects the provided pose and velocity output ports of a vehicle model to
  // the PoseAggregator and adds a PriusVis for visualizing the vehicle.
  void ConnectCarOutputsAndPriusVis(
      int id, const drake::systems::OutputPort<T>& pose_output,
      const drake::systems::OutputPort<T>& velocity_output);

  // Generates the URDF model of the road network and loads it into the
  // `RigidBodyTree`. Member variable `road_` must be set prior to calling this
  // method.
  void GenerateAndLoadRoadNetworkUrdf();

  // Fixes the scene geometry aggregator input port. This is performed on the
  // context, not the system itself, so this function requires the simulator to
  // have been created (since it owns the context).
  void InitializeSceneGeometryAggregator();

  // For both building and simulation.
  std::unique_ptr<const drake::maliput::api::RoadGeometry> road_geometry_{};

  // === Start for building. ===
  std::unique_ptr<RigidBodyTree<T>> tree_{std::make_unique<RigidBodyTree<T>>()};

  std::unique_ptr<drake::systems::DiagramBuilder<T>> builder_{
      std::make_unique<drake::systems::DiagramBuilder<T>>()};

  // Holds the desired initial states of each loadable agent. It is used to
  // initialize the simulation's diagram's state.
  std::map<int, std::unique_ptr<drake::systems::BasicVector<T>>>
      loadable_agent_initial_states_;

  // The output port of the Diagram that contains pose bundle information.
  int pose_bundle_output_port_{};

  // === End for building. ===

  // Adds the PoseAggregator.
  drake::systems::rendering::PoseAggregator<T>* aggregator_{};

  // Takes the poses of the vehicles and outputs the poses of the visual
  // elements that make up the visualization of the vehicles. For a system-level
  // architecture diagram, see #5541.
  drake::automotive::CarVisApplicator<T>* car_vis_applicator_{};

  // Aggregates multiple lcmt_viewer_load_robot messages into a single one
  // containing all models in the scene.
  LoadRobotAggregator* load_robot_aggregator_{};

  // Creates a scene message from a geometry description and up-to-date poses
  // for non-static elements.
  SceneSystem* scene_system_{};

  // Takes the output of car_vis_applicator_ and creates an lcmt_viewer_draw
  // message containing the latest poses of the visual elements.
  drake::systems::rendering::PoseBundleToDrawMessage* bundle_to_draw_{};

  // Every system in the Diagram must have a unique system id, we put the
  // simulator in charge of the unique id counter (better than global objects
  // with static id counters since they may run into threading problems)
  int unique_system_id_{0};

  // Maps from simulator generated unique id's to the agents.
  std::map<int, std::unique_ptr<delphyne::AgentBase<T>>> agents_;

  // For simulation.
  std::unique_ptr<drake::systems::Diagram<T>> diagram_{};
  std::unique_ptr<drake::systems::Simulator<T>> simulator_{};
};

}  // namespace delphyne