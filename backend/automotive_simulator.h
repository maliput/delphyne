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
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/rendering/pose_aggregator.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

#include "backend/ign_publisher_system.h"
#include "backend/ign_subscriber_system.h"
#include "backend/load_robot_aggregator.h"
#include "backend/scene_system.h"
#include "backend/system.h"

#include "../include/delphyne/linb-any"

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
  /// A constructor that configures this object to use DrakeLcm, which
  /// encapsulates a _real_ LCM instance.
  AutomotiveSimulator();
  explicit AutomotiveSimulator(
      std::unique_ptr<drake::lcm::DrakeLcmInterface> lcm);
  ~AutomotiveSimulator();

  /// Returns the LCM object used by this AutomotiveSimulator.
  drake::lcm::DrakeLcmInterface* get_lcm();

  /// Returns the DiagramBuilder.
  /// @pre Start() has NOT been called.
  drake::systems::DiagramBuilder<T>* get_builder();

  /// Return the scene.
  std::unique_ptr<ignition::msgs::Scene> GetScene();

  /// Adds a Vehicle to this simulation from a loadable module.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @param plugin_library_name The name of the plugin library, without the
  /// "lib" prefix or ".so" suffix.
  ///
  /// @param parameters Parameters to be passed to the loadable module
  /// "configure" method.
  ///
  /// @param name The agent's name, which must be unique among all agents.
  /// Otherwise a std::runtime_error will be thrown.
  ///
  /// @param initial_state The vehicle's initial state.
  ///
  /// @return The ID of the agent that was just added to the simulation, or -1
  /// on error.
  int AddLoadableAgent(
      const std::string& plugin_library_name,
      const std::map<std::string, linb::any>& parameters,
      const std::string& name,
      std::unique_ptr<drake::systems::BasicVector<T>> initial_state);

  /// Specify the exact plugin name if there should be more than one plugin
  /// in the plugin library.
  int AddLoadableAgent(
      const std::string& plugin_library_name, const std::string& plugin_name,
      const std::map<std::string, linb::any>& parameters,
      const std::string& name,
      std::unique_ptr<drake::systems::BasicVector<T>> initial_state);

  /// Sets the RoadGeometry for this simulation.
  ///
  /// @pre Start() has NOT been called.
  const drake::maliput::api::RoadGeometry* SetRoadGeometry(
      std::unique_ptr<const drake::maliput::api::RoadGeometry> road);

  /// Finds and returns a pointer to a lane with the specified name. This method
  /// throws a std::runtime_error if no such lane exists.
  ///
  /// @pre SetRoadGeometry() was called.
  ///
  const drake::maliput::api::Lane* FindLane(const std::string& name) const;

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

  int allocate_vehicle_number();

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

  void InitializeSimpleCars();
  void InitializeLoadableAgents();

  // For both building and simulation.
  std::unique_ptr<drake::lcm::DrakeLcmInterface> lcm_{};
  std::unique_ptr<const drake::maliput::api::RoadGeometry> road_{};

  // === Start for building. ===
  std::unique_ptr<RigidBodyTree<T>> tree_{std::make_unique<RigidBodyTree<T>>()};

  std::unique_ptr<drake::systems::DiagramBuilder<T>> builder_{
      std::make_unique<drake::systems::DiagramBuilder<T>>()};

  // Holds the desired initial states of each SimpleCar. It is used to
  // initialize the simulation's diagram's state.
  std::map<const drake::systems::System<T>*,
           drake::automotive::SimpleCarState<T>>
      simple_car_initial_states_;

  // Holds the desired initial states of each loadable agent. It is used to
  // initialize the simulation's diagram's state.
  std::map<drake::systems::System<T>*,
           std::unique_ptr<drake::systems::BasicVector<T>>>
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

  int next_vehicle_number_{0};

  // Maps an agent id to a pointer to the system that implements the agent.
  std::map<int, drake::systems::System<T>*> agents_;

  // For simulation.
  std::unique_ptr<drake::systems::Diagram<T>> diagram_{};
  std::unique_ptr<drake::systems::Simulator<T>> simulator_{};
};

}  // namespace delphyne
