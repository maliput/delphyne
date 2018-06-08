/**
 * @file src/agents/mobil_car.cc
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "agents/mobil_car.h"

#include <memory>
#include <string>
#include <utility>

#include <drake/automotive/idm_controller.h>
#include <drake/automotive/mobil_planner.h>
#include <drake/automotive/prius_vis.h>
#include <drake/automotive/pure_pursuit_controller.h>
#include <drake/common/eigen_types.h>
#include <drake/geometry/geometry_frame.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/geometry/geometry_instance.h>
#include <drake/geometry/shape_specification.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

// public headers
#include "delphyne/macros.h"

// private headers
#include "agents/helpers/geometry_wiring.h"
#include "backend/ign_publisher_system.h"
#include "systems/simple_car.h"
#include "translations/drake_simple_car_state_to_ign.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

MobilCar::MobilCar(const std::string& name, bool direction_of_travel, double x,
                   double y, double heading, double speed)
    : delphyne::Agent(name),
      initial_parameters_(direction_of_travel, x, y, heading, speed) {
  igndbg << "MobilCar constructor" << std::endl;
}

int MobilCar::Configure(
    int id, const drake::maliput::api::RoadGeometry* road_geometry,
    drake::systems::DiagramBuilder<double>* builder,
    drake::geometry::SceneGraph<double>* scene_graph,
    drake::systems::rendering::PoseAggregator<double>* aggregator,
    drake::automotive::CarVisApplicator<double>* car_vis_applicator) {
  DELPHYNE_DEMAND(builder != nullptr);
  DELPHYNE_DEMAND(scene_graph != nullptr);
  DELPHYNE_DEMAND(aggregator != nullptr);
  DELPHYNE_DEMAND(car_vis_applicator != nullptr);
  igndbg << "MobilCar configure" << std::endl;

  /*********************
   * Basics
   *********************/
  id_ = id;

  /*********************
   * Checks
   *********************/
  if (!road_geometry) {
    ignerr << "Rail cars need a road geometry to drive on, make sure "
           << "the simulation is configured with one." << std::endl;
    return -1;
  }

  /******************************************
   * Initial Context Variables
   ******************************************/
  typedef drake::automotive::SimpleCarState<double> ContextContinuousState;
  typedef drake::automotive::SimpleCarParams<double> ContextNumericParameters;
  ContextContinuousState context_continuous_state;
  context_continuous_state.set_x(initial_parameters_.x);
  context_continuous_state.set_y(initial_parameters_.y);
  context_continuous_state.set_heading(initial_parameters_.heading);
  context_continuous_state.set_velocity(initial_parameters_.speed);
  ContextNumericParameters context_numeric_parameters;

  /*********************
   * Instantiate Systems
   *********************/
  // driving
  drake::automotive::MobilPlanner<double>* mobil_planner =
      builder->template AddSystem<drake::automotive::MobilPlanner<double>>(
          std::make_unique<drake::automotive::MobilPlanner<double>>(
              *road_geometry, initial_parameters_.direction_of_travel,
              drake::automotive::RoadPositionStrategy::kExhaustiveSearch,
              0. /* time period (unused) */));
  mobil_planner->set_name(name_ + "_mobil_planner");

  drake::automotive::IdmController<double>* idm_controller =
      builder->template AddSystem<drake::automotive::IdmController<double>>(
          std::make_unique<drake::automotive::IdmController<double>>(
              *road_geometry, drake::automotive::ScanStrategy::kBranches,
              drake::automotive::RoadPositionStrategy::kExhaustiveSearch,
              0. /* time period (unused) */));
  idm_controller->set_name(name_ + "_idm_controller");

  drake::automotive::PurePursuitController<double>* pursuit =
      builder->template AddSystem<
          drake::automotive::PurePursuitController<double>>(
          std::make_unique<drake::automotive::PurePursuitController<double>>());
  pursuit->set_name(name_ + "_pure_pursuit_controller");

  typedef drake::automotive::SimpleCar2<double> SimpleCarSystem;
  SimpleCarSystem* simple_car_system =
      builder->template AddSystem<SimpleCarSystem>(
          std::make_unique<SimpleCarSystem>(context_continuous_state,
                                            context_numeric_parameters));
  simple_car_system->set_name(name_ + "_simple_car");

  drake::systems::Multiplexer<double>* mux =
      builder->template AddSystem<drake::systems::Multiplexer<double>>(
          std::make_unique<drake::systems::Multiplexer<double>>(
              drake::automotive::DrivingCommand<double>()));
  mux->set_name(name_ + "_mux");

  /*********************
   * State Publisher
   *********************/
  auto agent_state_translator =
      builder->template AddSystem<DrakeSimpleCarStateToIgn>();

  const std::string agent_state_channel =
      "agents/" + std::to_string(id) + "/state";
  typedef IgnPublisherSystem<ignition::msgs::SimpleCarState>
      AgentStatePublisherSystem;
  AgentStatePublisherSystem* agent_state_publisher_system =
      builder->template AddSystem<AgentStatePublisherSystem>(
          std::make_unique<AgentStatePublisherSystem>(agent_state_channel));

  // publishing
  builder->Connect(simple_car_system->state_output(),
                   agent_state_translator->get_input_port(0));
  builder->Connect(*agent_state_translator, *agent_state_publisher_system);

  /*********************
   * Diagram Wiring
   *********************/
  // driving
  builder->Connect(simple_car_system->pose_output(),
                   mobil_planner->ego_pose_input());
  builder->Connect(simple_car_system->velocity_output(),
                   mobil_planner->ego_velocity_input());
  builder->Connect(idm_controller->acceleration_output(),
                   mobil_planner->ego_acceleration_input());
  builder->Connect(aggregator->get_output_port(0),
                   mobil_planner->traffic_input());

  builder->Connect(simple_car_system->pose_output(),
                   idm_controller->ego_pose_input());
  builder->Connect(simple_car_system->velocity_output(),
                   idm_controller->ego_velocity_input());
  builder->Connect(aggregator->get_output_port(0),
                   idm_controller->traffic_input());

  builder->Connect(simple_car_system->pose_output(), pursuit->ego_pose_input());
  builder->Connect(mobil_planner->lane_output(), pursuit->lane_input());
  // Build DrivingCommand via a mux of two scalar outputs (a BasicVector where
  // row 0 = steering command, row 1 = acceleration command).
  builder->Connect(pursuit->steering_command_output(), mux->get_input_port(0));
  builder->Connect(idm_controller->acceleration_output(),
                   mux->get_input_port(1));
  builder->Connect(mux->get_output_port(0),
                   simple_car_system->get_input_port(0));

  /*********************
   * Simulator Wiring
   *********************/
  // TODO(daniel.stonier): This is a very repeatable pattern for vehicle
  // agents, reuse?
  drake::systems::rendering::PoseVelocityInputPortDescriptors<double> ports =
      aggregator->AddSinglePoseAndVelocityInput(name_, id);
  builder->Connect(simple_car_system->pose_output(), ports.pose_descriptor);
  builder->Connect(simple_car_system->velocity_output(),
                   ports.velocity_descriptor);
  car_vis_applicator->AddCarVis(
      std::make_unique<drake::automotive::PriusVis<double>>(id, name_));

  // Computes the initial world to car transform X_WC0.
  const drake::Isometry3<double> X_WC0 =
      drake::Translation3<double>(initial_parameters_.x,
                                  initial_parameters_.y, 0.)
      * drake::AngleAxis<double>(initial_parameters_.heading,
                                 drake::Vector3<double>::UnitZ());

  // Wires up the Prius geometry.
  builder->Connect(simple_car_system->pose_output(), WirePriusGeometry(
      name_, X_WC0, builder, scene_graph, &geometry_ids_));

  return 0;
}

}  // namespace delphyne
