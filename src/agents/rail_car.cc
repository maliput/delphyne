/**
 * @file src/agents/rail_car
 *
 * Copyright 2017 Toyota Research Institute
 */
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "agents/rail_car.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <experimental/optional>

#include <drake/automotive/gen/maliput_railcar_params.h>
#include <drake/automotive/gen/simple_car_state_translator.h>
#include <drake/automotive/lane_direction.h>
#include <drake/automotive/maliput/api/junction.h>
#include <drake/automotive/maliput/api/lane.h>
#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/automotive/maliput/api/segment.h>
#include <drake/automotive/prius_vis.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/rendering/pose_aggregator.h>

#include "backend/ign_publisher_system.h"
#include "backend/ign_subscriber_system.h"
#include "backend/translation_systems/drake_simple_car_state_to_ign.h"
#include "backend/translation_systems/ign_driving_command_to_drake.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace delphyne {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

RailCar::RailCar(const std::string& name, const drake::maliput::api::Lane& lane,
                 const bool& direction_of_travel,
                 const double& position,  // s
                 const double& speed, const double& nominal_speed)
    : delphyne::Agent(name),
      initial_parameters_(lane, direction_of_travel, position, speed,
                          nominal_speed),
      rail_car_context_state_(),
      rail_car_context_parameters_(),
      rail_car_system_(nullptr) {
  igndbg << "RailCar constructor" << std::endl;
}

int RailCar::Configure(
    const int& id, const drake::maliput::api::RoadGeometry& road_geometry,
    drake::systems::DiagramBuilder<double>* builder,
    drake::systems::rendering::PoseAggregator<double>* aggregator,
    drake::automotive::CarVisApplicator<double>* car_vis_applicator) {
  igndbg << "RailCar configure" << std::endl;

  /*********************
   * Basics
   *********************/
  id_ = id;

  /*********************
   * Checks
   *********************/
  if (!maliput::FindLane(initial_parameters_.lane.id(), road_geometry)) {
    ignerr << "RailCar::Configure(): "
              "The provided initial lane is not within this simulation's "
              "RoadGeometry."
           << std::endl;
    return -1;
  }

  /*********************
   * Instantiate System
   *********************/
  // TODO(daniel.stonier): LaneDirection is discombabulating, it is more than
  // just direction and not even expressive of the lane's direction. Rather
  // it is intended to be used by the car (not it is not maliput api) to
  // indicate where in the road network it is and whether it is desired to
  // travel against the flow the lane's nominal direction (traffic flow).
  // Probably preferable to not use this at all and specify things separately.
  //   Refactor back in drake to not use at all
  std::unique_ptr<RailCarSystem> system =
      std::make_unique<RailCarSystem>(drake::automotive::LaneDirection(
          &(initial_parameters_.lane),
          initial_parameters_.direction_of_travel));
  system->set_name(name_);
  rail_car_system_ =
      builder->template AddSystem<RailCarSystem>(std::move(system));

  /*********************
   * Diagram Wiring
   *********************/
  // TODO(daniel.stonier): This is a very repeatable pattern for vehicle
  // agents, reuse?
  auto ports = aggregator->AddSinglePoseAndVelocityInput(name_, id);
  builder->Connect(rail_car_system_->pose_output(), ports.pose_descriptor);
  builder->Connect(rail_car_system_->velocity_output(),
                   ports.velocity_descriptor);
  car_vis_applicator->AddCarVis(
      std::make_unique<drake::automotive::PriusVis<double>>(id_, name_));

  /*********************
   * Other
   *********************/
  auto agent_state_translator =
      builder->template AddSystem<DrakeSimpleCarStateToIgn>();

  const std::string car_state_channel =
      "agents/" + std::to_string(id) + "/state";
  auto car_state_publisher = builder->template AddSystem<
      IgnPublisherSystem<ignition::msgs::SimpleCarState>>(car_state_channel);

  // Drake car states are translated to ignition.
  builder->Connect(rail_car_->simple_car_state_output(),
                   agent_state_translator->get_input_port(0));

  // And then the translated ignition car state is published.
  builder->Connect(*agent_state_translator, *car_state_publisher);

  return 0;
}

int RailCar::Initialize(drake::systems::Context<double>* system_context) {
  // state that will be updated by the simulation

  /********************
   * State
   *******************/
  // TODO(daniel.stonier) shift this to the initialize section?
  rail_car_context_state_ = std::make_unique<RailCarContextState>();
  // TODO(daniel.stonier) check for 'in-lane' bounds?
  rail_car_context_state_->set_s(initial_parameters_.position);
  rail_car_context_state_->set_speed(initial_parameters_.speed);
  rail_car_context_parameters_ = std::make_unique<RailCarContextParameters>();
  rail_car_context_parameters_->set_r(0.0);
  // TODO(daniel.stonier) check or clamp to lane height?
  rail_car_context_parameters_->set_h(0.0);
  rail_car_context_parameters_->set_max_speed(
      initial_parameters_.nominal_speed);
  // rail_car_context_parameters->set_velocity_limit_kp() // just use the
  // default for now

  // initialize both the RailCarState (see addLoadableAgent) and the
  // MaliputRailCarParams?

  // RailCarState part
  drake::systems::VectorBase<double>& context_state =
      system_context->get_mutable_continuous_state().get_mutable_vector();
  drake::systems::BasicVector<double>* const state =
      dynamic_cast<drake::systems::BasicVector<double>*>(&context_state);
  DELPHYNE_ASSERT(state);
  state->set_value(rail_car_context_state_->get_value());

  // MaliputRailCarParams part
  RailCarContextParameters& context_parameters =
      rail_car_system_->get_mutable_parameters(system_context);
  context_parameters.set_value(rail_car_context_parameters_->get_value());

  return 0;
}

drake::systems::System<double>* RailCar::get_system() const {
  return rail_car_system_;
}

}  // namespace delphyne
