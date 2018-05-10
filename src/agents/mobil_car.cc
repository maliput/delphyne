// Copyright 2017 Toyota Research Institute

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

////#include "drake/automotive/calc_smooth_acceleration.h"
////#include "drake/automotive/gen/driving_command.h"
////#include "drake/automotive/gen/driving_command_translator.h"
//#include "drake/automotive/gen/simple_car_params.h"
//#include "drake/automotive/gen/simple_car_state.h"
//#include "drake/automotive/gen/simple_car_state_translator.h"
#include "drake/automotive/idm_controller.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/utility/generate_urdf.h"
#include "drake/automotive/mobil_planner.h"
#include "drake/automotive/pose_selector.h"
#include "drake/automotive/prius_vis.h"
#include "drake/automotive/pure_pursuit_controller.h"
//#include "drake/common/eigen_types.h"
//#include "drake/math/saturate.h"
//#include "drake/multibody/parsers/urdf_parser.h"
//#include "drake/systems/framework/leaf_system.h"
//#include "drake/systems/framework/system_constraint.h"
//#include "drake/systems/lcm/lcm_publisher_system.h"
//#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/multiplexer.h"
//#include "drake/systems/rendering/frame_velocity.h"
//#include "drake/systems/rendering/pose_vector.h"

#include <backend/ign_publisher_system.h>
#include <backend/translation_systems/drake_simple_car_state_to_ign.h>

#include "../../include/delphyne/agent_plugin_base.h"
#include "../../include/delphyne/linb-any"
#include "../systems/simple_car.h"

namespace delphyne {

class MobilCar final : public delphyne::AgentPlugin {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobilCar)

  MobilCar() : simple_car_() { igndbg << "MobilCar constructor" << std::endl; }

  int Configure(const std::string& name, const int& id,
                const std::map<std::string, linb::any>& parameters,
                drake::systems::DiagramBuilder<double>* builder,
                drake::systems::rendering::PoseAggregator<double>* aggregator,
                drake::automotive::CarVisApplicator<double>* car_vis_applicator)
      override {
    igndbg << "MobilCar configure" << std::endl;

    /*********************
     * Basics
     *********************/
    this->set_id(id);
    this->set_name(name);

    /*********************
     * Parse Parameters
     *********************/
    auto road = linb::any_cast<const drake::maliput::api::RoadGeometry*>(
        parameters.at("road"));
    if (road == nullptr) {
      ignerr << "RoadGeometry not valid. Please create a valid road before "
             << "instantiating this module" << std::endl;
      return -1;
    }

    bool initial_with_s = linb::any_cast<bool>(parameters.at("initial_with_s"));

    /*********************
     * Systems
     *********************/
    // driving
    drake::automotive::MobilPlanner<double>* mobil_planner =
        builder->template AddSystem<drake::automotive::MobilPlanner<double>>(
            std::make_unique<drake::automotive::MobilPlanner<double>>(
                *road, initial_with_s,
                drake::automotive::RoadPositionStrategy::kExhaustiveSearch,
                0. /* time period (unused) */));
    mobil_planner->set_name(name + "_mobil_planner");

    drake::automotive::IdmController<double>* idm_controller =
        builder->template AddSystem<drake::automotive::IdmController<double>>(
            std::make_unique<drake::automotive::IdmController<double>>(
                *road, drake::automotive::ScanStrategy::kBranches,
                drake::automotive::RoadPositionStrategy::kExhaustiveSearch,
                0. /* time period (unused) */));
    idm_controller->set_name(name + "_idm_controller");

    drake::automotive::PurePursuitController<double>* pursuit =
        builder->template AddSystem<
            drake::automotive::PurePursuitController<double>>(
            std::make_unique<
                drake::automotive::PurePursuitController<double>>());
    pursuit->set_name(name + "_pure_pursuit_controller");

    simple_car_ =
        builder->template AddSystem<drake::automotive::SimpleCar2<double>>(
            std::make_unique<drake::automotive::SimpleCar2<double>>());
    simple_car_->set_name(name + "_simple_car");

    drake::systems::Multiplexer<double>* mux =
        builder->template AddSystem<drake::systems::Multiplexer<double>>(
            std::make_unique<drake::systems::Multiplexer<double>>(
                drake::automotive::DrivingCommand<double>()));
    mux->set_name(name + "_mux");

    // publishing
    auto car_state_translator =
        builder->template AddSystem<DrakeSimpleCarStateToIgn>();

    const std::string car_state_channel =
        "agents/" + std::to_string(id) + "/state";

    IgnPublisherSystem<ignition::msgs::SimpleCarState>* car_state_publisher =
        builder->template AddSystem<
            IgnPublisherSystem<ignition::msgs::SimpleCarState>>(
            std::make_unique<
                IgnPublisherSystem<ignition::msgs::SimpleCarState>>(
                car_state_channel));

    /*********************
     * Diagram Wiring
     *********************/
    // driving
    builder->Connect(simple_car_->pose_output(),
                     mobil_planner->ego_pose_input());
    builder->Connect(simple_car_->velocity_output(),
                     mobil_planner->ego_velocity_input());
    builder->Connect(idm_controller->acceleration_output(),
                     mobil_planner->ego_acceleration_input());
    builder->Connect(aggregator->get_output_port(0),
                     mobil_planner->traffic_input());

    builder->Connect(simple_car_->pose_output(),
                     idm_controller->ego_pose_input());
    builder->Connect(simple_car_->velocity_output(),
                     idm_controller->ego_velocity_input());
    builder->Connect(aggregator->get_output_port(0),
                     idm_controller->traffic_input());

    builder->Connect(simple_car_->pose_output(), pursuit->ego_pose_input());
    builder->Connect(mobil_planner->lane_output(), pursuit->lane_input());
    // Build DrivingCommand via a mux of two scalar outputs (a BasicVector where
    // row 0 = steering command, row 1 = acceleration command).
    builder->Connect(pursuit->steering_command_output(),
                     mux->get_input_port(0));
    builder->Connect(idm_controller->acceleration_output(),
                     mux->get_input_port(1));
    builder->Connect(mux->get_output_port(0), simple_car_->get_input_port(0));

    // simulator
    // TODO(daniel.stonier): This is a very repeatable pattern for vehicle
    // agents, reuse?
    auto ports = aggregator->AddSinglePoseAndVelocityInput(name, id);
    builder->Connect(simple_car_->pose_output(), ports.pose_descriptor);
    builder->Connect(simple_car_->velocity_output(), ports.velocity_descriptor);
    car_vis_applicator->AddCarVis(
        std::make_unique<drake::automotive::PriusVis<double>>(id, name));

    // publishing
    builder->Connect(simple_car_->state_output(),
                     car_state_translator->get_input_port(0));
    builder->Connect(*car_state_translator, *car_state_publisher);

    return 0;
  }

  int Initialize(drake::systems::Context<double>* context) override {
    igndbg << "LoadableMobilControlledSimpleCar initialize" << std::endl;
    return 0;
  }

  drake::systems::System<double>* get_system() const { return simple_car_; }

 private:
  drake::automotive::SimpleCar2<double>* simple_car_;

  //  drake::optional<bool> DoHasDirectFeedthrough(int, int) const override {
  //    return false;
  //  }
};

class MobilCarFactory final : public delphyne::AgentPluginFactory {
 public:
  std::unique_ptr<delphyne::AgentPluginBase<double>> Create() {
    return std::make_unique<MobilCar>();
  }
};

}  // namespace delphyne

IGN_COMMON_REGISTER_SINGLE_PLUGIN(delphyne::MobilCarFactory,
                                  delphyne::AgentPluginFactory)
