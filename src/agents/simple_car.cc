// Copyright 2017 Toyota Research Institute

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/automotive/gen/simple_car_params.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/automotive/gen/simple_car_state_translator.h"
#include "drake/automotive/prius_vis.h"

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

#include <backend/ign_publisher_system.h>
#include <backend/ign_subscriber_system.h>
#include <backend/translation_systems/drake_simple_car_state_to_ign.h>
#include <backend/translation_systems/ign_driving_command_to_drake.h>

#include "systems/simple_car.h"

#include "../../include/delphyne/agent_plugin_base.h"
#include "../../include/delphyne/linb-any"

namespace delphyne {

class SimpleCar final : public delphyne::AgentPlugin {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleCar)

  SimpleCar() : params_(nullptr), simple_car_() {
    igndbg << "SimpleCar constructor" << std::endl;
  }

  int Configure(const std::string& name, const int& id,
                const std::map<std::string, linb::any>& parameters,
                drake::systems::DiagramBuilder<double>* builder,
                drake::systems::rendering::PoseAggregator<double>* aggregator,
                drake::automotive::CarVisApplicator<double>* car_vis_applicator)
      override {
    igndbg << "SimpleCar configure" << std::endl;

    /*********************
     * Basics
     *********************/
    this->set_id(id);
    this->set_name(name);

    /*********************
     * Instantiate System
     *********************/
    std::unique_ptr<drake::automotive::SimpleCar2<double>> system =
        std::make_unique<drake::automotive::SimpleCar2<double>>();
    system->set_name(name);
    simple_car_ =
        builder->template AddSystem<drake::automotive::SimpleCar2<double>>(
            std::move(system));

    /*********************
     * Teleop
     *********************/
    std::string command_channel = "teleop/" + std::to_string(id);
    auto driving_command_subscriber = builder->template AddSystem<
        IgnSubscriberSystem<ignition::msgs::AutomotiveDrivingCommand>>(
        command_channel);

    auto driving_command_translator =
        builder->template AddSystem<IgnDrivingCommandToDrake>();

    // Ignition driving commands received through the subscriber are translated
    // to Drake.
    builder->Connect(*driving_command_subscriber, *driving_command_translator);

    // And then the translated Drake command is sent to the car.
    builder->Connect(*driving_command_translator, *simple_car_);

    /*********************
     * Diagram Wiring
     *********************/
    // TODO(daniel.stonier): This is a very repeatable pattern for vehicle
    // agents, reuse?
    auto ports = aggregator->AddSinglePoseAndVelocityInput(name, id);
    builder->Connect(simple_car_->pose_output(), ports.pose_descriptor);
    builder->Connect(simple_car_->velocity_output(), ports.velocity_descriptor);
    car_vis_applicator->AddCarVis(
        std::make_unique<drake::automotive::PriusVis<double>>(id, name));

    /*********************
     * Other
     *********************/
    auto car_state_translator =
        builder->template AddSystem<DrakeSimpleCarStateToIgn>();

    const std::string car_state_channel =
        "agents/" + std::to_string(id) + "/state";
    auto car_state_publisher = builder->template AddSystem<
        IgnPublisherSystem<ignition::msgs::SimpleCarState>>(car_state_channel);

    // Drake car states are translated to ignition.
    builder->Connect(simple_car_->state_output(),
                     car_state_translator->get_input_port(0));

    // And then the translated ignition car state is published.
    builder->Connect(*car_state_translator, *car_state_publisher);

    return 0;
  }

  int Initialize(drake::systems::Context<double>* context) override {
    igndbg << "SimpleCar initialize" << std::endl;
    return 0;
  }

  drake::systems::System<double>* get_system() const { return simple_car_; }

 private:
  drake::automotive::SimpleCarParams<double>* params_;
  drake::automotive::SimpleCar2<double>* simple_car_;
};

class SimpleCarFactory final : public delphyne::AgentPluginFactory {
 public:
  std::unique_ptr<delphyne::AgentPluginBase<double>> Create() {
    return std::make_unique<SimpleCar>();
  }
};

}  // namespace delphyne

IGN_COMMON_REGISTER_SINGLE_PLUGIN(delphyne::SimpleCarFactory,
                                  delphyne::AgentPluginFactory)
