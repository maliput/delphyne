// Copyright 2017 Toyota Research Institute

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

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

#include <drake/automotive/gen/maliput_railcar_params.h>
#include <drake/automotive/lane_direction.h>
#include <drake/automotive/maliput/api/junction.h>
#include <drake/automotive/maliput/api/road_geometry.h>
#include <drake/automotive/maliput/api/segment.h>
#include <drake/automotive/prius_vis.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/rendering/pose_aggregator.h>

#include "systems/maliput_railcar.h"

#include "../../include/delphyne/agent_plugin_base.h"
#include "../../include/delphyne/linb-any"
#include "src/agents/rail_car.h"

namespace delphyne {

class RailCar final : public delphyne::AgentPlugin {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RailCar)

  static constexpr double kLaneEndEpsilon{1e-12};
  static constexpr double kTimeEpsilon{1e-12};
  static constexpr double kDefaultInitialS{0};
  static constexpr double kDefaultInitialSpeed{1};

  RailCar() : params_(nullptr), rail_car_() {
    igndbg << "RailCar constructor" << std::endl;
  }

  int Configure(const std::string& name, int id,
                drake::systems::DiagramBuilder<double>* builder,
                drake::systems::rendering::PoseAggregator<double>* aggregator,
                drake::automotive::CarVisApplicator<double>* car_vis_applicator,
                const drake::maliput::api::RoadGeometry* road,
                std::unique_ptr<AgentPluginParams> parameters) override {
    igndbg << "RailCar configure" << std::endl;

    /*********************
     * Basics
     *********************/
    id_ = id;
    name_ = name;

    /*********************
     * Parse Parameters
     *********************/
    params_ = std::move(downcast_params<RailCarParams>(std::move(parameters)));

    if (params_->get_raw_lane_direction()->lane == nullptr) {
      ignerr << "RailCar::Configure(): "
                "The provided initial lane is nullptr."
             << std::endl;
      return -1;
    }

    if (params_->get_raw_lane_direction()
            ->lane->segment()
            ->junction()
            ->road_geometry() != road) {
      ignerr << "RailCar::Configure(): "
                "The provided initial lane is not within this simulation's "
                "RoadGeometry."
             << std::endl;
      return -1;
    }

    /*********************
     * Instantiate System
     *********************/
    std::unique_ptr<drake::automotive::MaliputRailcar2<double>> system =
        std::make_unique<drake::automotive::MaliputRailcar2<double>>(
            *initial_lane_direction);
    system->set_name(name);
    rail_car_ =
        builder->template AddSystem<drake::automotive::MaliputRailcar2<double>>(
            std::move(system));

    /*********************
     * Diagram Wiring
     *********************/
    // TODO(daniel.stonier): This is a very repeatable pattern for vehicle
    // agents, reuse?
    auto ports = aggregator->AddSinglePoseAndVelocityInput(name, id_);
    builder->Connect(rail_car_->pose_output(), ports.pose_descriptor);
    builder->Connect(rail_car_->velocity_output(), ports.velocity_descriptor);
    lane_state_output_port_index_ =
        this->DeclareAbstractOutputPort(*params_->get_raw_lane_direction(),
                                        &RailCar::CalcLaneOutput)
            .get_index();

    auto ports = aggregator->AddSinglePoseAndVelocityInput(name, id);
    builder->Connect(this->pose_output(), ports.pose_descriptor);
    builder->Connect(this->velocity_output(), ports.velocity_descriptor);
    car_vis_applicator->AddCarVis(
        std::make_unique<drake::automotive::PriusVis<double>>(id_, name));

    return 0;
  }

  int Initialize(drake::systems::Context<double>* context) override {
    igndbg << "RailCar initialize" << std::endl;
    drake::automotive::MaliputRailcarParams<double>& railcar_system_params =
        rail_car_->get_mutable_parameters(context);
    railcar_system_params.set_value(params_->get_value());

    return 0;
  }

  drake::systems::System<double>* get_system() const { return rail_car_; }

 private:
  drake::automotive::MaliputRailcarParams<double>* params_;
  drake::automotive::MaliputRailcar2<double>* rail_car_;
};

class RailCarFactory final : public delphyne::AgentPluginFactory {
 public:
  std::unique_ptr<delphyne::AgentPluginBase<double>> Create() {
    return std::make_unique<RailCar>();
  }
};

}  // namespace delphyne

IGN_COMMON_REGISTER_SINGLE_PLUGIN(delphyne::RailCarFactory,
                                  delphyne::AgentPluginFactory)
