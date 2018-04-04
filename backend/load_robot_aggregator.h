// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace delphyne {
namespace backend {

class LoadRobotAggregator : public drake::systems::LeafSystem<double> {
 public:
  LoadRobotAggregator();

 private:
  void CalcAggregatedLoadRobot(
      const drake::systems::Context<double>& context,
      drake::lcmt_viewer_load_robot* load_robot_message) const;

  const int kPortIndex = 0;
};

}  // namespace backend
}  // namespace delphyne
