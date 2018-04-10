// Copyright 2018 Toyota Research Institute

#pragma once

#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace delphyne {
namespace backend {

/// @brief A system that aggregates LCM viewer load robot messages, creating a
/// new viewer load robot message, with all of the links of the input messages.
///
/// Because the number of messages to aggregate is unknown, this system has a
/// single abstract input port, in which a vector of functions that return an
/// LCM viewer load message is stored. std::function is used instead of plain
/// function pointers to make the system easier to use with lambda functions
/// that capture state.
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
