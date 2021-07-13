// Copyright 2018 Toyota Research Institute

#pragma once

#include <vector>

#include <drake/lcmt_viewer_load_robot.hpp>
#include <drake/systems/framework/leaf_system.h>

namespace delphyne {

/// @brief A system that aggregates LCM viewer load robot messages, creating a
/// new viewer load robot message, with all of the links of the input messages.
class LoadRobotAggregator : public drake::systems::LeafSystem<double> {
 public:
  explicit LoadRobotAggregator(const std::vector<drake::lcmt_viewer_load_robot>& load_robot_messages);

  // The system has a single input port, and a single output port.
  static const int kPortIndex = 0;

 private:
  // Aggregates into @p load_robot_message all lcmt_viewer_load_robot messages
  // returned from the functions stored on the vector on the input port.
  void CalcAggregatedLoadRobot(const drake::systems::Context<double>& context,
                               drake::lcmt_viewer_load_robot* load_robot_message) const;

  std::vector<drake::lcmt_viewer_load_robot> load_robot_messages_;
};

}  // namespace delphyne
