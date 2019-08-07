// Copyright 2018 Toyota Research Institute

#include "backend/load_robot_aggregator.h"

#include <vector>

#include "delphyne/macros.h"

namespace delphyne {

LoadRobotAggregator::LoadRobotAggregator(const std::vector<drake::lcmt_viewer_load_robot>& load_robot_messages)
    : load_robot_messages_(load_robot_messages) {
  DeclareAbstractOutputPort(&LoadRobotAggregator::CalcAggregatedLoadRobot);
}

void LoadRobotAggregator::CalcAggregatedLoadRobot(const drake::systems::Context<double>& context,
                                                  drake::lcmt_viewer_load_robot* load_robot_message) const {
  DELPHYNE_VALIDATE(load_robot_message != nullptr, std::invalid_argument,
                    "Load robot message pointer must not be null");

  // Clears state from the previous call.
  // @see DeclareAbstractOutputPort
  load_robot_message->link.clear();

  for (const auto& message : load_robot_messages_) {
    for (const drake::lcmt_viewer_link_data& link : message.link) {
      load_robot_message->link.push_back(link);
    }
  }

  load_robot_message->num_links = load_robot_message->link.size();
}

}  // namespace delphyne
