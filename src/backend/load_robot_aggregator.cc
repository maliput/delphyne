// Copyright 2018 Toyota Research Institute

#include "backend/load_robot_aggregator.h"

#include <functional>
#include <vector>

#include "delphyne/macros.h"

namespace delphyne {

using LoadRobotGenerator = std::function<drake::lcmt_viewer_load_robot()>;

LoadRobotAggregator::LoadRobotAggregator() {
  DeclareAbstractInputPort(
      drake::systems::kUseDefaultName,
      drake::systems::Value<std::vector<LoadRobotGenerator>>());
  DeclareAbstractOutputPort(&LoadRobotAggregator::CalcAggregatedLoadRobot);
}

void LoadRobotAggregator::CalcAggregatedLoadRobot(
    const drake::systems::Context<double>& context,
    drake::lcmt_viewer_load_robot* load_robot_message) const {
  DELPHYNE_VALIDATE(load_robot_message != nullptr, std::invalid_argument,
                    "Load robot message pointer must not be null");

  const std::vector<LoadRobotGenerator>* load_robot_generators =
      this->template EvalInputValue<std::vector<LoadRobotGenerator>>(
          context, kPortIndex);

  // Clears state from the previous call.
  // @see DeclareAbstractOutputPort
  load_robot_message->link.clear();

  for (const auto& load_robot_generator : *load_robot_generators) {
    for (const drake::lcmt_viewer_link_data& link :
         load_robot_generator().link) {
      load_robot_message->link.push_back(link);
    }
  }

  load_robot_message->num_links = load_robot_message->link.size();
}

}  // namespace delphyne
