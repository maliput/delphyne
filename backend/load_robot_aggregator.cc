// Copyright 2017 Toyota Research Institute

#include "backend/load_robot_aggregator.h"

#include <functional>
#include <vector>

#include "backend/system.h"

namespace delphyne {
namespace backend {

LoadRobotAggregator::LoadRobotAggregator() {
  DeclareAbstractInputPort();
  DeclareAbstractOutputPort(&LoadRobotAggregator::CalcAggregatedLoadRobot);
}

void LoadRobotAggregator::CalcAggregatedLoadRobot(
    const drake::systems::Context<double>& context,
    drake::lcmt_viewer_load_robot* load_robot_message) const {
  DELPHYNE_DEMAND(load_robot_message != nullptr);

  using LoadRobotGenerator = std::function<drake::lcmt_viewer_load_robot()>;

  const drake::systems::AbstractValue* load_robot_generators_input =
      EvalAbstractInput(context, kPortIndex);
  const auto& load_robot_generators =
      load_robot_generators_input->GetValue<std::vector<LoadRobotGenerator>>();

  // Clears state from the previous call.
  // @see DeclareAbstractOutputPort
  load_robot_message->link.clear();

  for (const auto& load_robot_generator : load_robot_generators) {
    for (const drake::lcmt_viewer_link_data& link :
         load_robot_generator().link) {
      load_robot_message->link.push_back(link);
    }
  }

  load_robot_message->num_links = load_robot_message->link.size();
}

}  // namespace backend
}  // namespace delphyne
