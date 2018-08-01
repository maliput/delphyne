// Copyright 2018 Toyota Research Institute

#include "translations/ign_agent_state_to_drake.h"

namespace delphyne {

void IgnAgentStateToDrake::DoIgnToDrakeTranslation(
    const ignition::msgs::AgentState& ign_message,
    drake::automotive::SimpleCarState<double>* drake_message) const {
  DELPHYNE_VALIDATE(drake_message != nullptr, std::invalid_argument,
                    "Drake message pointer must not be null");

  drake_message->set_x(ign_message.position().x());
  drake_message->set_y(ign_message.position().y());
  drake_message->set_heading(ign_message.orientation().yaw());

  const auto& vel = ign_message.linear_velocity();
  drake_message->set_velocity(sqrt((vel.x()*vel.x() + vel.y()*vel.y())));
}

}  // namespace delphyne
