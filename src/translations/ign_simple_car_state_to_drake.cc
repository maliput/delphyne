// Copyright 2018 Toyota Research Institute

#include "translations/ign_simple_car_state_to_drake.h"

namespace delphyne {

void IgnSimpleCarStateToDrake::DoIgnToDrakeTranslation(
    const ignition::msgs::SimpleCarState& ign_message,
    drake::automotive::SimpleCarState<double>* drake_message) const {
  DELPHYNE_VALIDATE(drake_message != nullptr, std::invalid_argument,
                    "Drake message pointer must not be null");

  drake_message->set_x(ign_message.x());
  drake_message->set_y(ign_message.y());
  drake_message->set_heading(ign_message.heading());
  drake_message->set_velocity(ign_message.velocity());
}

}  // namespace delphyne
