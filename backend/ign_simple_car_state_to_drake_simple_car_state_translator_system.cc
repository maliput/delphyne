// Copyright 2018 Toyota Research Institute

#include "backend/ign_simple_car_state_to_drake_simple_car_state_translator_system.h"

namespace delphyne {
namespace backend {

void IgnSimpleCarStateToDrakeSimpleCarStateTranslatorSystem::
    DoIgnToDrakeTranslation(
        const ignition::msgs::SimpleCarState& ign_message,
        drake::automotive::SimpleCarState<double>* drake_message) const {
  DELPHYNE_DEMAND(drake_message != nullptr);

  drake_message->set_x(ign_message.x());
  drake_message->set_y(ign_message.y());
  drake_message->set_heading(ign_message.heading());
  drake_message->set_velocity(ign_message.velocity());
}

}  // namespace backend
}  // namespace delphyne
