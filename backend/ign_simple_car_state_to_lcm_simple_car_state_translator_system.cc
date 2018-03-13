// Copyright 2018 Toyota Research Institute

#include "backend/ign_simple_car_state_to_lcm_simple_car_state_translator_system.h"

namespace delphyne {
namespace backend {

IgnSimpleCarStateToLcmSimpleCarStateTranslatorSystem::
    IgnSimpleCarStateToLcmSimpleCarStateTranslatorSystem() {
  InitPorts();
}

void IgnSimpleCarStateToLcmSimpleCarStateTranslatorSystem::
    DoIgnToLcmTranslation(
        const ignition::msgs::SimpleCarState& ign_message,
        drake::automotive::SimpleCarState<double>* lcm_message) const {
  DELPHYNE_DEMAND(lcm_message != nullptr);

  lcm_message->set_x(ign_message.x());
  lcm_message->set_y(ign_message.y());
  lcm_message->set_heading(ign_message.heading());
  lcm_message->set_velocity(ign_message.velocity());
}

}  // namespace backend
}  // namespace delphyne
