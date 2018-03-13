// Copyright 2018 Toyota Research Institute

#include <map>
#include <string>

#include "backend/lcm_simple_car_state_to_ign_simple_car_state_translator_system.h"

namespace delphyne {
namespace backend {

LcmSimpleCarStateToIgnSimpleCarStateTranslatorSystem::
    LcmSimpleCarStateToIgnSimpleCarStateTranslatorSystem() {
  InitPorts();
}

void LcmSimpleCarStateToIgnSimpleCarStateTranslatorSystem::
    DoLcmToIgnTranslation(
        const drake::automotive::SimpleCarState<double>& lcm_message,
        ignition::msgs::SimpleCarState* ign_message) const {
  DELPHYNE_DEMAND(ign_message != nullptr);

  auto time_ms = static_cast<int64_t>(translation_context->get_time()) * 1000;

  LcmTimestampToIgnition(time_ms, ign_message->mutable_time());
  ign_message->set_x(lcm_message.x());
  ign_message->set_y(lcm_message.y());
  ign_message->set_heading(lcm_message.heading());
  ign_message->set_velocity(lcm_message.velocity());
}

int LcmSimpleCarStateToIgnSimpleCarStateTranslatorSystem::GetVectorSize()
    const {
  return drake::automotive::SimpleCarStateIndices::kNumCoordinates;
}

}  // namespace backend
}  // namespace delphyne
