// Copyright 2018 Toyota Research Institute

#include <map>
#include <string>

#include "backend/translation_systems/drake_simple_car_state_to_ign_simple_car_state.h"

namespace delphyne {
namespace backend {
namespace translation_systems {

DrakeSimpleCarStateToIgnSimpleCarState::DrakeSimpleCarStateToIgnSimpleCarState()
    : DrakeToIgn(drake::automotive::SimpleCarStateIndices::kNumCoordinates) {}

void DrakeSimpleCarStateToIgnSimpleCarState::DoDrakeToIgnTranslation(
    const drake::automotive::SimpleCarState<double>& drake_message,
    ignition::msgs::SimpleCarState* ign_message, int64_t time_ms) const {
  DELPHYNE_DEMAND(ign_message != nullptr);

  LcmTimestampToIgnition(time_ms, ign_message->mutable_time());
  ign_message->set_x(drake_message.x());
  ign_message->set_y(drake_message.y());
  ign_message->set_heading(drake_message.heading());
  ign_message->set_velocity(drake_message.velocity());
}

}  // namespace translation_systems
}  // namespace backend
}  // namespace delphyne
