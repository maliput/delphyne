// Copyright 2018 Toyota Research Institute

#include "translations/drake_simple_car_state_to_ign.h"

#include <map>
#include <string>

#include "translations/time_conversion.h"

namespace delphyne {

DrakeSimpleCarStateToIgn::DrakeSimpleCarStateToIgn()
    : DrakeToIgn(drake::automotive::SimpleCarStateIndices::kNumCoordinates) {}

void DrakeSimpleCarStateToIgn::DoDrakeToIgnTranslation(
    const drake::automotive::SimpleCarState<double>& drake_message,
    ignition::msgs::AgentState* ign_message, int64_t time_ms) const {
  DELPHYNE_VALIDATE(ign_message != nullptr, std::invalid_argument,
                    "Ignition message pointer must not be null");

  ign_message->mutable_time()->CopyFrom(MillisToIgnitionTime(time_ms));
  ign_message->mutable_position()->set_x(drake_message.x());
  ign_message->mutable_position()->set_y(drake_message.y());
  ign_message->mutable_position()->set_z(0.0);
  ign_message->mutable_orientation()->set_roll(0.0);
  ign_message->mutable_orientation()->set_pitch(0.0);
  ign_message->mutable_orientation()->set_yaw(drake_message.heading());
  //ign_message->set_velocity(drake_message.velocity());
}

}  // namespace delphyne
