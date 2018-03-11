// Copyright 2017 Toyota Research Institute

#pragma once

namespace ignition {
namespace msgs {
class AutomotiveDrivingCommand;
class Model_V;
}
}

namespace drake {
class lcmt_driving_command_t;
class lcmt_viewer_draw;
}

namespace delphyne {
namespace backend {

/// \brief Translate a driving command from LCM to ignition
/// \param[in]  ign_driving_command An ignition message containing the driving
/// command
/// \param[out] lcm_driving_command The resulting LCM command
void ignToLcm(
    const ignition::msgs::AutomotiveDrivingCommand& ign_driving_command,
    drake::lcmt_driving_command_t* lcm_driving_command);

/// \brief Translate a model vector ignition message to an LCM view draw message
/// \param[in]  robot_models An ignition message containing the model vector
/// \param[out] robot_draw_data The resulting LCM view draw message
void ignToLcm(const ignition::msgs::Model_V& robot_models,
              drake::lcmt_viewer_draw* robot_draw_data);

}  // namespace backend
}  // namespace delphyne
