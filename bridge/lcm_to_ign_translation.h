// Copyright 2017 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef DELPHYNE_BRIDGE_LCMTOIGNTRANSLATION_HH_
#define DELPHYNE_BRIDGE_LCMTOIGNTRANSLATION_HH_

#include <ignition/msgs.hh>
#include <drake/lcmt_simple_car_state_t.hpp>
#include <drake/lcmt_viewer_command.hpp>
#include <drake/lcmt_viewer_draw.hpp>
#include <drake/lcmt_viewer_geometry_data.hpp>
#include <drake/lcmt_viewer_load_robot.hpp>
#include <robotlocomotion/viewer2_comms_t.hpp>

#include <protobuf/simple_car_state.pb.h>
#include <protobuf/viewer2_comms.pb.h>
#include <protobuf/viewer_command.pb.h>

#include "bridge/translate_exception.h"

namespace delphyne {
namespace bridge {

/// \brief Translate a car status message from LCM to ignition
/// \param[in]  lcmData An LCM message containing the the car state
/// \param[out] ignData The resuting ignition message with the car state
void lcmToIgn(const drake::lcmt_simple_car_state_t& lcmData,
              ignition::msgs::SimpleCarState* ignData);

/// \brief Translate a viewer status message from LCM to ignition
/// \param[in]  lcmData An LCM message containing the viewer status
/// \param[out] ignData The resulting ignition message with the translation
void lcmToIgn(const drake::lcmt_viewer_command& lcmData,
              ignition::msgs::ViewerCommand* ignData);

/// \brief Translate a viewer2_comms message from LCM to ignition
/// \param[in]  lcmViewer2Data An LCM message containing information about
/// Director
/// \param[out] ignViewer2Data The resulting ignition message with the
/// translation
void lcmToIgn(const robotlocomotion::viewer2_comms_t& lcmViewer2Data,
              ignition::msgs::Viewer2Comms* ignViewer2Data);

/// \brief Translate a whole robot model definition from LCM to a vector
/// of ignition model
/// \param[in]  robotData  An LCM message containing the robot(s) data
/// \param[out] robotModels The resulting ignition message with the vector of
///             robot models
void lcmToIgn(const drake::lcmt_viewer_load_robot& robotData,
              ignition::msgs::Model_V* robotModels);

/// \brief Translate a whole robot model definition from LCM to ignition
/// \param[in]  robotData  An LCM message containing the robot data
/// \param[out] robotModel The resulting ignition message with the robot model
void lcmToIgn(const drake::lcmt_viewer_load_robot& robotData,
              ignition::msgs::Model* robotModel);

/// \brief Translate a link definition from LCM to ignition
/// \param[in]  linkData  An LCM message containing the link data
/// \param[out] linkModel The resulting ignition message with the link model
void lcmToIgn(const drake::lcmt_viewer_link_data& linkData,
              ignition::msgs::Link* linkModel);

/// \brief Translate a geometry and visual definition from LCM to ignition
/// \param[in]  geometryData  An LCM message containing the geometry data
/// \param[out] visualModel The resulting ignition message including the
/// geometry and visual properties
void lcmToIgn(const drake::lcmt_viewer_geometry_data& geometryData,
              ignition::msgs::Visual* visualModel);

/// \brief Translate a geometry definition from LCM to ignition
/// \param[in]  geometryData  An LCM message containing the geometry data
/// \param[out] geometryModel The resulting ignition message including the
/// geometry model
void lcmToIgn(const drake::lcmt_viewer_geometry_data& geometryData,
              ignition::msgs::Geometry* geometryModel);

/// \brief Translate a position definition from LCM to ignition
/// \param[in]  positionData  An LCM message containing the position data
/// \param[out] positionModel The resulting ignition message with the position
/// model
void lcmToIgn(const float positionData[3],
              ignition::msgs::Vector3d* positionModel);

/// \brief Translate an orientation definition from LCM to ignition
/// \param[in]  quaternionData  An LCM message containing the orientation data
/// \param[out] quaternionModel The resulting ignition message with the
/// orientation model
void lcmToIgn(const float quaternionData[4],
              ignition::msgs::Quaternion* quaternionModel);

/// \brief Translate an color definition from LCM to ignition
/// \param[in]  colorData  An LCM message containing the color data
/// \param[out] colorModel The resulting ignition message with the color model
void lcmToIgn(const float colorData[4], ignition::msgs::Color* colorModel);

/// \brief Translate a list of robot poses from LCM to an updated ignition
/// model
/// \param[in]  robotDrawData  An LCM message containing the robot poses
/// \param[out] robotModels The resulting ignition message with the poses vector
void lcmToIgn(const drake::lcmt_viewer_draw& robotDrawData,
              ignition::msgs::Model_V* robotModels);

}  // namespace bridge
}  // namespace delphyne

#endif
