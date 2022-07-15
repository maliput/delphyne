// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2017-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Some/all of the methods here could eventually make it upstream to
// Drake as conveniences or be made redundant should some of the
// methods there gain python bindings traction.

#pragma once

/*****************************************************************************
** Includes
****************************************************************************/

#include <limits>
#include <memory>
#include <string>

#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {
namespace roads {

/*****************************************************************************
** Methods
*****************************************************************************/

/// @brief Creates a maliput::api::RoadNetwork based on an available maliput::plugin::RoadNetworkLoader plugin
/// implementation.
///
/// @param[in] road_network_plugin_name maliput::plugin::RoadnetworkLoader plugin name to be used.
/// @param[in] loader_parameters Parameters to be passed to the maliput::api::RoadNetwork builder.
///
/// @throws maliput::common::assertion_error When `road_network_plugin_name` isn't found.
std::unique_ptr<maliput::api::RoadNetwork> CreateRoadNetwork(
    const std::string& road_network_plugin_name, const std::map<std::string, std::string>& loader_parameters);

/// @brief Creates a dragway.
///
/// @param[in] name The name of the dragway. Will be used as the ID of the
/// underlying RoadGeometry.
///
/// @param[in] num_lanes The number of lanes (m).
///
/// @param[in] length The length of the dragway (m).
///
/// @param[in] lane_width The width of each lane (m).
///
/// @param[in] shoulder_width The width of the shoulders on each side of the
/// road (m).
///
/// @param[in] maximum_height The maximum height above the road surface
/// modelled by the RoadGeometry (m).
///
/// @param[in] linear_tolerance The tolerance guaranteed for linear
/// measurements (m).
///
/// @param[in] angular_tolerance The tolerance guaranteed for angular
/// measurements (m).
std::unique_ptr<maliput::api::RoadNetwork> CreateDragway(
    const std::string& name, int num_lanes, double length, double lane_width, double shoulder_width,
    double maximum_height, double linear_tolerance = std::numeric_limits<double>::epsilon(),
    double angular_tolerance = std::numeric_limits<double>::epsilon());

/// @brief Creates a multilane from yaml source.
///
/// @param[in] file_path A string pointing to the file to be loaded.
std::unique_ptr<maliput::api::RoadNetwork> CreateMultilaneFromFile(const std::string& file_path);

/// @brief Creates a multilane from yaml description.
///
/// @param[in] yaml_description A serialized yaml description to be loaded.
std::unique_ptr<maliput::api::RoadNetwork> CreateMultilaneFromDescription(const std::string& yaml_description);

/// @brief Creates a malidrive from xodr source.
///
/// @param[in] name A name for the road geometry to be created.
/// @param[in] file_path A string pointing to the file to be loaded.
/// @param[in] linear_tolerance The linear RoadGeometry tolerance. Default value is 1e-3m.
/// @param[in] angular_tolerance The angular RoadGeometry tolerance. Default value is 1e-3rad.
/// @return A maliput::api::RoadNetwork.
std::unique_ptr<maliput::api::RoadNetwork> CreateMalidriveFromXodr(const std::string& name,
                                                                   const std::string& file_path,
                                                                   double linear_tolerance = 1e-3,
                                                                   double angular_tolerance = 1e-3);

/// @brief Creates a malidrive from xodr source.
///
/// @param[in] name A name for the road geometry to be created.
/// @param[in] file_path A string pointing to the XODR file to be loaded.
/// @param[in] rule_registry_file_path A string pointing to the RuleRegistry file to be loaded.
/// @param[in] road_rulebook_file_path A string pointing to the Rulebook file to be loaded.
/// @param[in] traffic_light_book_path A string pointing to the TrafficLightBook file to be loaded.
/// @param[in] phase_ring_path A string pointing to the PhaseRingBook file to be loaded.
/// @param[in] intersection_book_path A string pointing to the IntersectionBook file to be loaded.
/// @param[in] linear_tolerance The linear RoadGeometry tolerance. Default value is 1e-3m.
/// @param[in] angular_tolerance The angular RoadGeometry tolerance. Default value is 1e-3rad.
/// @return A maliput::api::RoadNetwork.
std::unique_ptr<maliput::api::RoadNetwork> CreateMalidriveRoadNetworkFromXodr(
    const std::string& name, const std::string& file_path, const std::string& rule_registry_file_path = std::string(),
    const std::string& road_rulebook_file_path = std::string(),
    const std::string& traffic_light_book_path = std::string(), const std::string& phase_ring_path = std::string(),
    const std::string& intersection_book_path = std::string(), double linear_tolerance = 1e-3,
    double angular_tolerance = 1e-3);

/// @brief Creates a multilane on-ramp.
std::unique_ptr<maliput::api::RoadNetwork> CreateOnRamp();

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace roads
}  // namespace delphyne
