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

/*****************************************************************************
** Includes
*****************************************************************************/

#include "delphyne/roads/road_builder.h"

#include <map>

#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>
#include <maliput/common/assertion_error.h>
#include <maliput/math/vector.h>
#include <maliput/plugin/create_road_network.h>

#include "delphyne/macros.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {
namespace roads {

namespace api = maliput::api;

/*****************************************************************************
** Implementation
*****************************************************************************/

std::unique_ptr<maliput::api::RoadNetwork> CreateDragway(const std::string& name, int num_lanes, double length,
                                                         double lane_width, double shoulder_width,
                                                         double maximum_height, double linear_tolerance,
                                                         double angular_tolerance) {
  const std::map<std::string, std::string> parameters{
      {"road_geometry_id", name},
      {"linear_tolerance", std::to_string(linear_tolerance)},
      {"angular_tolerance", std::to_string(angular_tolerance)},
      {"num_lanes", std::to_string(num_lanes)},
      {"length", std::to_string(length)},
      {"lane_width", std::to_string(lane_width)},
      {"shoulder_width", std::to_string(shoulder_width)},
      {"maximum_height", std::to_string(maximum_height)},
      {"inertial_to_backend_frame_translation", "{0., 0., 0.}"},
  };
  return maliput::plugin::CreateRoadNetwork("maliput_dragway", parameters);
}

std::unique_ptr<maliput::api::RoadNetwork> CreateMultilaneFromFile(const std::string& file_path) {
  const std::map<std::string, std::string> parameters{
      {"yaml_file", file_path},
  };
  return maliput::plugin::CreateRoadNetwork("maliput_multilane", parameters);
}

std::unique_ptr<maliput::api::RoadNetwork> CreateMultilaneFromDescription(const std::string& yaml_description) {
  const std::map<std::string, std::string> parameters{
      {"yaml_description", yaml_description},
  };
  return maliput::plugin::CreateRoadNetwork("maliput_multilane", parameters);
}

std::unique_ptr<maliput::api::RoadNetwork> CreateOnRamp() {
  return maliput::plugin::CreateRoadNetwork("maliput_multilane_on_ramp", {});
}

std::unique_ptr<maliput::api::RoadNetwork> CreateMalidriveFromXodr(const std::string& name,
                                                                   const std::string& file_path,
                                                                   double linear_tolerance, double angular_tolerance) {
  return CreateMalidriveRoadNetworkFromXodr(
      name, file_path, {/*rule_registry_file_path*/}, {/*road_rulebook_file_path*/}, {/*traffic_light_book_path*/},
      {/*phase_ring_path*/}, {/*intersection_book_path*/}, linear_tolerance, angular_tolerance);
}

std::unique_ptr<maliput::api::RoadNetwork> CreateMalidriveRoadNetworkFromXodr(
    const std::string& name, const std::string& file_path, const std::string& rule_registry_file_path,
    const std::string& road_rulebook_file_path, const std::string& traffic_light_book_path,
    const std::string& phase_ring_path, const std::string& intersection_book_path, double linear_tolerance,
    double angular_tolerance) {
  static constexpr double kScaleLength{1.};
  std::map<std::string, std::string> road_network_configuration{
      {"road_geometry_id", name},
      {"opendrive_file", file_path},
      {"linear_tolerance", std::to_string(linear_tolerance)},
      {"angular_tolerance", std::to_string(angular_tolerance)},
      {"scale_length", std::to_string(kScaleLength)},
  };

  if (!rule_registry_file_path.empty()) {
    road_network_configuration.emplace("rule_registry", rule_registry_file_path);
  }
  if (!road_rulebook_file_path.empty()) {
    road_network_configuration.emplace("road_rule_book", road_rulebook_file_path);
  }
  if (!traffic_light_book_path.empty()) {
    road_network_configuration.emplace("traffic_light_book", traffic_light_book_path);
  }
  if (!phase_ring_path.empty()) {
    road_network_configuration.emplace("phase_ring_book", phase_ring_path);
  }
  if (!phase_ring_path.empty()) {
    road_network_configuration.emplace("intersection_book", intersection_book_path);
  }

  return maliput::plugin::CreateRoadNetwork("maliput_malidrive", road_network_configuration);
}

std::unique_ptr<maliput::api::RoadNetwork> CreateMaliputOSMRoadNetwork(
    const std::string& name, const std::string& file_path, const std::string& origin,
    const std::string& rule_registry_file_path, const std::string& road_rulebook_file_path,
    const std::string& traffic_light_book_path, const std::string& phase_ring_path,
    const std::string& intersection_book_path, double linear_tolerance, double angular_tolerance) {
  static constexpr double kScaleLength{1.};
  std::map<std::string, std::string> config{
      {"road_geometry_id", name},
      {"osm_file", file_path},
      {"linear_tolerance", std::to_string(linear_tolerance)},
      {"angular_tolerance", std::to_string(angular_tolerance)},
      {"scale_length", std::to_string(kScaleLength)},
  };

  if (!origin.empty()) {
    config.emplace("origin", origin);
  }
  if (!rule_registry_file_path.empty()) {
    config.emplace("rule_registry", rule_registry_file_path);
  }
  if (!road_rulebook_file_path.empty()) {
    config.emplace("road_rule_book", road_rulebook_file_path);
  }
  if (!traffic_light_book_path.empty()) {
    config.emplace("traffic_light_book", traffic_light_book_path);
  }
  if (!phase_ring_path.empty()) {
    config.emplace("phase_ring_book", phase_ring_path);
  }
  if (!phase_ring_path.empty()) {
    config.emplace("intersection_book", intersection_book_path);
  }

  return maliput::plugin::CreateRoadNetwork("maliput_osm", config);
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

}  // namespace roads
}  // namespace delphyne
