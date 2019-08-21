// Copyright 2017 Toyota Research Institute
//
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
std::unique_ptr<const maliput::api::RoadGeometry> CreateDragway(
    const std::string& name, int num_lanes, double length, double lane_width, double shoulder_width,
    double maximum_height, double linear_tolerance = std::numeric_limits<double>::epsilon(),
    double angular_tolerance = std::numeric_limits<double>::epsilon());

/// @brief Create a multilane from yaml source.
///
/// @param[in] file_path A string pointing to the file to be loaded.
std::unique_ptr<const maliput::api::RoadGeometry> CreateMultilaneFromFile(const std::string& file_path);

/// @brief Create a malidrive from xodr source.
///
/// @param[in] name A name for the road geometry to be created.
/// @param[in] file_path A string pointing to the file to be loaded.
std::unique_ptr<const maliput::api::RoadNetwork> CreateMalidriveFromFile(
    const std::string& name,
    const std::string& file_path,
    const std::string& road_rulebook_file_path = std::string(),
    const std::string& traffic_light_book_path = std::string(),
    const std::string& phase_ring_path = std::string());

/// @brief Create a multilane on-ramp.
std::unique_ptr<const maliput::api::RoadGeometry> CreateOnRamp();

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace roads
}  // namespace delphyne
