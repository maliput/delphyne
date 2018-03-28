// Copyright 2017 Toyota Research Institute

#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <drake/automotive/maliput/dragway/road_geometry.h>
#include <drake/automotive/maliput/monolane/loader.h>
#include <drake/automotive/maliput/multilane/loader.h>
#include <drake/automotive/monolane_onramp_merge.h>

#include "backend/automotive_simulator.h"
#include "backend/road_builder.h"

namespace delphyne {
namespace backend {

/// This class is used to build and add different types of maliput roads to a
/// simulator. Linear and angular tolerances can be specified when a builder
/// is created and will be applied to all the roads constructed (otherwise
/// std::numeric_limits<double>::epsilon() is used).
///
/// @tparam T must be a valid Eigen ScalarType.
template <typename T>
class DELPHYNE_BACKEND_VISIBLE RoadBuilder {
 public:
  /// @brief Default constructor.
  ///
  /// @param[in] simulator A pointer to the automotive simulator where the
  /// roads will be added.
  explicit RoadBuilder(AutomotiveSimulator<T>* simulator)
      : simulator_(simulator) {}

  /// @brief Default constructor.
  ///
  /// @param[in] simulator A pointer to the automotive simulator where the
  /// roads will be added.
  ///
  /// @param[in] linear_tolerance The tolerance guaranteed for linear
  /// measurements (positions).
  ///
  /// @param[in] angular_tolerance The tolerance guaranteed for angular
  /// measurements (orientations).
  explicit RoadBuilder(AutomotiveSimulator<T>* simulator,
                       double linear_tolerance, double angular_tolerance)
      : simulator_(simulator),
        linear_tolerance_(linear_tolerance),
        angular_tolerance_(angular_tolerance) {}

  /// @brief Adds a dragway to the underlying simulator.
  ///
  /// @param[in] name The name of the dragway. Will be used as the ID of the
  /// underlying RoadGeometry.
  ///
  /// @param[in] num_lanes The number of lanes.
  ///
  /// @param[in] length The length of the dragway.
  ///
  /// @param[in] lane_width The width of each lane.
  ///
  /// @param[in] shoulder_width The width of the shoulders on each side of the
  /// road.
  ///
  /// @param[in] maximum_height The maximum height above the road surface
  /// modelled by the RoadGeometry.
  const drake::maliput::api::RoadGeometry* AddDragway(
      const std::string& name, int num_lanes, double length, double lane_width,
      double shoulder_width, double maximum_height) {
    auto id = drake::maliput::api::RoadGeometryId(name);
    std::unique_ptr<const drake::maliput::api::RoadGeometry> road_geometry =
        std::make_unique<const drake::maliput::dragway::RoadGeometry>(
            id, num_lanes, length, lane_width, shoulder_width, maximum_height,
            linear_tolerance_, angular_tolerance_);
    return simulator_->SetRoadGeometry(std::move(road_geometry));
  }

  /// @brief Adds a monolane-based on-ramp road network to the underlying
  /// simulator.
  const drake::maliput::api::RoadGeometry* AddOnramp() {
    auto onramp_generator =
        std::make_unique<drake::automotive::MonolaneOnrampMerge>();
    return simulator_->SetRoadGeometry(onramp_generator->BuildOnramp());
  }

  /// @brief Adds a monolane-based road network, loading it from the specified
  /// file path
  const drake::maliput::api::RoadGeometry* AddMonolaneFromFile(
      const std::string& file_path) {
    auto road_geometry = drake::maliput::monolane::LoadFile(file_path);
    return simulator_->SetRoadGeometry(std::move(road_geometry));
  }

  /// @brief Adds a multinale-based road network, loading it from the specified
  /// file path
  void AddMultilaneFromFile(const std::string& file_path) {
    auto road_geometry = drake::maliput::multilane::LoadFile(file_path);
    simulator_->SetRoadGeometry(std::move(road_geometry));
  }

 private:
  // The automotive simulator were the different roads will be added.
  AutomotiveSimulator<T>* simulator_{nullptr};

  // The tolerance guaranteed for linear measurements (positions).
  double linear_tolerance_{std::numeric_limits<double>::epsilon()};

  // The tolerance guaranteed for angular measurements (orientations).
  double angular_tolerance_{std::numeric_limits<double>::epsilon()};
};

}  // namespace backend
}  // namespace delphyne
