// Copyright 2018 Toyota Research Institute
#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

#include "opendrive/OdrManager.hh"

#include "malidrive/backend/lane_direction.h"
#include "malidrive/backend/step_strategy.h"
#include "malidrive/backend/driving_command.h"
#include "examples/kinematic_car_model.h"
#include "examples/kinematic_car_state.h"
#include "examples/opendriver_controller_state.h"
#include "examples/pure_pursuit.h"

namespace delphyne {

/// Holds static parameters of the OpenDriverController.
struct OpenDriverControllerParams {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(OpenDriverControllerParams)

  OpenDriverControllerParams() = delete;

  /// Constructs an OpenDriverControllerParams.
  ///
  /// @param _s_lookahead is the look ahead distance along the lane s-coordinate
  /// based on the closest position on the path to the vehicle. It must be
  /// positive.
  /// @param _default_acceleration is the default acceleration to apply to a
  /// backend::DrivingCommand by OpenDriverController. It must not be negative.
  /// @param _car_properties are the properties of a KinematicCarModel that
  /// the driver must know to drive it.
  OpenDriverControllerParams(
      double _s_lookahead,
      double _default_acceleration,
      const KinematicCarModel::CarProperties& _car_properties) :
        s_lookahead(_s_lookahead), default_acceleration(_default_acceleration),
        car_properties(_car_properties) {
    DRAKE_DEMAND(s_lookahead > 0.);
    DRAKE_DEMAND(default_acceleration >= 0.);
  }

  double s_lookahead{};
  double default_acceleration{};
  KinematicCarModel::CarProperties car_properties;
};

/// Computes a DriveCommand for a KinematicCar.
///
/// This class goes in hand with a KinematicCar and wraps a PurePursuit
/// algorithm to generate each steering command when driving. It will constantly
/// accelerate the car independently of any road characteristic.
class OpenDriverController {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OpenDriverController)

  OpenDriverController() = delete;

  /// Creates an OpenDriverController.
  ///
  /// @param parameters are the driver parameters.
  /// @param state is the initial state of the driver.
  /// @param step_strategy is the strategy to follow when the car approaches a
  /// LaneSection end. It must not be nullptr.
  /// @param manager is the OpenDRIVE manager. It must not be nullptr.
  OpenDriverController(const OpenDriverControllerParams& parameters,
                       const OpenDriverControllerState& state,
                       std::unique_ptr<backend::StepStrategy> step_strategy,
                       OpenDrive::OdrManager* manager) :
      parameters_(parameters), state_(state),
      step_strategy_(std::move(step_strategy)), manager_(manager) {
    DRAKE_DEMAND(step_strategy_ != nullptr);
    DRAKE_DEMAND(manager_ != nullptr);
  }

  OpenDriverControllerParams get_parameters() const { return parameters_; }

  OpenDriverControllerState get_state() const { return state_; }

  /// Returns a backend::DrivingCommand that should be sent to
  /// a KinematicCarModel.
  /// `car_state`'s inertial position and orientation are mapped into an
  /// OpenDRIVE TRACK-Frame. First, the last track and lane IDs are used as
  /// hint to map from the INERTIAL-Frame to TRACK-Frame. When the map is not
  /// feasible (due to OpenDRIVE mapping issues), OdrManager::intersectCircle
  /// API is used, taking as radius the distance between the last two
  /// INERTIAL-frame positions (or the default value exploration radius). The
  /// goal would be selected from the obtained list of LaneCoord objects. If
  /// there is not any intersected points, the returned backend::DrivingCommand
  /// would make the car stop.
  backend::DrivingCommand Drive(const KinematicCarState& car_state);

 private:
  // Returns the acceleration command.
  //
  // When `goal_type` is backend::GoalType::kEndpoint, the acceleration command
  // makes the car stop considering the distance between `inertial_pose` to
  // `target_goal` and car's `speed`. Otherwise, default acceleration is
  // returned.
  double EvaluateAcceleration(
      const OpenDrive::Coord& inertial_pose,
      const OpenDrive::Coord& target_goal,
      backend::GoalType goal_type,
      double speed) const;

  const OpenDriverControllerParams parameters_;
  OpenDriverControllerState state_;
  std::unique_ptr<backend::StepStrategy> step_strategy_;
  OpenDrive::OdrManager* manager_{};
};

}  // namespace delphyne
