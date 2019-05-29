// Copyright 2018 Toyota Research Institute
#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

#include "malidrive/backend/lane_direction.h"
#include "examples/pure_pursuit.h"

namespace delphyne {

/// Holds an OpenDRIVE driver state.
struct OpenDriverControllerState {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(OpenDriverControllerState)

  OpenDriverControllerState() = delete;

  /// Creates a state for an OpenDRIVER driver controller.
  explicit OpenDriverControllerState(backend::PurePursuitOpenDriveGoal _goal) :
      goal(_goal) {}

  backend::PurePursuitOpenDriveGoal goal;
};

}  // namespace delphyne
