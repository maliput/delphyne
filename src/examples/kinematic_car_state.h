// Copyright 2018 Toyota Research Institute
#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"

namespace delphyne {

/// Holds the kinematic car model state.
struct KinematicCarState {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(KinematicCarState)

  KinematicCarState() :
      inertial_position(), inertial_rotation(0., 0., 0.), velocity(0.) {}

  /// Constructs the kinematic car state.
  /// @param _inertial_position Car's inertial frame position.
  /// @param _inertial_rotation Car's inertial frame rotation.
  /// @param _velocity Car's scalar velocity.
  KinematicCarState(const drake::Vector3<double>& _inertial_position,
                    const drake::math::RollPitchYaw<double>& _inertial_rotation,
                    double _velocity) :
      inertial_position(_inertial_position),
      inertial_rotation(_inertial_rotation),
      velocity(_velocity) {}

  drake::Vector3<double> inertial_position;
  drake::math::RollPitchYaw<double> inertial_rotation;
  double velocity{};
};

}  // namespace delphyne
