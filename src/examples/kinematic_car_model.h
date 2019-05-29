// Copyright 2018 Toyota Research Institute
#pragma once

#include "drake/common/drake_copyable.h"

#include "examples/kinematic_car_state.h"
#include "malidrive/backend/driving_command.h"

namespace delphyne {

/// Kinematic (not dynamic) car model. The system evolves thanks to
/// backend::DrivingCommands applied during time steps.
/// This class definition and implementation has been inspired in
/// `drake::automotive::SimpleCar`.
class KinematicCarModel {
 public:
  /// Holds car kinematic properties (geometric characteristics) and maximum
  /// system ratings.
  struct CarProperties {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CarProperties)

    CarProperties() = default;

    CarProperties(
        double _wheelbase, double _track, double _max_abs_steering_angle,
        double _max_velocity, double _max_acceleration,
        double _velocity_limit_kp) :
        wheelbase(_wheelbase),
        track(_track),
        max_abs_steering_angle(_max_abs_steering_angle),
        max_velocity(_max_velocity),
        max_acceleration(_max_acceleration),
        velocity_limit_kp(_velocity_limit_kp) {
      DRAKE_DEMAND(wheelbase > 0.);
      DRAKE_DEMAND(track > 0.);
      DRAKE_DEMAND(max_abs_steering_angle > 0.);
      DRAKE_DEMAND(max_velocity >= 0.);
      DRAKE_DEMAND(max_acceleration >= 0.);
      DRAKE_DEMAND(velocity_limit_kp >= 0.);
    }

    /// The distance between the front and rear axles of the vehicle.
    /// It is measured in meters.
    double wheelbase{2.7};
    /// The distance between the center of two wheels on the same axle.
    /// It is measured in meters.
    double track{1.521};
    /// The limit on the driving_command.steering angle input (the desired
    /// steering angle of a virtual center wheel); this element is applied
    /// symmetrically to both left- and right-turn limits.
    /// It is measured in radians.
    double max_abs_steering_angle{0.471};
    /// The limit on the car's forward speed.
    /// It is measured in meters per second.
    double max_velocity{45.};
    /// The limit on the car's acceleration and deceleration.
    /// It is measured in meters per squared seconds.
    double max_acceleration{4.};
    /// The smoothing constant for min/max velocity limits.
    /// It is measured in hertz.
    double velocity_limit_kp{10.};
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KinematicCarModel)

  KinematicCarModel() = delete;

  /// Constructs a kinematic car model.
  /// @param properties Geometric properties and system maximum ratings.
  /// @param initial_state Initial car state.
  KinematicCarModel(const CarProperties& properties,
                    const KinematicCarState& initial_state) :
      properties_(properties), state_(initial_state) {}

  /// Returns current car state.
  KinematicCarState get_state() const { return state_; }

  /// Returns car properties.
  CarProperties get_car_properties() const { return properties_; }

  /// Sets and saturates `driving_command`.
  /// `driving_command.steering` will be saturated based on car properties.
  void set_driving_command(const ::backend::DrivingCommand& driving_command);

  /// Steps the system by `time_step` seconds.
  void StepBy(double time_step);

 private:
  CarProperties properties_;
  KinematicCarState state_;
  ::backend::DrivingCommand driving_command_;
};

}  // namespace delphyne
