// Copyright 2018 Toyota Research Institute

#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <drake/common/drake_bool.h>
#include <drake/common/dummy_value.h>
#include <drake/common/never_destroyed.h>
#include <drake/common/symbolic.h>
#include <drake/systems/framework/basic_vector.h>

// TODO(jwnimmer-tri) Elevate this to drake/common.
#if __has_cpp_attribute(nodiscard)
#define DRAKE_VECTOR_GEN_NODISCARD [[nodiscard]]  // NOLINT(whitespace/braces)
#else
#define DRAKE_VECTOR_GEN_NODISCARD
#endif

namespace delphyne {

/// Describes the row indices of a SimpleCarParams.
struct SimpleCarParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 6;

  // The index of each individual coordinate.
  static const int kWheelbase = 0;
  static const int kTrack = 1;
  static const int kMaxAbsSteeringAngle = 2;
  static const int kMaxVelocity = 3;
  static const int kMaxAcceleration = 4;
  static const int kVelocityLimitKp = 5;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `SimpleCarParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class SimpleCarParams final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef SimpleCarParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c wheelbase defaults to 2.700 m.
  /// @arg @c track defaults to 1.521 m.
  /// @arg @c max_abs_steering_angle defaults to 0.471 rad.
  /// @arg @c max_velocity defaults to 45.0 m/s.
  /// @arg @c max_acceleration defaults to 4.0 m/s^2.
  /// @arg @c velocity_limit_kp defaults to 10.0 Hz.
  SimpleCarParams() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_wheelbase(2.700);
    this->set_track(1.521);
    this->set_max_abs_steering_angle(0.471);
    this->set_max_velocity(45.0);
    this->set_max_acceleration(4.0);
    this->set_velocity_limit_kp(10.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  SimpleCarParams(const SimpleCarParams& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  SimpleCarParams(SimpleCarParams&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  SimpleCarParams& operator=(const SimpleCarParams& other) {
    this->values() = other.values();
    return *this;
  }
  SimpleCarParams& operator=(SimpleCarParams&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }
  //@}

  /// Create a drake::symbolic::Variable for each element with the known
  /// variable
  /// name.  This is only available for T == drake::symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<
      std::is_same<U, drake::symbolic::Expression>::value>::type
  SetToNamedVariables() {
    this->set_wheelbase(drake::symbolic::Variable("wheelbase"));
    this->set_track(drake::symbolic::Variable("track"));
    this->set_max_abs_steering_angle(
        drake::symbolic::Variable("max_abs_steering_angle"));
    this->set_max_velocity(drake::symbolic::Variable("max_velocity"));
    this->set_max_acceleration(drake::symbolic::Variable("max_acceleration"));
    this->set_velocity_limit_kp(drake::symbolic::Variable("velocity_limit_kp"));
  }

  SimpleCarParams<T>* DoClone() const final { return new SimpleCarParams; }

  /// @name Getters and Setters
  //@{
  /// The distance between the front and rear axles of the vehicle.
  /// @note @c wheelbase is expressed in units of m.
  /// @note @c wheelbase has a limited domain of [0.0, +Inf].
  const T& wheelbase() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kWheelbase);
  }
  /// Setter that matches wheelbase().
  void set_wheelbase(const T& wheelbase) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kWheelbase, wheelbase);
  }
  /// Fluent setter that matches wheelbase().
  /// Returns a copy of `this` with wheelbase set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  SimpleCarParams<T> with_wheelbase(const T& wheelbase) const {
    SimpleCarParams<T> result(*this);
    result.set_wheelbase(wheelbase);
    return result;
  }
  /// The distance between the center of two wheels on the same axle.
  /// @note @c track is expressed in units of m.
  /// @note @c track has a limited domain of [0.0, +Inf].
  const T& track() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kTrack);
  }
  /// Setter that matches track().
  void set_track(const T& track) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kTrack, track);
  }
  /// Fluent setter that matches track().
  /// Returns a copy of `this` with track set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  SimpleCarParams<T> with_track(const T& track) const {
    SimpleCarParams<T> result(*this);
    result.set_track(track);
    return result;
  }
  /// The limit on the driving_command.steering angle input (the desired
  /// steering angle of a virtual center wheel); this element is applied
  /// symmetrically to both left- and right-turn limits.
  /// @note @c max_abs_steering_angle is expressed in units of rad.
  /// @note @c max_abs_steering_angle has a limited domain of [0.0, +Inf].
  const T& max_abs_steering_angle() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMaxAbsSteeringAngle);
  }
  /// Setter that matches max_abs_steering_angle().
  void set_max_abs_steering_angle(const T& max_abs_steering_angle) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMaxAbsSteeringAngle, max_abs_steering_angle);
  }
  /// Fluent setter that matches max_abs_steering_angle().
  /// Returns a copy of `this` with max_abs_steering_angle set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  SimpleCarParams<T> with_max_abs_steering_angle(
      const T& max_abs_steering_angle) const {
    SimpleCarParams<T> result(*this);
    result.set_max_abs_steering_angle(max_abs_steering_angle);
    return result;
  }
  /// The limit on the car's forward speed.
  /// @note @c max_velocity is expressed in units of m/s.
  /// @note @c max_velocity has a limited domain of [0.0, +Inf].
  const T& max_velocity() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMaxVelocity);
  }
  /// Setter that matches max_velocity().
  void set_max_velocity(const T& max_velocity) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMaxVelocity, max_velocity);
  }
  /// Fluent setter that matches max_velocity().
  /// Returns a copy of `this` with max_velocity set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  SimpleCarParams<T> with_max_velocity(const T& max_velocity) const {
    SimpleCarParams<T> result(*this);
    result.set_max_velocity(max_velocity);
    return result;
  }
  /// The limit on the car's acceleration and deceleration.
  /// @note @c max_acceleration is expressed in units of m/s^2.
  /// @note @c max_acceleration has a limited domain of [0.0, +Inf].
  const T& max_acceleration() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMaxAcceleration);
  }
  /// Setter that matches max_acceleration().
  void set_max_acceleration(const T& max_acceleration) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMaxAcceleration, max_acceleration);
  }
  /// Fluent setter that matches max_acceleration().
  /// Returns a copy of `this` with max_acceleration set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  SimpleCarParams<T> with_max_acceleration(const T& max_acceleration) const {
    SimpleCarParams<T> result(*this);
    result.set_max_acceleration(max_acceleration);
    return result;
  }
  /// The smoothing constant for min/max velocity limits.
  /// @note @c velocity_limit_kp is expressed in units of Hz.
  /// @note @c velocity_limit_kp has a limited domain of [0.0, +Inf].
  const T& velocity_limit_kp() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kVelocityLimitKp);
  }
  /// Setter that matches velocity_limit_kp().
  void set_velocity_limit_kp(const T& velocity_limit_kp) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kVelocityLimitKp, velocity_limit_kp);
  }
  /// Fluent setter that matches velocity_limit_kp().
  /// Returns a copy of `this` with velocity_limit_kp set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  SimpleCarParams<T> with_velocity_limit_kp(const T& velocity_limit_kp) const {
    SimpleCarParams<T> result(*this);
    result.set_velocity_limit_kp(velocity_limit_kp);
    return result;
  }
  //@}

  /// See SimpleCarParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return SimpleCarParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(wheelbase());
    result = result && (wheelbase() >= T(0.0));
    result = result && !isnan(track());
    result = result && (track() >= T(0.0));
    result = result && !isnan(max_abs_steering_angle());
    result = result && (max_abs_steering_angle() >= T(0.0));
    result = result && !isnan(max_velocity());
    result = result && (max_velocity() >= T(0.0));
    result = result && !isnan(max_acceleration());
    result = result && (max_acceleration() >= T(0.0));
    result = result && !isnan(velocity_limit_kp());
    result = result && (velocity_limit_kp() >= T(0.0));
    return result;
  }

  // VectorBase override.
  void CalcInequalityConstraint(drake::VectorX<T>* value) const final {
    value->resize(6);
    (*value)[0] = wheelbase() - T(0.0);
    (*value)[1] = track() - T(0.0);
    (*value)[2] = max_abs_steering_angle() - T(0.0);
    (*value)[3] = max_velocity() - T(0.0);
    (*value)[4] = max_acceleration() - T(0.0);
    (*value)[5] = velocity_limit_kp() - T(0.0);
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The SimpleCarParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
