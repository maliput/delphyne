// Copyright 2017 Toyota Research Institute

// This is roughly equivalent to the results of a class generated from
// a drake 'named vector' protobuf file. Given the effort to support named
// vectors outside of drake, it is as yet undecided whether there is sufficient
// value add in Delphyne to follow suit.

#pragma once

/*****************************************************************************
** Includes
*****************************************************************************/

#include <array>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <drake/systems/framework/basic_vector.h>

#include "delphyne/types.h"

// TODO(jwnimmer-tri) Elevate this to drake/common.
#if __has_cpp_attribute(nodiscard)
#define DRAKE_VECTOR_GEN_NODISCARD [[nodiscard]]  // NOLINT(whitespace/braces)
#else
#define DRAKE_VECTOR_GEN_NODISCARD
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/// Specializes BasicVector with specific getters and setters that
/// specify the parameters used in updating a RailFollower system.
///
/// Parameters:
///   r : orthogonal offset from the lane centreline (rail) (m)
///
template <typename T>
class RailFollowerParams final : public drake::systems::BasicVector<T> {
 private:
  /// @name Vector indices.
  //@{
  static constexpr int kR = 0;
  static constexpr int kH = 1;
  static constexpr int kMaxSpeed = 2;
  static constexpr int kVelocityLimitKp = 3;
  //@}

  /// String names for each variable in the vector.
  static const std::vector<std::string> kNames;

  /// Default values for all variables.
  static const std::vector<double> kDefaults;

  /// Docstrings for each variable in the vector.
  static const std::vector<std::string> kDocStrings;

 public:
  /// @brief Initialise the vector with defaults.
  RailFollowerParams() : drake::systems::BasicVector<T>(4) {
    // set defaults
    this->set_r(kDefaults[kR]);
    this->set_h(kDefaults[kH]);
    this->set_max_speed(kDefaults[kMaxSpeed]);
    this->set_velocity_limit_kp(kDefaults[kVelocityLimitKp]);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  RailFollowerParams(const RailFollowerParams& other) : drake::systems::BasicVector<T>(other.values()) {}
  RailFollowerParams(RailFollowerParams&& other) noexcept : drake::systems::BasicVector<T>(std::move(other.values())) {}
  RailFollowerParams& operator=(const RailFollowerParams& other) {
    this->values() = other.values();
    return *this;
  }
  RailFollowerParams& operator=(RailFollowerParams&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }
  //@}

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == delphyne::Symbolic.
  template <typename U = T>
  typename std::enable_if<std::is_same<U, Symbolic>::value>::type SetToNamedVariables() {
    this->set_r(drake::symbolic::Variable(kNames[kR]));
    this->set_h(drake::symbolic::Variable(kNames[kH]));
    this->set_max_speed(drake::symbolic::Variable(kNames[kMaxSpeed]));
    this->set_velocity_limit_kp(drake::symbolic::Variable(kNames[kVelocityLimitKp]));
  }

  RailFollowerParams<T>* DoClone() const final { return new RailFollowerParams; }

  /// @name Getters and Setters
  //@{
  /// The vehicle's position on the lane's r-axis.
  /// @note @c r is expressed in units of m.
  const T& r() const {
    ThrowIfEmpty();
    return this->GetAtIndex(kR);
  }
  /// Setter that matches r().
  void set_r(const T& r) {
    ThrowIfEmpty();
    this->SetAtIndex(kR, r);
  }
  /// Fluent setter that matches r().
  /// Returns a copy of `this` with r set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  RailFollowerParams<T> with_r(const T& r) const {
    RailFollowerParams<T> result(*this);
    result.set_r(r);
    return result;
  }
  /// The vehicle's height above the lane's surface.
  /// @note @c h is expressed in units of m.
  const T& h() const {
    ThrowIfEmpty();
    return this->GetAtIndex(kH);
  }
  /// Setter that matches h().
  void set_h(const T& h) {
    ThrowIfEmpty();
    this->SetAtIndex(kH, h);
  }
  /// Fluent setter that matches h().
  /// Returns a copy of `this` with h set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  RailFollowerParams<T> with_h(const T& h) const {
    RailFollowerParams<T> result(*this);
    result.set_h(h);
    return result;
  }
  /// The limit on the vehicle's forward speed, in meters per second; this
  /// element must be positive.
  /// @note @c max_speed is expressed in units of m/s.
  const T& max_speed() const {
    ThrowIfEmpty();
    return this->GetAtIndex(kMaxSpeed);
  }
  /// Setter that matches max_speed().
  void set_max_speed(const T& max_speed) {
    ThrowIfEmpty();
    this->SetAtIndex(kMaxSpeed, max_speed);
  }
  /// Fluent setter that matches max_speed().
  /// Returns a copy of `this` with max_speed set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  RailFollowerParams<T> with_max_speed(const T& max_speed) const {
    RailFollowerParams<T> result(*this);
    result.set_max_speed(max_speed);
    return result;
  }
  /// The smoothing constant for min/max velocity limits; this element must be
  /// positive.
  /// @note @c velocity_limit_kp is expressed in units of Hz.
  const T& velocity_limit_kp() const {
    ThrowIfEmpty();
    return this->GetAtIndex(kVelocityLimitKp);
  }
  /// Setter that matches velocity_limit_kp().
  void set_velocity_limit_kp(const T& velocity_limit_kp) { this->SetAtIndex(kVelocityLimitKp, velocity_limit_kp); }
  /// Fluent setter that matches velocity_limit_kp().
  /// Returns a copy of `this` with velocity_limit_kp set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  RailFollowerParams<T> with_velocity_limit_kp(const T& velocity_limit_kp) const {
    RailFollowerParams<T> result(*this);
    result.set_max_velocity_limit_kp(velocity_limit_kp);
    return result;
  }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(r());
    result = result && !isnan(h());
    result = result && !isnan(max_speed());
    result = result && !isnan(velocity_limit_kp());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The RailFollowerParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
