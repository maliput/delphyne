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

#include <drake/systems/framework/basic_vector.h>

#include "delphyne/types.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/// Specializes BasicVector with specific getters and setters.
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

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == delphyne::Symbolic.
  template <typename U = T>
  typename std::enable_if<std::is_same<U, Symbolic>::value>::type
  SetToNamedVariables() {
    this->set_r(drake::symbolic::Variable(kNames[kR]));
    this->set_h(drake::symbolic::Variable(kNames[kH]));
    this->set_max_speed(drake::symbolic::Variable(kNames[kMaxSpeed]));
    this->set_velocity_limit_kp(drake::symbolic::Variable(kNames[kVelocityLimitKp]));
  }

  RailFollowerParams<T>* DoClone() const final {
    return new RailFollowerParams;
  }

  /// @name Getters and Setters
  //@{
  /// The vehicle's position on the lane's r-axis.
  /// @note @c r is expressed in units of m.
  const T& r() const { return this->GetAtIndex(kR); }
  void set_r(const T& r) { this->SetAtIndex(kR, r); }
  /// The vehicle's height above the lane's surface.
  /// @note @c h is expressed in units of m.
  const T& h() const { return this->GetAtIndex(kH); }
  void set_h(const T& h) { this->SetAtIndex(kH, h); }
  /// The limit on the vehicle's forward speed, in meters per second; this
  /// element must be positive.
  /// @note @c max_speed is expressed in units of m/s.
  const T& max_speed() const { return this->GetAtIndex(kMaxSpeed); }
  void set_max_speed(const T& max_speed) {
    this->SetAtIndex(kMaxSpeed, max_speed);
  }
  /// The smoothing constant for min/max velocity limits; this element must be
  /// positive.
  /// @note @c velocity_limit_kp is expressed in units of Hz.
  const T& velocity_limit_kp() const {
    return this->GetAtIndex(kVelocityLimitKp);
  }
  void set_velocity_limit_kp(const T& velocity_limit_kp) {
    this->SetAtIndex(kVelocityLimitKp, velocity_limit_kp);
  }
  //@}

  /// Returns whether the current values of this vector are well-formed.
  drake::Bool<T> IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(r());
    result = result && !isnan(h());
    result = result && !isnan(max_speed());
    result = result && !isnan(velocity_limit_kp());
    return result;
  }
};


/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace delphyne

