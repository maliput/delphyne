// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
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
#include <drake/common/symbolic.h>
#include <drake/systems/framework/basic_vector.h>
#include <maliput/common/maliput_never_destroyed.h>

// TODO(jwnimmer-tri) Elevate this to drake/common.
#if __has_cpp_attribute(nodiscard)
#define DRAKE_VECTOR_GEN_NODISCARD [[nodiscard]]  // NOLINT(whitespace/braces)
#else
#define DRAKE_VECTOR_GEN_NODISCARD
#endif

namespace delphyne {

/// Describes the row indices of a DrivingCommand.
struct DrivingCommandIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kSteeringAngle = 0;
  static const int kAcceleration = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `DrivingCommandIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class DrivingCommand final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef DrivingCommandIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c steering_angle defaults to 0.0 rad.
  /// @arg @c acceleration defaults to 0.0 m/s^2.
  DrivingCommand() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_steering_angle(0.0);
    this->set_acceleration(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  DrivingCommand(const DrivingCommand& other) : drake::systems::BasicVector<T>(other.values()) {}
  DrivingCommand(DrivingCommand&& other) noexcept : drake::systems::BasicVector<T>(std::move(other.values())) {}
  DrivingCommand& operator=(const DrivingCommand& other) {
    this->values() = other.values();
    return *this;
  }
  DrivingCommand& operator=(DrivingCommand&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }
  //@}

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<std::is_same<U, drake::symbolic::Expression>::value>::type SetToNamedVariables() {
    this->set_steering_angle(drake::symbolic::Variable("steering_angle"));
    this->set_acceleration(drake::symbolic::Variable("acceleration"));
  }

  DrivingCommand<T>* DoClone() const final { return new DrivingCommand; }

  /// @name Getters and Setters
  //@{
  /// The desired steering angle of a virtual center wheel, positive results in
  /// the vehicle turning left.
  /// @note @c steering_angle is expressed in units of rad.
  const T& steering_angle() const { return this->GetAtIndex(K::kSteeringAngle); }
  void set_steering_angle(const T& steering_angle) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kSteeringAngle, steering_angle);
  }
  /// Fluent setter that matches steering_angle().
  /// Returns a copy of `this` with steering_angle set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DrivingCommand<T> with_steering_angle(const T& steering_angle) const {
    DrivingCommand<T> result(*this);
    result.set_steering_angle(steering_angle);
    return result;
  }
  /// The signed acceleration, positive means speed up; negative means slow
  /// down, but should not move in reverse.
  /// @note @c acceleration is expressed in units of m/s^2.
  const T& acceleration() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kAcceleration);
  }
  // Setter that matches acceleration().
  void set_acceleration(const T& acceleration) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kAcceleration, acceleration);
  }
  /// Fluent setter that matches acceleration().
  /// Returns a copy of `this` with acceleration set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DrivingCommand<T> with_acceleration(const T& acceleration) const {
    DrivingCommand<T> result(*this);
    result.set_acceleration(acceleration);
    return result;
  }
  //@}

  /// See DrivingCommandIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() { return DrivingCommandIndices::GetCoordinateNames(); }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(steering_angle());
    result = result && !isnan(acceleration());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The DrivingCommand vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
