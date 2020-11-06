// Copyright 2020 Toyota Research Institute

#pragma once

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

/// Describes the row indices of a AngularRateAccelerationCommand.
struct AngularRateAccelerationCommandIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kAngularRate = 0;
  static const int kAcceleration = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `AngularRateAccelerationCommandIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class AngularRateAccelerationCommand final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef AngularRateAccelerationCommandIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c angular_rate defaults to 0.0 rad.
  /// @arg @c acceleration defaults to 0.0 m/s^2.
  AngularRateAccelerationCommand() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_angular_rate(0.0);
    this->set_acceleration(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  AngularRateAccelerationCommand(const AngularRateAccelerationCommand& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  AngularRateAccelerationCommand(AngularRateAccelerationCommand&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  AngularRateAccelerationCommand& operator=(const AngularRateAccelerationCommand& other) {
    this->values() = other.values();
    return *this;
  }
  AngularRateAccelerationCommand& operator=(AngularRateAccelerationCommand&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }
  //@}

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<std::is_same<U, drake::symbolic::Expression>::value>::type SetToNamedVariables() {
    this->set_angular_rate(drake::symbolic::Variable("angular_rate"));
    this->set_acceleration(drake::symbolic::Variable("acceleration"));
  }

  AngularRateAccelerationCommand<T>* DoClone() const final { return new AngularRateAccelerationCommand; }

  /// @name Getters and Setters
  //@{
  /// The desired angular rate; positive results in the vehicle turning left.
  /// @note @c angular_rate is expressed in units of rad/s.
  const T& angular_rate() const { return this->GetAtIndex(K::kAngularRate); }
  void set_angular_rate(const T& angular_rate) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kAngularRate, angular_rate);
  }
  /// Fluent setter that matches angular_rate().
  /// Returns a copy of `this` with angular_rate set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  AngularRateAccelerationCommand<T> with_angular_rate(const T& angular_rate) const {
    AngularRateAccelerationCommand<T> result(*this);
    result.set_angular_rate(angular_rate);
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
  AngularRateAccelerationCommand<T> with_acceleration(const T& acceleration) const {
    AngularRateAccelerationCommand<T> result(*this);
    result.set_acceleration(acceleration);
    return result;
  }
  //@}

  /// See AngularRateAccelerationCommandIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return AngularRateAccelerationCommandIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(angular_rate());
    result = result && !isnan(acceleration());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The AngularRateAccelerationCommand vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
