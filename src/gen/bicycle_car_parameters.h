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

/// Describes the row indices of a BicycleCarParameters.
struct BicycleCarParametersIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 6;

  // The index of each individual coordinate.
  static const int kMass = 0;
  static const int kLf = 1;
  static const int kLr = 2;
  static const int kIz = 3;
  static const int kCf = 4;
  static const int kCr = 5;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `BicycleCarParametersIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class BicycleCarParameters final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef BicycleCarParametersIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c mass defaults to 2278.0 kg.
  /// @arg @c lf defaults to 1.292 m.
  /// @arg @c lr defaults to 1.515 m.
  /// @arg @c Iz defaults to 3210.0 kg m^2.
  /// @arg @c Cf defaults to 10.8e4 N / rad.
  /// @arg @c Cr defaults to 10.8e4 N / rad.
  BicycleCarParameters() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_mass(2278.0);
    this->set_lf(1.292);
    this->set_lr(1.515);
    this->set_Iz(3210.0);
    this->set_Cf(10.8e4);
    this->set_Cr(10.8e4);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  BicycleCarParameters(const BicycleCarParameters& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  BicycleCarParameters(BicycleCarParameters&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  BicycleCarParameters& operator=(const BicycleCarParameters& other) {
    this->values() = other.values();
    return *this;
  }
  BicycleCarParameters& operator=(BicycleCarParameters&& other) noexcept {
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
    this->set_mass(drake::symbolic::Variable("mass"));
    this->set_lf(drake::symbolic::Variable("lf"));
    this->set_lr(drake::symbolic::Variable("lr"));
    this->set_Iz(drake::symbolic::Variable("Iz"));
    this->set_Cf(drake::symbolic::Variable("Cf"));
    this->set_Cr(drake::symbolic::Variable("Cr"));
  }

  BicycleCarParameters<T>* DoClone() const final {
    return new BicycleCarParameters;
  }

  /// @name Getters and Setters
  //@{
  /// mass
  /// @note @c mass is expressed in units of kg.
  /// @note @c mass has a limited domain of [0.0, +Inf].
  const T& mass() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMass);
  }
  /// Setter that matches mass().
  void set_mass(const T& mass) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMass, mass);
  }
  /// Fluent setter that matches mass().
  /// Returns a copy of `this` with mass set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BicycleCarParameters<T> with_mass(const T& mass) const {
    BicycleCarParameters<T> result(*this);
    result.set_mass(mass);
    return result;
  }
  /// distance from the center of mass to the front axle
  /// @note @c lf is expressed in units of m.
  /// @note @c lf has a limited domain of [0.0, +Inf].
  const T& lf() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kLf);
  }
  /// Setter that matches lf().
  void set_lf(const T& lf) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kLf, lf);
  }
  /// Fluent setter that matches lf().
  /// Returns a copy of `this` with lf set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BicycleCarParameters<T> with_lf(const T& lf) const {
    BicycleCarParameters<T> result(*this);
    result.set_lf(lf);
    return result;
  }
  /// distance from the center of mass to the rear axle
  /// @note @c lr is expressed in units of m.
  /// @note @c lr has a limited domain of [0.0, +Inf].
  const T& lr() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kLr);
  }
  /// Setter that matches lr().
  void set_lr(const T& lr) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kLr, lr);
  }
  /// Fluent setter that matches lr().
  /// Returns a copy of `this` with lr set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BicycleCarParameters<T> with_lr(const T& lr) const {
    BicycleCarParameters<T> result(*this);
    result.set_lr(lr);
    return result;
  }
  /// moment of inertia about the yaw-axis
  /// @note @c Iz is expressed in units of kg m^2.
  /// @note @c Iz has a limited domain of [0.0, +Inf].
  const T& Iz() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kIz);
  }
  /// Setter that matches Iz().
  void set_Iz(const T& Iz) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kIz, Iz);
  }
  /// Fluent setter that matches Iz().
  /// Returns a copy of `this` with Iz set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BicycleCarParameters<T> with_Iz(const T& Iz) const {
    BicycleCarParameters<T> result(*this);
    result.set_Iz(Iz);
    return result;
  }
  /// cornering stiffness (front)
  /// @note @c Cf is expressed in units of N / rad.
  /// @note @c Cf has a limited domain of [0.0, +Inf].
  const T& Cf() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kCf);
  }
  /// Setter that matches Cf().
  void set_Cf(const T& Cf) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kCf, Cf);
  }
  /// Fluent setter that matches Cf().
  /// Returns a copy of `this` with Cf set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BicycleCarParameters<T> with_Cf(const T& Cf) const {
    BicycleCarParameters<T> result(*this);
    result.set_Cf(Cf);
    return result;
  }
  /// cornering stiffness (rear)
  /// @note @c Cr is expressed in units of N / rad.
  /// @note @c Cr has a limited domain of [0.0, +Inf].
  const T& Cr() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kCr);
  }
  /// Setter that matches Cr().
  void set_Cr(const T& Cr) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kCr, Cr);
  }
  /// Fluent setter that matches Cr().
  /// Returns a copy of `this` with Cr set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BicycleCarParameters<T> with_Cr(const T& Cr) const {
    BicycleCarParameters<T> result(*this);
    result.set_Cr(Cr);
    return result;
  }
  //@}

  /// See BicycleCarParametersIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return BicycleCarParametersIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(mass());
    result = result && (mass() >= T(0.0));
    result = result && !isnan(lf());
    result = result && (lf() >= T(0.0));
    result = result && !isnan(lr());
    result = result && (lr() >= T(0.0));
    result = result && !isnan(Iz());
    result = result && (Iz() >= T(0.0));
    result = result && !isnan(Cf());
    result = result && (Cf() >= T(0.0));
    result = result && !isnan(Cr());
    result = result && (Cr() >= T(0.0));
    return result;
  }

  // VectorBase override.
  void CalcInequalityConstraint(drake::VectorX<T>* value) const final {
    value->resize(6);
    (*value)[0] = mass() - T(0.0);
    (*value)[1] = lf() - T(0.0);
    (*value)[2] = lr() - T(0.0);
    (*value)[3] = Iz() - T(0.0);
    (*value)[4] = Cf() - T(0.0);
    (*value)[5] = Cr() - T(0.0);
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The BicycleCarParameters vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
