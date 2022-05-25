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
#include <limits>
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
  BicycleCarParameters(const BicycleCarParameters& other) : drake::systems::BasicVector<T>(other.values()) {}
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
  typename std::enable_if<std::is_same<U, drake::symbolic::Expression>::value>::type SetToNamedVariables() {
    this->set_mass(drake::symbolic::Variable("mass"));
    this->set_lf(drake::symbolic::Variable("lf"));
    this->set_lr(drake::symbolic::Variable("lr"));
    this->set_Iz(drake::symbolic::Variable("Iz"));
    this->set_Cf(drake::symbolic::Variable("Cf"));
    this->set_Cr(drake::symbolic::Variable("Cr"));
  }

  BicycleCarParameters<T>* DoClone() const final { return new BicycleCarParameters; }

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

  void GetElementBounds(Eigen::VectorXd* lower, Eigen::VectorXd* upper) const final {
    const double kInf = std::numeric_limits<double>::infinity();
    *lower = Eigen::Matrix<double, 6, 1>::Constant(-kInf);
    *upper = Eigen::Matrix<double, 6, 1>::Constant(kInf);
    (*lower)(K::kMass) = 0.0;
    (*lower)(K::kLf) = 0.0;
    (*lower)(K::kLr) = 0.0;
    (*lower)(K::kIz) = 0.0;
    (*lower)(K::kCf) = 0.0;
    (*lower)(K::kCr) = 0.0;
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
