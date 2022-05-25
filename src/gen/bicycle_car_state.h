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

/// Describes the row indices of a BicycleCarState.
struct BicycleCarStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 6;

  // The index of each individual coordinate.
  static const int kPsi = 0;
  static const int kPsiDot = 1;
  static const int kBeta = 2;
  static const int kVel = 3;
  static const int kSx = 4;
  static const int kSy = 5;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `BicycleCarStateIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class BicycleCarState final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef BicycleCarStateIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c Psi defaults to 0.0 with unknown units.
  /// @arg @c Psi_dot defaults to 0.0 with unknown units.
  /// @arg @c beta defaults to 0.0 with unknown units.
  /// @arg @c vel defaults to 0.0 with unknown units.
  /// @arg @c sx defaults to 0.0 with unknown units.
  /// @arg @c sy defaults to 0.0 with unknown units.
  BicycleCarState() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_Psi(0.0);
    this->set_Psi_dot(0.0);
    this->set_beta(0.0);
    this->set_vel(0.0);
    this->set_sx(0.0);
    this->set_sy(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  BicycleCarState(const BicycleCarState& other) : drake::systems::BasicVector<T>(other.values()) {}
  BicycleCarState(BicycleCarState&& other) noexcept : drake::systems::BasicVector<T>(std::move(other.values())) {}
  BicycleCarState& operator=(const BicycleCarState& other) {
    this->values() = other.values();
    return *this;
  }
  BicycleCarState& operator=(BicycleCarState&& other) noexcept {
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
    this->set_Psi(drake::symbolic::Variable("Psi"));
    this->set_Psi_dot(drake::symbolic::Variable("Psi_dot"));
    this->set_beta(drake::symbolic::Variable("beta"));
    this->set_vel(drake::symbolic::Variable("vel"));
    this->set_sx(drake::symbolic::Variable("sx"));
    this->set_sy(drake::symbolic::Variable("sy"));
  }

  BicycleCarState<T>* DoClone() const final { return new BicycleCarState; }

  /// @name Getters and Setters
  //@{
  /// yaw angle
  const T& Psi() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kPsi);
  }
  /// Setter that matches Psi().
  void set_Psi(const T& Psi) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kPsi, Psi);
  }
  /// Fluent setter that matches Psi().
  /// Returns a copy of `this` with Psi set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BicycleCarState<T> with_Psi(const T& Psi) const {
    BicycleCarState<T> result(*this);
    result.set_Psi(Psi);
    return result;
  }
  /// yaw angular rate
  const T& Psi_dot() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kPsiDot);
  }
  /// Setter that matches Psi_dot().
  void set_Psi_dot(const T& Psi_dot) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kPsiDot, Psi_dot);
  }
  /// Fluent setter that matches Psi_dot().
  /// Returns a copy of `this` with Psi_dot set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BicycleCarState<T> with_Psi_dot(const T& Psi_dot) const {
    BicycleCarState<T> result(*this);
    result.set_Psi_dot(Psi_dot);
    return result;
  }
  /// slip angle at the center of mass
  const T& beta() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kBeta);
  }
  /// Setter that matches beta().
  void set_beta(const T& beta) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kBeta, beta);
  }
  /// Fluent setter that matches beta().
  /// Returns a copy of `this` with beta set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BicycleCarState<T> with_beta(const T& beta) const {
    BicycleCarState<T> result(*this);
    result.set_beta(beta);
    return result;
  }
  /// velocity magnitude
  const T& vel() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kVel);
  }
  /// Setter that matches vel().
  void set_vel(const T& vel) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kVel, vel);
  }
  /// Fluent setter that matches vel().
  /// Returns a copy of `this` with vel set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BicycleCarState<T> with_vel(const T& vel) const {
    BicycleCarState<T> result(*this);
    result.set_vel(vel);
    return result;
  }
  /// x-position at the center of mass
  const T& sx() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kSx);
  }
  /// Setter that matches sx().
  void set_sx(const T& sx) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kSx, sx);
  }
  /// Fluent setter that matches sx().
  /// Returns a copy of `this` with sx set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BicycleCarState<T> with_sx(const T& sx) const {
    BicycleCarState<T> result(*this);
    result.set_sx(sx);
    return result;
  }
  /// y-position at the center of mass
  const T& sy() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kSy);
  }
  /// Setter that matches sy().
  void set_sy(const T& sy) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kSy, sy);
  }
  /// Fluent setter that matches sy().
  /// Returns a copy of `this` with sy set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  BicycleCarState<T> with_sy(const T& sy) const {
    BicycleCarState<T> result(*this);
    result.set_sy(sy);
    return result;
  }
  //@}

  /// See BicycleCarStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() { return BicycleCarStateIndices::GetCoordinateNames(); }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(Psi());
    result = result && !isnan(Psi_dot());
    result = result && !isnan(beta());
    result = result && !isnan(vel());
    result = result && !isnan(sx());
    result = result && !isnan(sy());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The BicycleCarState vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
