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

/// Describes the row indices of a MaliputRailcarState.
struct MaliputRailcarStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kS = 0;
  static const int kSpeed = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `MaliputRailcarStateIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class MaliputRailcarState final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef MaliputRailcarStateIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c s defaults to 0.0 with unknown units.
  /// @arg @c speed defaults to 0.0 with unknown units.
  MaliputRailcarState() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_s(0.0);
    this->set_speed(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  MaliputRailcarState(const MaliputRailcarState& other) : drake::systems::BasicVector<T>(other.values()) {}
  MaliputRailcarState(MaliputRailcarState&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  MaliputRailcarState& operator=(const MaliputRailcarState& other) {
    this->values() = other.values();
    return *this;
  }
  MaliputRailcarState& operator=(MaliputRailcarState&& other) noexcept {
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
    this->set_s(drake::symbolic::Variable("s"));
    this->set_speed(drake::symbolic::Variable("speed"));
  }

  MaliputRailcarState<T>* DoClone() const final { return new MaliputRailcarState; }

  /// @name Getters and Setters
  //@{
  /// The s-coordinate of the vehicle in a `Lane`-frame.
  const T& s() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kS);
  }
  /// Setter that matches s().
  void set_s(const T& s) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kS, s);
  }
  /// Fluent setter that matches s().
  /// Returns a copy of `this` with s set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  MaliputRailcarState<T> with_s(const T& s) const {
    MaliputRailcarState<T> result(*this);
    result.set_s(s);
    return result;
  }
  /// The speed of the vehicle in physical space.
  const T& speed() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kSpeed);
  }
  /// Setter that matches speed().
  void set_speed(const T& speed) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kSpeed, speed);
  }
  /// Fluent setter that matches speed().
  /// Returns a copy of `this` with speed set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  MaliputRailcarState<T> with_speed(const T& speed) const {
    MaliputRailcarState<T> result(*this);
    result.set_speed(speed);
    return result;
  }
  //@}

  /// See MaliputRailcarStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return MaliputRailcarStateIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(s());
    result = result && !isnan(speed());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The MaliputRailcarState vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
