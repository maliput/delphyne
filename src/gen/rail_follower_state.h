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

#include <drake/common/symbolic_variable.h>
#include <drake/systems/framework/basic_vector.h>

// public headers
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
/// specify the continuous state for a RailFollower system.
template <typename T>
class RailFollowerState final : public drake::systems::BasicVector<T> {
 private:
  /// @name Vector indices.
  //@{
  static constexpr int kS = 0;
  static constexpr int kSpeed = 1;
  //@}

  /// String names for each variable in the vector.
  static const std::vector<std::string> kNames;

  /// Default values for all variables.
  static const std::vector<double> kDefaults;

  /// Docstrings for each variable in the vector.
  static const std::vector<std::string> kDocStrings;

 public:
  /// @brief Initialise the vector with defaults.
  RailFollowerState() : drake::systems::BasicVector<T>(2) {
    // set defaults
    this->set_s(kDefaults[kS]);
    this->set_speed(kDefaults[kSpeed]);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  RailFollowerState(const RailFollowerState& other)
      : drake::systems::BasicVector<T>(other.values()) {}
  RailFollowerState(RailFollowerState&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  RailFollowerState& operator=(const RailFollowerState& other) {
    this->values() = other.values();
    return *this;
  }
  RailFollowerState& operator=(RailFollowerState&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }
  //@}

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T ==  delphyne::Symbolic.
  template <typename U = T>
  typename std::enable_if<std::is_same<U, Symbolic>::value>::type
  SetToNamedVariables() {
    this->set_s(drake::symbolic::Variable(kNames[kS]));
    this->set_speed(drake::symbolic::Variable(kNames[kSpeed]));
  }

  RailFollowerState<T>* DoClone() const final { return new RailFollowerState; }

  /// @name Getters and Setters
  //@{
  /// The longitudinal position along the current rail (m).
  const T& s() const {
    ThrowIfEmpty();
    return this->GetAtIndex(kS);
  }
  /// Setter that matches s().
  void set_s(const T& s) {
    ThrowIfEmpty();
    this->SetAtIndex(kS, s);
  }
  /// Fluent setter that matches s().
  /// Returns a copy of `this` with s set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  RailFollowerState<T> with_s(const T& s) const {
    RailFollowerState<T> result(*this);
    result.set_s(s);
    return result;
  }
  /// The speed of the vehicle in physical space (not nec. rail speed) (m/s).
  const T& speed() const {
    ThrowIfEmpty();
    return this->GetAtIndex(kSpeed);
  }
  /// Setter that matches speed().
  void set_speed(const T& speed) {
    ThrowIfEmpty();
    this->SetAtIndex(kSpeed, speed);
  }
  /// Fluent setter that matches speed().
  /// Returns a copy of `this` with speed set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  RailFollowerState<T> with_speed(const T& speed) const {
    RailFollowerState<T> result(*this);
    result.set_speed(speed);
    return result;
  }
  //@}

  /// @brief Verbose string representation.
  std::string ToString() const {
    std::ostringstream oss;
    oss << "Rail Follower State\n";
    for (int index : {kS, kSpeed}) {
      oss << "  " << kNames[index] << ": " << kDocStrings[index] << "\n";
      oss << "    default: " << kDefaults[index] << "\n";
      oss << "    current: " << this->GetAtIndex[index] << "\n";
    }
    return oss.str();
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
          "The RailFollowerState vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
