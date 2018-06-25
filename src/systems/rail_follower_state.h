// Copyright 2017 Toyota Research Institute

// This is roughly equivalent to the results of a class generated from
// a drake 'named vector' protobuf file. Adding the machinery for writing
// and generating drake named vectors is beyond the scope of delphyne.

#pragma once

/*****************************************************************************
** Includes
*****************************************************************************/

#include <array>
#include <sstream>
#include <string>

#include <drake/common/symbolic_variable.h>
#include <drake/systems/framework/basic_vector.h>

// public headers
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

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<std::is_same<U, Symbolic>::value>::type
  SetToNamedVariables() {
    this->set_s(drake::symbolic::Variable(kNames[kS]));
    this->set_speed(drake::symbolic::Variable(kNames[kSpeed]));
  }

  RailFollowerState<T>* DoClone() const final {
    return new RailFollowerState;
  }

  /// @name Getters and Setters
  //@{
  /// The s-coordinate of the vehicle in a `Lane`-frame.
  const T& s() const { return this->GetAtIndex(kS); }
  void set_s(const T& s) { this->SetAtIndex(kS, s); }
  /// The speed of the vehicle in physical space.
  const T& speed() const { return this->GetAtIndex(kSpeed); }
  void set_speed(const T& speed) { this->SetAtIndex(kSpeed, speed); }
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
  drake::Bool<T> IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(s());
    result = result && !isnan(speed());
    return result;
  }
};


/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace delphyne
