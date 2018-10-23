// Copyright 2018 Toyota Research Institute

#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <drake/common/drake_bool.h>
#include <drake/common/dummy_value.h>
#include <drake/common/never_destroyed.h>
#include <drake/common/symbolic.h>
#include <drake/systems/framework/basic_vector.h>

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

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<
      std::is_same<U, drake::symbolic::Expression>::value>::type
  SetToNamedVariables() {
    this->set_s(drake::symbolic::Variable("s"));
    this->set_speed(drake::symbolic::Variable("speed"));
  }

  MaliputRailcarState<T>* DoClone() const final {
    return new MaliputRailcarState;
  }

  /// @name Getters and Setters
  //@{
  /// The s-coordinate of the vehicle in a `Lane`-frame.
  const T& s() const { return this->GetAtIndex(K::kS); }
  void set_s(const T& s) { this->SetAtIndex(K::kS, s); }
  /// The speed of the vehicle in physical space.
  const T& speed() const { return this->GetAtIndex(K::kSpeed); }
  void set_speed(const T& speed) { this->SetAtIndex(K::kSpeed, speed); }
  //@}

  /// See MaliputRailcarStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return MaliputRailcarStateIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::scalar_predicate_t<T> IsValid() const {
    using std::isnan;
    drake::scalar_predicate_t<T> result{true};
    result = result && !isnan(s());
    result = result && !isnan(speed());
    return result;
  }
};

}  // namespace delphyne
