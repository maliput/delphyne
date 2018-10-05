// Copyright 2018 Toyota Research Institute

#pragma once

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_bool.h"
#include "drake/common/dummy_value.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace automotive {

/// Describes the row indices of a SimpleCarState.
struct SimpleCarStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 4;

  // The index of each individual coordinate.
  static const int kX = 0;
  static const int kY = 1;
  static const int kHeading = 2;
  static const int kVelocity = 3;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `SimpleCarStateIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class SimpleCarState final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef SimpleCarStateIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c x defaults to 0.0 with unknown units.
  /// @arg @c y defaults to 0.0 with unknown units.
  /// @arg @c heading defaults to 0.0 with unknown units.
  /// @arg @c velocity defaults to 0.0 with unknown units.
  SimpleCarState() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_x(0.0);
    this->set_y(0.0);
    this->set_heading(0.0);
    this->set_velocity(0.0);
  }

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<std::is_same<U, symbolic::Expression>::value>::type
  SetToNamedVariables() {
    this->set_x(symbolic::Variable("x"));
    this->set_y(symbolic::Variable("y"));
    this->set_heading(symbolic::Variable("heading"));
    this->set_velocity(symbolic::Variable("velocity"));
  }

  SimpleCarState<T>* DoClone() const final { return new SimpleCarState; }

  /// @name Getters and Setters
  //@{
  /// x
  const T& x() const { return this->GetAtIndex(K::kX); }
  void set_x(const T& x) { this->SetAtIndex(K::kX, x); }
  /// y
  const T& y() const { return this->GetAtIndex(K::kY); }
  void set_y(const T& y) { this->SetAtIndex(K::kY, y); }
  /// heading
  const T& heading() const { return this->GetAtIndex(K::kHeading); }
  void set_heading(const T& heading) { this->SetAtIndex(K::kHeading, heading); }
  /// velocity
  const T& velocity() const { return this->GetAtIndex(K::kVelocity); }
  void set_velocity(const T& velocity) {
    this->SetAtIndex(K::kVelocity, velocity);
  }
  //@}

  /// See SimpleCarStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return SimpleCarStateIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::scalar_predicate_t<T> IsValid() const {
    using std::isnan;
    drake::scalar_predicate_t<T> result{true};
    result = result && !isnan(x());
    result = result && !isnan(y());
    result = result && !isnan(heading());
    result = result && !isnan(velocity());
    return result;
  }
};

}  // namespace automotive
}  // namespace drake
