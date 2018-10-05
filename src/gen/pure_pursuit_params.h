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

/// Describes the row indices of a PurePursuitParams.
struct PurePursuitParamsIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 1;

  // The index of each individual coordinate.
  static const int kSLookahead = 0;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `PurePursuitParamsIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class PurePursuitParams final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef PurePursuitParamsIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c s_lookahead defaults to 15.0 m.
  PurePursuitParams() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_s_lookahead(15.0);
  }

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<std::is_same<U, symbolic::Expression>::value>::type
  SetToNamedVariables() {
    this->set_s_lookahead(symbolic::Variable("s_lookahead"));
  }

  PurePursuitParams<T>* DoClone() const final { return new PurePursuitParams; }

  /// @name Getters and Setters
  //@{
  /// distance along the s-direction to place the reference point
  /// @note @c s_lookahead is expressed in units of m.
  /// @note @c s_lookahead has a limited domain of [0.0, +Inf].
  const T& s_lookahead() const { return this->GetAtIndex(K::kSLookahead); }
  void set_s_lookahead(const T& s_lookahead) {
    this->SetAtIndex(K::kSLookahead, s_lookahead);
  }
  //@}

  /// See PurePursuitParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return PurePursuitParamsIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::scalar_predicate_t<T> IsValid() const {
    using std::isnan;
    drake::scalar_predicate_t<T> result{true};
    result = result && !isnan(s_lookahead());
    result = result && (s_lookahead() >= T(0.0));
    return result;
  }

  // VectorBase override.
  void CalcInequalityConstraint(drake::VectorX<T>* value) const final {
    value->resize(1);
    (*value)[0] = s_lookahead() - T(0.0);
  }
};

}  // namespace automotive
}  // namespace drake
