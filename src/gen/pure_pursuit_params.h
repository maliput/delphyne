// Copyright 2018 Toyota Research Institute

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
  PurePursuitParams() : drake::systems::BasicVector<T>(K::kNumCoordinates) { this->set_s_lookahead(10.0); }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  PurePursuitParams(const PurePursuitParams& other) : drake::systems::BasicVector<T>(other.values()) {}
  PurePursuitParams(PurePursuitParams&& other) noexcept : drake::systems::BasicVector<T>(std::move(other.values())) {}
  PurePursuitParams& operator=(const PurePursuitParams& other) {
    this->values() = other.values();
    return *this;
  }
  PurePursuitParams& operator=(PurePursuitParams&& other) noexcept {
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
    this->set_s_lookahead(drake::symbolic::Variable("s_lookahead"));
  }

  PurePursuitParams<T>* DoClone() const final { return new PurePursuitParams; }

  /// @name Getters and Setters
  //@{
  /// distance along the s-direction to place the reference point
  /// @note @c s_lookahead is expressed in units of m.
  /// @note @c s_lookahead has a limited domain of [0.0, +Inf].
  const T& s_lookahead() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kSLookahead);
  }
  /// Setter that matches s_lookahead().
  void set_s_lookahead(const T& s_lookahead) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kSLookahead, s_lookahead);
  }
  /// Fluent setter that matches s_lookahead().
  /// Returns a copy of `this` with s_lookahead set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  PurePursuitParams<T> with_s_lookahead(const T& s_lookahead) const {
    PurePursuitParams<T> result(*this);
    result.set_s_lookahead(s_lookahead);
    return result;
  }
  //@}

  /// See PurePursuitParamsIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() { return PurePursuitParamsIndices::GetCoordinateNames(); }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(s_lookahead());
    result = result && (s_lookahead() >= T(0.0));
    return result;
  }

  void GetElementBounds(Eigen::VectorXd* lower, Eigen::VectorXd* upper) const final {
    const double kInf = std::numeric_limits<double>::infinity();
    *lower = Eigen::Matrix<double, 1, 1>::Constant(-kInf);
    *upper = Eigen::Matrix<double, 1, 1>::Constant(kInf);
    (*lower)(K::kSLookahead) = 0.0;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The PurePursuitParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
