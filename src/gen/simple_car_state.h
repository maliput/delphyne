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

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  SimpleCarState(const SimpleCarState& other) : drake::systems::BasicVector<T>(other.values()) {}
  SimpleCarState(SimpleCarState&& other) noexcept : drake::systems::BasicVector<T>(std::move(other.values())) {}
  SimpleCarState& operator=(const SimpleCarState& other) {
    this->values() = other.values();
    return *this;
  }
  SimpleCarState& operator=(SimpleCarState&& other) noexcept {
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
    this->set_x(drake::symbolic::Variable("x"));
    this->set_y(drake::symbolic::Variable("y"));
    this->set_heading(drake::symbolic::Variable("heading"));
    this->set_velocity(drake::symbolic::Variable("velocity"));
  }

  SimpleCarState<T>* DoClone() const final { return new SimpleCarState; }

  /// @name Getters and Setters
  //@{
  /// x
  const T& x() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kX);
  }
  /// Setter that matches x().
  void set_x(const T& x) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kX, x);
  }
  /// Fluent setter that matches x().
  /// Returns a copy of `this` with x set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  SimpleCarState<T> with_x(const T& x) const {
    SimpleCarState<T> result(*this);
    result.set_x(x);
    return result;
  }
  /// y
  const T& y() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kY);
  }
  /// Setter that matches y().
  void set_y(const T& y) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kY, y);
  }
  /// Fluent setter that matches y().
  /// Returns a copy of `this` with y set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  SimpleCarState<T> with_y(const T& y) const {
    SimpleCarState<T> result(*this);
    result.set_y(y);
    return result;
  }
  /// heading
  const T& heading() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kHeading);
  }
  /// Setter that matches heading().
  void set_heading(const T& heading) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kHeading, heading);
  }
  /// Fluent setter that matches heading().
  /// Returns a copy of `this` with heading set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  SimpleCarState<T> with_heading(const T& heading) const {
    SimpleCarState<T> result(*this);
    result.set_heading(heading);
    return result;
  }
  /// velocity
  const T& velocity() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kVelocity);
  }
  /// Setter that matches velocity().
  void set_velocity(const T& velocity) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kVelocity, velocity);
  }
  /// Fluent setter that matches velocity().
  /// Returns a copy of `this` with velocity set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  SimpleCarState<T> with_velocity(const T& velocity) const {
    SimpleCarState<T> result(*this);
    result.set_velocity(velocity);
    return result;
  }
  //@}

  /// See SimpleCarStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() { return SimpleCarStateIndices::GetCoordinateNames(); }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(x());
    result = result && !isnan(y());
    result = result && !isnan(heading());
    result = result && !isnan(velocity());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The SimpleCarState vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
