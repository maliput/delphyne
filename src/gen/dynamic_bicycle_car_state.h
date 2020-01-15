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

/// Describes the row indices of a DynamicBicycleCarState.
struct DynamicBicycleCarStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 6;

  // The index of each individual coordinate.
  static const int kPLocpX = 0;
  static const int kPLocpY = 1;
  static const int kYawLc = 2;
  static const int kVLcpX = 3;
  static const int kVLcpY = 4;
  static const int kYawdtLc = 5;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `DynamicBicycleCarStateIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class DynamicBicycleCarState final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef DynamicBicycleCarStateIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c p_LoCp_x defaults to 0.0 m.
  /// @arg @c p_LoCp_y defaults to 0.0 m.
  /// @arg @c yaw_LC defaults to 0.0 rad.
  /// @arg @c v_LCp_x defaults to 0.0 m/s.
  /// @arg @c v_LCp_y defaults to 0.0 m/s.
  /// @arg @c yawDt_LC defaults to 0.0 rad/s.
  DynamicBicycleCarState() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_p_LoCp_x(0.0);
    this->set_p_LoCp_y(0.0);
    this->set_yaw_LC(0.0);
    this->set_v_LCp_x(0.0);
    this->set_v_LCp_y(0.0);
    this->set_yawDt_LC(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  DynamicBicycleCarState(const DynamicBicycleCarState& other) : drake::systems::BasicVector<T>(other.values()) {}
  DynamicBicycleCarState(DynamicBicycleCarState&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  DynamicBicycleCarState& operator=(const DynamicBicycleCarState& other) {
    this->values() = other.values();
    return *this;
  }
  DynamicBicycleCarState& operator=(DynamicBicycleCarState&& other) noexcept {
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
    this->set_p_LoCp_x(drake::symbolic::Variable("p_LoCp_x"));
    this->set_p_LoCp_y(drake::symbolic::Variable("p_LoCp_y"));
    this->set_yaw_LC(drake::symbolic::Variable("yaw_LC"));
    this->set_v_LCp_x(drake::symbolic::Variable("v_LCp_x"));
    this->set_v_LCp_y(drake::symbolic::Variable("v_LCp_y"));
    this->set_yawDt_LC(drake::symbolic::Variable("yawDt_LC"));
  }

  DynamicBicycleCarState<T>* DoClone() const final { return new DynamicBicycleCarState; }

  /// @name Getters and Setters
  //@{
  /// Lx measure of the location of Cp from Lo.
  /// @note @c p_LoCp_x is expressed in units of m.
  const T& p_LoCp_x() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kPLocpX);
  }
  /// Setter that matches p_LoCp_x().
  void set_p_LoCp_x(const T& p_LoCp_x) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kPLocpX, p_LoCp_x);
  }
  /// Fluent setter that matches p_LoCp_x().
  /// Returns a copy of `this` with p_LoCp_x set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarState<T> with_p_LoCp_x(const T& p_LoCp_x) const {
    DynamicBicycleCarState<T> result(*this);
    result.set_p_LoCp_x(p_LoCp_x);
    return result;
  }
  /// Ly measure of the location of Cp from Lo.
  /// @note @c p_LoCp_y is expressed in units of m.
  const T& p_LoCp_y() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kPLocpY);
  }
  /// Setter that matches p_LoCp_y().
  void set_p_LoCp_y(const T& p_LoCp_y) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kPLocpY, p_LoCp_y);
  }
  /// Fluent setter that matches p_LoCp_y().
  /// Returns a copy of `this` with p_LoCp_y set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarState<T> with_p_LoCp_y(const T& p_LoCp_y) const {
    DynamicBicycleCarState<T> result(*this);
    result.set_p_LoCp_y(p_LoCp_y);
    return result;
  }
  /// Yaw angle from Lx to Cx with positive Lz sense.
  /// @note @c yaw_LC is expressed in units of rad.
  const T& yaw_LC() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kYawLc);
  }
  /// Setter that matches yaw_LC().
  void set_yaw_LC(const T& yaw_LC) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kYawLc, yaw_LC);
  }
  /// Fluent setter that matches yaw_LC().
  /// Returns a copy of `this` with yaw_LC set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarState<T> with_yaw_LC(const T& yaw_LC) const {
    DynamicBicycleCarState<T> result(*this);
    result.set_yaw_LC(yaw_LC);
    return result;
  }
  /// Cx measure of Cp's velocity in L.
  /// @note @c v_LCp_x is expressed in units of m/s.
  const T& v_LCp_x() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kVLcpX);
  }
  /// Setter that matches v_LCp_x().
  void set_v_LCp_x(const T& v_LCp_x) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kVLcpX, v_LCp_x);
  }
  /// Fluent setter that matches v_LCp_x().
  /// Returns a copy of `this` with v_LCp_x set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarState<T> with_v_LCp_x(const T& v_LCp_x) const {
    DynamicBicycleCarState<T> result(*this);
    result.set_v_LCp_x(v_LCp_x);
    return result;
  }
  /// Cy measure of Cp's velocity in L.
  /// @note @c v_LCp_y is expressed in units of m/s.
  const T& v_LCp_y() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kVLcpY);
  }
  /// Setter that matches v_LCp_y().
  void set_v_LCp_y(const T& v_LCp_y) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kVLcpY, v_LCp_y);
  }
  /// Fluent setter that matches v_LCp_y().
  /// Returns a copy of `this` with v_LCp_y set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarState<T> with_v_LCp_y(const T& v_LCp_y) const {
    DynamicBicycleCarState<T> result(*this);
    result.set_v_LCp_y(v_LCp_y);
    return result;
  }
  /// C's angular velocity in frame L.
  /// @note @c yawDt_LC is expressed in units of rad/s.
  const T& yawDt_LC() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kYawdtLc);
  }
  /// Setter that matches yawDt_LC().
  void set_yawDt_LC(const T& yawDt_LC) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kYawdtLc, yawDt_LC);
  }
  /// Fluent setter that matches yawDt_LC().
  /// Returns a copy of `this` with yawDt_LC set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarState<T> with_yawDt_LC(const T& yawDt_LC) const {
    DynamicBicycleCarState<T> result(*this);
    result.set_yawDt_LC(yawDt_LC);
    return result;
  }
  //@}

  /// See DynamicBicycleCarStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return DynamicBicycleCarStateIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(p_LoCp_x());
    result = result && !isnan(p_LoCp_y());
    result = result && !isnan(yaw_LC());
    result = result && !isnan(v_LCp_x());
    result = result && !isnan(v_LCp_y());
    result = result && !isnan(yawDt_LC());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The DynamicBicycleCarState vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
