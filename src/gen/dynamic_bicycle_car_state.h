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
  DynamicBicycleCarState()
      : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_p_LoCp_x(0.0);
    this->set_p_LoCp_y(0.0);
    this->set_yaw_LC(0.0);
    this->set_v_LCp_x(0.0);
    this->set_v_LCp_y(0.0);
    this->set_yawDt_LC(0.0);
  }

  /// Create a drake::symbolic::Variable for each element with the known
  /// variable
  /// name.  This is only available for T == drake::symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<
      std::is_same<U, drake::symbolic::Expression>::value>::type
  SetToNamedVariables() {
    this->set_p_LoCp_x(drake::symbolic::Variable("p_LoCp_x"));
    this->set_p_LoCp_y(drake::symbolic::Variable("p_LoCp_y"));
    this->set_yaw_LC(drake::symbolic::Variable("yaw_LC"));
    this->set_v_LCp_x(drake::symbolic::Variable("v_LCp_x"));
    this->set_v_LCp_y(drake::symbolic::Variable("v_LCp_y"));
    this->set_yawDt_LC(drake::symbolic::Variable("yawDt_LC"));
  }

  DynamicBicycleCarState<T>* DoClone() const final {
    return new DynamicBicycleCarState;
  }

  /// @name Getters and Setters
  //@{
  /// Lx measure of the location of Cp from Lo.
  /// @note @c p_LoCp_x is expressed in units of m.
  const T& p_LoCp_x() const { return this->GetAtIndex(K::kPLocpX); }
  void set_p_LoCp_x(const T& p_LoCp_x) {
    this->SetAtIndex(K::kPLocpX, p_LoCp_x);
  }
  /// Ly measure of the location of Cp from Lo.
  /// @note @c p_LoCp_y is expressed in units of m.
  const T& p_LoCp_y() const { return this->GetAtIndex(K::kPLocpY); }
  void set_p_LoCp_y(const T& p_LoCp_y) {
    this->SetAtIndex(K::kPLocpY, p_LoCp_y);
  }
  /// Yaw angle from Lx to Cx with positive Lz sense.
  /// @note @c yaw_LC is expressed in units of rad.
  const T& yaw_LC() const { return this->GetAtIndex(K::kYawLc); }
  void set_yaw_LC(const T& yaw_LC) { this->SetAtIndex(K::kYawLc, yaw_LC); }
  /// Cx measure of Cp's velocity in L.
  /// @note @c v_LCp_x is expressed in units of m/s.
  const T& v_LCp_x() const { return this->GetAtIndex(K::kVLcpX); }
  void set_v_LCp_x(const T& v_LCp_x) { this->SetAtIndex(K::kVLcpX, v_LCp_x); }
  /// Cy measure of Cp's velocity in L.
  /// @note @c v_LCp_y is expressed in units of m/s.
  const T& v_LCp_y() const { return this->GetAtIndex(K::kVLcpY); }
  void set_v_LCp_y(const T& v_LCp_y) { this->SetAtIndex(K::kVLcpY, v_LCp_y); }
  /// C's angular velocity in frame L.
  /// @note @c yawDt_LC is expressed in units of rad/s.
  const T& yawDt_LC() const { return this->GetAtIndex(K::kYawdtLc); }
  void set_yawDt_LC(const T& yawDt_LC) {
    this->SetAtIndex(K::kYawdtLc, yawDt_LC);
  }
  //@}

  /// See DynamicBicycleCarStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return DynamicBicycleCarStateIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::scalar_predicate_t<T> IsValid() const {
    using std::isnan;
    drake::scalar_predicate_t<T> result{true};
    result = result && !isnan(p_LoCp_x());
    result = result && !isnan(p_LoCp_y());
    result = result && !isnan(yaw_LC());
    result = result && !isnan(v_LCp_x());
    result = result && !isnan(v_LCp_y());
    result = result && !isnan(yawDt_LC());
    return result;
  }
};

}  // namespace delphyne
