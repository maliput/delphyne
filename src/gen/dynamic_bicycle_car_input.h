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

/// Describes the row indices of a DynamicBicycleCarInput.
struct DynamicBicycleCarInputIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 2;

  // The index of each individual coordinate.
  static const int kSteerCd = 0;
  static const int kFCpX = 1;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `DynamicBicycleCarInputIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class DynamicBicycleCarInput final : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef DynamicBicycleCarInputIndices K;

  /// Default constructor.  Sets all rows to their default value:
  /// @arg @c steer_CD defaults to 0.0 rad.
  /// @arg @c f_Cp_x defaults to 0.0 N.
  DynamicBicycleCarInput()
      : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_steer_CD(0.0);
    this->set_f_Cp_x(0.0);
  }

  /// Create a drake::symbolic::Variable for each element with the known
  /// variable
  /// name.  This is only available for T == drake::symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<
      std::is_same<U, drake::symbolic::Expression>::value>::type
  SetToNamedVariables() {
    this->set_steer_CD(drake::symbolic::Variable("steer_CD"));
    this->set_f_Cp_x(drake::symbolic::Variable("f_Cp_x"));
  }

  DynamicBicycleCarInput<T>* DoClone() const final {
    return new DynamicBicycleCarInput;
  }

  /// @name Getters and Setters
  //@{
  /// Steer angle from Cx to Dx with positive Cz sense.
  /// @note @c steer_CD is expressed in units of rad.
  const T& steer_CD() const { return this->GetAtIndex(K::kSteerCd); }
  void set_steer_CD(const T& steer_CD) {
    this->SetAtIndex(K::kSteerCd, steer_CD);
  }
  /// The Cx measure of the Longitudinal force on body C at Cp.
  /// @note @c f_Cp_x is expressed in units of N.
  const T& f_Cp_x() const { return this->GetAtIndex(K::kFCpX); }
  void set_f_Cp_x(const T& f_Cp_x) { this->SetAtIndex(K::kFCpX, f_Cp_x); }
  //@}

  /// See DynamicBicycleCarInputIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return DynamicBicycleCarInputIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::scalar_predicate_t<T> IsValid() const {
    using std::isnan;
    drake::scalar_predicate_t<T> result{true};
    result = result && !isnan(steer_CD());
    result = result && !isnan(f_Cp_x());
    return result;
  }
};

}  // namespace delphyne
