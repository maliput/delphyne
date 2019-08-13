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
#include <drake/common/never_destroyed.h>
#include <drake/common/symbolic.h>
#include <drake/systems/framework/basic_vector.h>

// TODO(jwnimmer-tri) Elevate this to drake/common.
#if __has_cpp_attribute(nodiscard)
#define DRAKE_VECTOR_GEN_NODISCARD [[nodiscard]]  // NOLINT(whitespace/braces)
#else
#define DRAKE_VECTOR_GEN_NODISCARD
#endif

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
  DynamicBicycleCarInput() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_steer_CD(0.0);
    this->set_f_Cp_x(0.0);
  }

  // Note: It's safe to implement copy and move because this class is final.

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable
  //@{
  DynamicBicycleCarInput(const DynamicBicycleCarInput& other) : drake::systems::BasicVector<T>(other.values()) {}
  DynamicBicycleCarInput(DynamicBicycleCarInput&& other) noexcept
      : drake::systems::BasicVector<T>(std::move(other.values())) {}
  DynamicBicycleCarInput& operator=(const DynamicBicycleCarInput& other) {
    this->values() = other.values();
    return *this;
  }
  DynamicBicycleCarInput& operator=(DynamicBicycleCarInput&& other) noexcept {
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
    this->set_steer_CD(drake::symbolic::Variable("steer_CD"));
    this->set_f_Cp_x(drake::symbolic::Variable("f_Cp_x"));
  }

  DynamicBicycleCarInput<T>* DoClone() const final { return new DynamicBicycleCarInput; }

  /// @name Getters and Setters
  //@{
  /// Steer angle from Cx to Dx with positive Cz sense.
  /// @note @c steer_CD is expressed in units of rad.
  const T& steer_CD() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kSteerCd);
  }
  /// Setter that matches steer_CD().
  void set_steer_CD(const T& steer_CD) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kSteerCd, steer_CD);
  }
  /// Fluent setter that matches steer_CD().
  /// Returns a copy of `this` with steer_CD set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarInput<T> with_steer_CD(const T& steer_CD) const {
    DynamicBicycleCarInput<T> result(*this);
    result.set_steer_CD(steer_CD);
    return result;
  }
  /// The Cx measure of the Longitudinal force on body C at Cp.
  /// @note @c f_Cp_x is expressed in units of N.
  const T& f_Cp_x() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kFCpX);
  }
  /// Setter that matches f_Cp_x().
  void set_f_Cp_x(const T& f_Cp_x) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kFCpX, f_Cp_x);
  }
  /// Fluent setter that matches f_Cp_x().
  /// Returns a copy of `this` with f_Cp_x set to a new value.
  DRAKE_VECTOR_GEN_NODISCARD
  DynamicBicycleCarInput<T> with_f_Cp_x(const T& f_Cp_x) const {
    DynamicBicycleCarInput<T> result(*this);
    result.set_f_Cp_x(f_Cp_x);
    return result;
  }
  //@}

  /// See DynamicBicycleCarInputIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return DynamicBicycleCarInputIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::boolean<T> IsValid() const {
    using std::isnan;
    drake::boolean<T> result{true};
    result = result && !isnan(steer_CD());
    result = result && !isnan(f_Cp_x());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->values().size() == 0) {
      throw std::out_of_range(
          "The DynamicBicycleCarInput vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};

}  // namespace delphyne

#undef DRAKE_VECTOR_GEN_NODISCARD
