// Copyright 2018 Toyota Research Institute

#pragma once

#include <cstdint>
#include <type_traits>

#include <drake/lcmt_viewer_geometry_data.hpp>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/vector_base.h>

#include "backend/translate_exception.h"
#include "delphyne/macros.h"

namespace delphyne {

/// @brief A system that translates Drake messages on its single input port
/// (which will be discrete or abstract based on the type of the Drake message)
/// to an ignition message on its single abstract output port. This is a base
/// class that provides the input-output port boilerplate and helper translator
/// functions: derived classes need to implement the actual translation.
///
/// @tparam DRAKE_TYPE must be a valid Drake message type.
/// @tparam IGN_TYPE must be a valid ignition message type.
template <class DRAKE_TYPE, class IGN_TYPE>
class DrakeToIgn : public drake::systems::LeafSystem<double> {
 public:
  // Two constructors exist, but only one is enabled, depending on DRAKE_TYPE.

  // @brief Constructor for translators with a DRAKE_TYPE that inherits from
  // drake::systems::VectorBase.
  //
  // @param[in] vector_size The size of the vector stored in the input port.
  template <class T = DRAKE_TYPE>
  DrakeToIgn(int vector_size,
             typename std::enable_if<
                 std::is_base_of<drake::systems::VectorBase<double>, T>::value,
                 void>::type* = 0) {
    // Vector input port.
    DeclareInputPort(drake::systems::kVectorValued, vector_size);

    // Output port (abstract for all ignition types).
    DeclareAbstractOutputPort(&DrakeToIgn::CalcIgnMessage);
  }

  // @brief Constructor for translators with a DRAKE_TYPE that does not inherit
  // from
  // drake::systems::VectorBase.
  //
  // Takes no parameters.
  template <class T = DRAKE_TYPE>
  DrakeToIgn(typename std::enable_if<
                 !std::is_base_of<drake::systems::VectorBase<double>, T>::value,
                 void>::type* = 0) {
    // Abstract input port.
    DeclareAbstractInputPort(drake::systems::kUseDefaultName,
                             drake::Value<DRAKE_TYPE>());

    // Output port (abstract for all ignition types).
    DeclareAbstractOutputPort(&DrakeToIgn::CalcIgnMessage);
  }

 protected:
  // @brief Translates a @p drake_message into an @p ign_message. All derived
  // translators must implement this method with the actual translation.
  // @p ign_message is guaranteed to not be null, but it is NOT ever
  // re-constructed: the same object is copied and passed on each call. This
  // function must perform any required cleanup from the previous call.
  // @see DeclareAbstractOutputPort
  //
  // @param[in] time_ms The curent time, in milliseconds.
  virtual void DoDrakeToIgnTranslation(const DRAKE_TYPE& drake_message,
                                       IGN_TYPE* ign_message,
                                       int64_t) const = 0;

  // Translation helper functions and constants, to be used by derived
  // translators.

  static const unsigned int kPositionVectorSize{3};
  static const unsigned int kOrientationVectorSize{4};

  // @brief Converts an array of floats to an
  // ignition position message.
  //
  // @param[in] position The position array
  // @param[out] ign_position The ignition position message.
  static void PositionArrayToIgnition(const float position[3],
                                      ignition::msgs::Vector3d* ign_position) {
    DELPHYNE_VALIDATE(ign_position != nullptr, std::invalid_argument,
                      "Ignition vector pointer must not be null");

    ign_position->set_x(position[0]);
    ign_position->set_y(position[1]);
    ign_position->set_z(position[2]);
  }

  // @brief Converts an array of floats to an
  // ignition quaternion message.
  //
  // @param[in] quaternion The orientation array.
  // @param[out] ign_quaternion The ign quaternion message.
  static void QuaternionArrayToIgnition(
      const float quaternion[4], ignition::msgs::Quaternion* ign_quaternion) {
    DELPHYNE_VALIDATE(ign_quaternion != nullptr, std::invalid_argument,
                      "Ignition quaternion pointer must not be null");

    ign_quaternion->set_w(quaternion[0]);
    ign_quaternion->set_x(quaternion[1]);
    ign_quaternion->set_y(quaternion[2]);
    ign_quaternion->set_z(quaternion[3]);
  }

  // @brief Converts an array of floats (LCM's type for a color) to an
  // ignition color message.
  //
  // @param[in] lcm_color The LCM color array.
  // @param[out] ign_color The ign color message.
  static void LcmColorToIgnition(const float lcm_color[4],
                                 ignition::msgs::Color* ign_color) {
    DELPHYNE_VALIDATE(ign_color != nullptr, std::invalid_argument,
                      "Ignition color pointer must not be null");

    ign_color->set_r(lcm_color[0]);
    ign_color->set_g(lcm_color[1]);
    ign_color->set_b(lcm_color[2]);
    ign_color->set_a(lcm_color[3]);
  }

  // @brief Converts an LCM geometry to an ignition geometry. Note that an LCM
  // geometry has fields (such as color and position) for which the ignition
  // counterpart is not a geometry: these fields are not converted by this
  // function.
  //
  // @param[in] lcm_color The LCM geometry.
  // @param[out] ign_color The ign geometry.
  static void LcmGeometryToIgnition(
      const drake::lcmt_viewer_geometry_data& lcm_geometry,
      ignition::msgs::Geometry* ign_geometry) {
    // Call the specialized overload for each geometry type. Because the
    // ignition geometry message itself also needs to be aware of the type of
    // the inner geometry message (box, sphere, etc.), said type must be added
    // here.
    switch (lcm_geometry.type) {
      case drake::lcmt_viewer_geometry_data::BOX:
        ign_geometry->set_type(ignition::msgs::Geometry::BOX);
        LcmBoxToIgnition(lcm_geometry, ign_geometry->mutable_box());
        break;

      case drake::lcmt_viewer_geometry_data::SPHERE:
        ign_geometry->set_type(ignition::msgs::Geometry::SPHERE);
        LcmSphereToIgnition(lcm_geometry, ign_geometry->mutable_sphere());
        break;

      case drake::lcmt_viewer_geometry_data::CYLINDER:
        ign_geometry->set_type(ignition::msgs::Geometry::CYLINDER);
        LcmCylinderToIgnition(lcm_geometry, ign_geometry->mutable_cylinder());
        break;

      case drake::lcmt_viewer_geometry_data::MESH:
        ign_geometry->set_type(ignition::msgs::Geometry::MESH);
        LcmMeshToIgnition(lcm_geometry, ign_geometry->mutable_mesh());
        break;

      default:
        throw TranslateException("Cannot translate geometry of type: " +
                                 std::to_string(lcm_geometry.type));
    }
  }

  // @brief Converts an LCM geometry to an ignition box geometry. The LCM
  // goemetry must represent a box geometry.
  //
  // @param[in] lcm_color The LCM geometry.
  // @param[out] ign_color The ign box geometry.
  static void LcmBoxToIgnition(const drake::lcmt_viewer_geometry_data& lcm_box,
                               ignition::msgs::BoxGeom* ign_box) {
    DELPHYNE_VALIDATE(lcm_box.type == drake::lcmt_viewer_geometry_data::BOX,
                      std::invalid_argument,
                      "LCM geometry data expected to be a BOX");
    DELPHYNE_VALIDATE(ign_box != nullptr, std::invalid_argument,
                      "Ignition box pointer must not be null");
    if (lcm_box.num_float_data != 3) {
      throw TranslateException(
          "Expected 3 float elements for box translation, but got " +
          std::to_string(lcm_box.num_float_data));
    }

    ignition::msgs::Vector3d* size = ign_box->mutable_size();
    size->set_x(lcm_box.float_data[0]);
    size->set_y(lcm_box.float_data[1]);
    size->set_z(lcm_box.float_data[2]);
  }

  // @brief Converts an LCM geometry to an ignition sphere geometry. The LCM
  // goemetry must represent a sphere geometry.
  //
  // @param[in] lcm_color The LCM geometry.
  // @param[out] ign_color The ign sphere geometry.
  static void LcmSphereToIgnition(
      const drake::lcmt_viewer_geometry_data& lcm_sphere,
      ignition::msgs::SphereGeom* ign_sphere) {
    DELPHYNE_VALIDATE(
        lcm_sphere.type == drake::lcmt_viewer_geometry_data::SPHERE,
        std::invalid_argument, "LCM geometry data expected to be a SPHERE");
    DELPHYNE_VALIDATE(ign_sphere != nullptr, std::invalid_argument,
                      "Ignition sphere pointer must not be null");
    if (lcm_sphere.num_float_data != 1) {
      throw TranslateException(
          "Expected 1 float element for sphere translation, but got " +
          std::to_string(lcm_sphere.num_float_data));
    }

    ign_sphere->set_radius(lcm_sphere.float_data[0]);
  }

  // @brief Converts an LCM geometry to an ignition cylinder geometry. The LCM
  // goemetry must represent a cylinder geometry.
  //
  // @param[in] lcm_color The LCM geometry.
  // @param[out] ign_color The ign cylinder geometry.
  static void LcmCylinderToIgnition(
      const drake::lcmt_viewer_geometry_data& lcm_cylinder,
      ignition::msgs::CylinderGeom* ign_cylinder) {
    DELPHYNE_VALIDATE(
        lcm_cylinder.type == drake::lcmt_viewer_geometry_data::CYLINDER,
        std::invalid_argument, "LCM geometry data expected to be a CYLINDER");
    DELPHYNE_VALIDATE(ign_cylinder != nullptr, std::invalid_argument,
                      "Ignition cylinder pointer must not be null");
    if (lcm_cylinder.num_float_data != 2) {
      throw TranslateException(
          "Expected 2 float elements for cylinder translation, but got " +
          std::to_string(lcm_cylinder.num_float_data));
    }

    ign_cylinder->set_radius(lcm_cylinder.float_data[0]);
    ign_cylinder->set_length(lcm_cylinder.float_data[1]);
  }

  // @brief Converts an LCM geometry to an ignition mesh geometry. The LCM
  // goemetry must represent a mesh geometry.
  //
  // @param[in] lcm_color The LCM geometry.
  // @param[out] ign_color The ign mesh geometry.
  static void LcmMeshToIgnition(
      const drake::lcmt_viewer_geometry_data& lcm_mesh,
      ignition::msgs::MeshGeom* ign_mesh) {
    DELPHYNE_VALIDATE(lcm_mesh.type == drake::lcmt_viewer_geometry_data::MESH,
                      std::invalid_argument,
                      "LCM geometry data expected to be a MESH");
    DELPHYNE_VALIDATE(ign_mesh != nullptr, std::invalid_argument,
                      "Ignition mesh pointer must not be null");
    if (lcm_mesh.string_data.empty()) {
      throw TranslateException("Expected a mesh filename for translation");
    }

    if (lcm_mesh.num_float_data != 3) {
      throw TranslateException(
          "Expected 3 float elements for mesh translation, but got " +
          std::to_string(lcm_mesh.num_float_data));
    }

    ign_mesh->set_filename(lcm_mesh.string_data);

    ignition::msgs::Vector3d* scale = ign_mesh->mutable_scale();
    scale->set_x(lcm_mesh.float_data[0]);
    scale->set_y(lcm_mesh.float_data[1]);
    scale->set_z(lcm_mesh.float_data[2]);
  }

 private:
  // The translator has a single input port, and a single output port.
  const int kPortIndex = 0;

  // @brief Calculates the state of the system's output port.
  //
  // @param[in] context The Drake system context.
  // @param[out] ign_message The ignition message that will be stored in the
  // output port.
  void CalcIgnMessage(const drake::systems::Context<double>& context,
                      IGN_TYPE* ign_message) const {
    DELPHYNE_VALIDATE(ign_message != nullptr, std::invalid_argument,
                      "Ignition message pointer must not be null");

    // Retrieves the Drake message from the input port.
    const DRAKE_TYPE& drake_message = ReadInputPort(context);

    // And then translates to ignition.
    auto time_ms = static_cast<int64_t>(context.get_time() * 1000);
    DoDrakeToIgnTranslation(drake_message, ign_message, time_ms);
  }

  // Depending on the type of DRAKE_TYPE (whether or not it inherits from
  // drake::systems::VectorBase), we need to read from a vector input port, or
  // from an abstract output port. The problem is those functions perform
  // assertions on the inferred return type, so we get compiler errors if a call
  // to the wrong function is compiled (e.g. a call to read a vector input port
  // into a non-vector object).
  // The solution to this issue is to use std::enable_if, which relies in SFINAE
  // to prevent compilation of ill-formed overloads.
  // When (if) we switch to C++17, all of this can be replaced with a simple
  // constexpr if.

  // @brief Reads an input port for Drake objects that inherit from VectorBase.
  template <class T = DRAKE_TYPE>
  typename std::enable_if<
      std::is_base_of<drake::systems::VectorBase<double>, T>::value,
      const DRAKE_TYPE&>::type
  ReadInputPort(const drake::systems::Context<double>& context) const {
    const drake::systems::VectorBase<double>* const input_vector =
        EvalVectorInput(context, kPortIndex);
    DELPHYNE_VALIDATE(input_vector != nullptr, std::runtime_error,
                      "Could not get vector from system");

    const DRAKE_TYPE* const vector =
        dynamic_cast<const DRAKE_TYPE*>(input_vector);

    return *vector;
  }

  // @brief Reads an input port for Drake objects that do not inherit from
  // VectorBase.
  template <class T = DRAKE_TYPE>
  typename std::enable_if<
      !std::is_base_of<drake::systems::VectorBase<double>, T>::value,
      const DRAKE_TYPE&>::type
  ReadInputPort(const drake::systems::Context<double>& context) const {
    return *this->template EvalInputValue<DRAKE_TYPE>(context, kPortIndex);
  }
};

}  // namespace delphyne
