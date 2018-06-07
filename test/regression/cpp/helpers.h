// Copyright 2017 Toyota Research Institute

#pragma once

#include <drake/lcmt_viewer_draw.hpp>
#include <drake/lcmt_viewer_load_robot.hpp>
#include <drake/systems/rendering/pose_bundle.h>

#include "gtest/gtest.h"

#include "google/protobuf/message.h"

#include "ignition/msgs.hh"

namespace delphyne {
namespace test {

// Generates a pre-loaded lcmt_viewer_draw message.
//
// @return a loaded lcmt_viewer_draw message.
drake::lcmt_viewer_draw BuildPreloadedDrawMsg();

// Generates a pre-loaded lcmt_viewer_load_robot message.
//
// @return a loaded lcmt_viewer_load_robot message.
drake::lcmt_viewer_load_robot BuildPreloadedLoadRobotMsg();

// Generates a pre-loaded Model_V message.
//
// @return a loaded Model_V message.
ignition::msgs::Model_V BuildPreloadedModelVMsg();

// Generates a pre-loaded pose bundle with model poses
// (@see BuildPreloadedModelVMsg).
//
// @return a drake::systems::rendering::PoseBundle<double> instance.
drake::systems::rendering::PoseBundle<double> BuildPreloadedPoseBundle();

// Asserts that all the array-iterable values from
// lcm_msg match the content of the ign_models object.
//
// @param lcm_msg An lcm viewer draw message with the desired values.
// @param ign_models An ignition messages Model_V with the translated values.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckMsgTranslation(
    const drake::lcmt_viewer_draw& lcm_msg,
    const ignition::msgs::Model_V& ign_models);

// Asserts that all the array-iterable values from lcm_msg match the content of
// the ign_models object.
//
// @param lcm_msg An lcm viewer load robot message with the desired values.
// @param ign_models An ignition messages Model_V with the translated values.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckMsgTranslation(
    const drake::lcmt_viewer_load_robot& lcm_msg,
    const ignition::msgs::Model_V& ign_models);

// Asserts that all the array-iterable values from
// lcm_msg match the content of the scene object.
//
// @param lcm_msg An lcm viewer draw message with the desired values.
// @param ign_models An ignition messages Scene with the translated values.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckMsgTranslation(
    const drake::lcmt_viewer_draw& lcm_msg, const ignition::msgs::Scene& scene);

// Asserts that the position values found on an array provenient from an
// lcm message are equivalent to those found on the ignition object.
//
// @param lcm_position An array of length 3 with the lcm position values.
// @param ign_position An ignition message of type Vector3d with a position
// value.
// @param tolerance A double containing the translation's tolerance.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckLcmArrayToVector3dEquivalence(
    const float lcm_position[], const ignition::msgs::Vector3d& ign_position,
    double tolerance);

// Asserts that the quaternion values found on an array provenient from an
// lcm message are equivalent to those found on the ignition object.
//
// @param lcm_orientation An array of length 4 with the lcm orientation values.
// @param ign_orientation An ignition messages of type Quaternion.
// @param tolerance A double containing the translation's tolerance.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckLcmArrayToQuaternionEquivalence(
    const float lcm_orientation[],
    const ignition::msgs::Quaternion& ign_orientation, double tolerance);

// Asserts that the color values found on an array provenient from an lcm
// message are equivalent to those found on the ignition object.
//
// @param lcm_color An array of length 4 with the lcm color values.
// @param ign_color An ignition messages of type Color.
// @param tolerance A double containing the translation's tolerance.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckLcmArrayToColorEquivalence(
    const float lcm_color[], const ignition::msgs::Color& ign_color,
    double tolerance);

// Asserts the equivalence between an lcm geometry type versus an
// ignition messages geometry type.
//
// @param lcm_geometry_type An integer that translates into an lcm
// geometry type.
// @param ign_orientation An ignition geometry type value.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckGeometryTypeEquivalence(
    int8_t lcm_geometry_type, ignition::msgs::Geometry::Type ign_geometry_type);

// Asserts that two protobuf messages are equal.
//
// @param lhs A protobuf messsage.
// @param rhs A second protobuf messsage to compare against.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckProtobufMsgEquality(
    const google::protobuf::MessageLite& lhs,
    const google::protobuf::MessageLite& rhs);

}  // namespace test
}  // namespace delphyne
