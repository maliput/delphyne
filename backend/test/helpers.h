// Copyright 2017 Toyota Research Institute

#pragma once

#include <drake/lcmt_viewer_draw.hpp>

#include <gtest/gtest.h>

#include <ignition/msgs.hh>

namespace delphyne {
namespace test {

// Generates a pre-loaded lcmt_viewer_draw message.
//
// @return a loaded lcmt_viewer_draw message.
__attribute__((visibility("default"))) drake::lcmt_viewer_draw
BuildPreloadedDrawMsg();

// Asserts that all the array-iterable values from
// lcm_msg matches the content of the ign_models object.
//
// @param lcm_msg An lcm viewer draw message with the desired values.
// @param ign_models An ignition messages Model_V with the translated values.
// @return a google test's AssertionResult.
__attribute__((visibility("default")))::testing::AssertionResult
CheckMsgTranslation(const drake::lcmt_viewer_draw& lcm_msg,
                    const ignition::msgs::Model_V& ign_models);

// Asserts that all the array-iterable values from
// lcm_msg matches the content of the scene object.
//
// @param lcm_msg An lcm viewer draw message with the desired values.
// @param ign_models An ignition messages Scene with the translated values.
// @return a google test's AssertionResult.
__attribute__((visibility("default")))::testing::AssertionResult
CheckMsgTranslation(const drake::lcmt_viewer_draw& lcm_msg,
                    const ignition::msgs::Scene& scene);

// Asserts that the position values found on the original lcm message are
// equivalent to the values found on the translated ignition object.
//
// @param pose An ignition messages pose object with the translated values.
// @param lcm_msg An lcm viewer draw message with the desired values.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckPositionTranslation(
    const ignition::msgs::Pose& pose, const drake::lcmt_viewer_draw& lcm_msg,
    int lcm_index, double tolerance);

// Asserts that the position values found on the original lcm message are
// equivalent to the values found on the translated ignition object.
//
// @param pose An ignition messages pose object with the translated values.
// @param lcm_msg An lcm viewer draw message with the desired values.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckOrientationTranslation(
    const ignition::msgs::Pose& pose, const drake::lcmt_viewer_draw& lcm_msg,
    int lcm_index, double tolerance);

}  // namespace test
}  // namespace delphyne
