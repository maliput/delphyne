// Copyright 2018 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include "drake/lcmt_viewer_draw.hpp"

#include "gtest/gtest.h"

#include "google/protobuf/message.h"

#include "ignition/msgs.hh"

#include "backend/system.h"

namespace delphyne {
namespace test {

// Generates a pre-loaded lcmt_viewer_draw message.
//
// @return a loaded lcmt_viewer_draw message.
DELPHYNE_BACKEND_VISIBLE drake::lcmt_viewer_draw BuildPreloadedDrawMsg();

// Generates a pre-loaded Model_V message.
//
// @return a loaded Model_V message.
DELPHYNE_BACKEND_VISIBLE ignition::msgs::Model_V BuildPreloadedModelVMsg();

// Asserts that all the array-iterable values from
// lcm_msg matches the content of the ign_models object.
//
// @param lcm_msg An lcm viewer draw message with the desired values.
// @param ign_models An ignition messages Model_V with the translated values.
// @return a google test's AssertionResult.
DELPHYNE_BACKEND_VISIBLE::testing::AssertionResult CheckMsgTranslation(
    const drake::lcmt_viewer_draw& lcm_msg,
    const ignition::msgs::Model_V& ign_models);

// Asserts that all the array-iterable values from
// lcm_msg matches the content of the scene object.
//
// @param lcm_msg An lcm viewer draw message with the desired values.
// @param ign_models An ignition messages Scene with the translated values.
// @return a google test's AssertionResult.
DELPHYNE_BACKEND_VISIBLE::testing::AssertionResult CheckMsgTranslation(
    const drake::lcmt_viewer_draw& lcm_msg, const ignition::msgs::Scene& scene);

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

DELPHYNE_BACKEND_VISIBLE::testing::AssertionResult CheckProtobufMsgEquality(
    const google::protobuf::MessageLite& lhs,
    const google::protobuf::MessageLite& rhs);

}  // namespace test
}  // namespace delphyne
