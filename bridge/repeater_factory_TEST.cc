// Copyright 2017 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include <ignition/transport.hh>
#include <lcm/lcm-cpp.hpp>

#include "drake/lcmt_driving_command_t.hpp"
#include "protobuf/automotive_driving_command.pb.h"
#include "bridge/repeater_factory.h"

namespace delphyne {
namespace bridge {

std::shared_ptr<delphyne::bridge::AbstractRepeater> fakeFactoryFunction(
    std::shared_ptr<lcm::LCM> lcm, const std::string& topicName) {
  return std::make_shared<delphyne::bridge::IgnTopicRepeater<
      ignition::msgs::AutomotiveDrivingCommand, drake::lcmt_driving_command_t>>(
      lcm, topicName);
}

//////////////////////////////////////////////////
/// \brief Test that nullptr is returned if the requested type has not been
/// previously registered.
GTEST_TEST(RepeaterFactory, TestNewFailsOnNoAvailableMapping) {
  // Setup LCM
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  ASSERT_EQ(nullptr, RepeaterFactory::New("nonExistentType", lcm));
}

//////////////////////////////////////////////////
/// \brief Test that a repeater is created if the type has been previously
/// registered.
GTEST_TEST(RepeaterFactory, TestNewCreatesRepeater) {
  // Setup LCM
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  RepeaterFactory::Register("fakeType", fakeFactoryFunction);

  ASSERT_NE(nullptr, RepeaterFactory::New("fakeType", lcm));
}

//////////////////////////////////////////////////
/// \brief Test that a different instances are created if new is called multiple
/// times
GTEST_TEST(RepeaterFactory, TestNewCreatesMultipleRepeaters) {
  // Setup LCM
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  RepeaterFactory::Register("fakeType", fakeFactoryFunction);

  auto repeater1 = RepeaterFactory::New("fakeType", lcm);
  auto repeater2 = RepeaterFactory::New("fakeType", lcm);

  ASSERT_NE(nullptr, repeater1);
  ASSERT_NE(nullptr, repeater2);
  ASSERT_NE(repeater1, repeater2);
}

}  // namespace bridge
}  // namespace delphyne
