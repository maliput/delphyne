#include <gtest/gtest.h>
#include <ignition/transport.hh>
#include <lcm/lcm-cpp.hpp>

#include "drake/lcmt_driving_command_t.hpp"
#include "protobuf/automotive_driving_command.pb.h"
#include "bridge/repeater_factory.hh"

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
