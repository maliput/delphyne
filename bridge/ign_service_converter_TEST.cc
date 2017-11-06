#include <iostream>
#include <gtest/gtest.h>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "drake/lcmt_viewer_command.hpp"
#include "bridge/ign_service_converter.hh"

// LCM entry point
#include "lcm/lcm-cpp.hpp"

namespace delphyne {
namespace bridge {

class LCMHandler {
 public:
  ~LCMHandler() {}
  int handlerCounter = 0;
  drake::lcmt_viewer_command responseMsg;

  void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                     const drake::lcmt_viewer_command* msg) {
    handlerCounter++;
    responseMsg = *msg;
  }
};

//////////////////////////////////////////////////
/// \brief Test that an LCM draw message is properly
/// translated to an ignition Geometry message.
class ServiceConverterTest : public ::testing::Test {
 protected:
  // Create an ignition transport node.
  ignition::transport::Node node;
  // Create empty request and response messages for ignition service
  ignition::msgs::Empty serviceRequest;
  ignition::msgs::Boolean serviceResponse;
  bool serviceResult;
  unsigned int serviceTimeout = 100;
  // Ignition service name
  std::string notifierServiceName = "/test_service";
  std::string lcmChannelName = "DRAKE_VIEWER_STATUS";

  // Create an lcm instance
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
  // Subscribe lcm handler to channel
  LCMHandler handlerObject;

  virtual void SetUp() override {
    // Subscribe to LCM channel
    lcm->subscribe(lcmChannelName, &LCMHandler::handleMessage, &handlerObject);
  }
};

//////////////////////////////////////////////////
/// \brief Test an end to end service to lcm msg conversion
TEST_F(ServiceConverterTest, TestConversionEndToEnd) {
  // Create service to channel converter instance
  delphyne::bridge::IgnitionServiceConverter<ignition::msgs::Empty,
                                             drake::lcmt_viewer_command>
      serviceConverter(lcm, notifierServiceName, lcmChannelName);

  // Start ignition service to lcm channel converter
  serviceConverter.Start();

  // Request the republisher service.
  node.Request(notifierServiceName, serviceRequest, serviceTimeout,
               serviceResponse, serviceResult);

  // Wait for lcm message up to 100 millis before timeout
  lcm->handleTimeout(100);

  // Check handler has been called once
  ASSERT_EQ(1, handlerObject.handlerCounter);
  // Check msg content
  EXPECT_EQ(0, handlerObject.responseMsg.command_type);
  EXPECT_EQ("successfully loaded robot",
            handlerObject.responseMsg.command_data);
}

//////////////////////////////////////////////////
/// \brief Test convertion with multiple requests of the ignition service
TEST_F(ServiceConverterTest, TestConversionCalledMultipleTimes) {
  // Create service to channel converter instance
  delphyne::bridge::IgnitionServiceConverter<ignition::msgs::Empty,
                                             drake::lcmt_viewer_command>
      serviceConverter(lcm, notifierServiceName, lcmChannelName);

  // Start ignition service to lcm channel converter
  serviceConverter.Start();

  // Request the republisher service.
  for (int i = 0; i < 3; i++) {
    node.Request(notifierServiceName, serviceRequest, serviceTimeout,
                 serviceResponse, serviceResult);
    lcm->handleTimeout(100);
  }

  // Check handler has been called three times
  ASSERT_EQ(3, handlerObject.handlerCounter);
}

//////////////////////////////////////////////////
/// \brief Test failure if converter hasn't been started
TEST_F(ServiceConverterTest, TestConversionNotStarted) {
  // Create service to channel converter instance
  delphyne::bridge::IgnitionServiceConverter<ignition::msgs::Empty,
                                             drake::lcmt_viewer_command>
      serviceConverter(lcm, notifierServiceName, lcmChannelName);

  // Request the republisher service.
  node.Request(notifierServiceName, serviceRequest, serviceTimeout,
               serviceResponse, serviceResult);
  lcm->handleTimeout(100);

  // Check handler hasn't been called
  ASSERT_EQ(0, handlerObject.handlerCounter);
}

}  // namespace bridge
}  // namespace delphyne
