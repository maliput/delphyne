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

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "backend/automotive_simulator.h"
#include "backend/simulation_runner.h"

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <protobuf/robot_model_request.pb.h>

namespace delphyne {
namespace backend {

class SimulationRunnerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto simulator =
      std::make_unique<delphyne::backend::AutomotiveSimulator<double>>();
    sim_runner_ =
        std::make_unique<SimulatorRunner>(std::move(simulator), kTimeStep);
  }

  // Callback method for handlig RobotModelRequest service calls
  void RobotModelRequestCallback(const ignition::msgs::Model_V& request) {
    callback_called_ = true;
  }

  // Advertises a service for a given service_name, with
  // the method RobotModelRequestCallback as callback
  void AdvertiseRobotModelRequest(std::string service_name) {
    node_.Advertise(service_name,
                    &SimulationRunnerTest::RobotModelRequestCallback, this);
  }

  const double kTimeStep{0.01};  // 10 millis

  bool callback_called_{false};

  std::unique_ptr<SimulatorRunner> sim_runner_;

  ignition::transport::Node node_;
};

// \brief Checks that WaitForShutdown captures the SIGINT signal and the
// simulation terminates gracefully.
TEST_F(SimulationRunnerTest, SigIntTermination) {
  sim_runner_->Start();

  std::thread t([]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::raise(SIGINT);
  });

  // Zzzzzz.
  WaitForShutdown();

  if (t.joinable()) t.join();
}

// \brief Checks the time elapsed during the simulation
// step was at least as much as the defined kTimeStep.
TEST_F(SimulationRunnerTest, ElapsedTimeOnStep) {
  auto step_start = std::chrono::steady_clock::now();
  sim_runner_->RunSimulationStep();
  auto step_end = std::chrono::steady_clock::now();

  // Calculates duration in milliseconds.
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                      step_end - step_start);

  std::chrono::milliseconds min_simulation_time(10);

  EXPECT_GE(duration, min_simulation_time);
}

// \brief Verifies that an incoming message has
// been consumed from the incoming_msgs_ queue
TEST_F(SimulationRunnerTest, ConsumedEventOnQueue) {
  const std::string service_name{"test_service_name"};

  ignition::msgs::RobotModelRequest robot_model_request_msg;

  robot_model_request_msg.set_response_topic(service_name);

  AdvertiseRobotModelRequest(service_name);

  ignition::msgs::Boolean response;
  const unsigned int timeout = 100;
  bool result = false;
  const std::string service = "/get_robot_model";
  node_.Request(service, robot_model_request_msg, timeout, response,
                result);

  EXPECT_TRUE(result);
  EXPECT_FALSE(callback_called_);

  sim_runner_->RunSimulationStep();

  EXPECT_TRUE(callback_called_);
}

}  // namespace backend
}  // namespace delphyne
