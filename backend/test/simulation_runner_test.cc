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
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <memory>
#include <mutex>
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

// @brief Checks that WaitForShutdown captures the SIGINT signal and the
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

// @brief Checks the time elapsed during the simulation
// step was at least as much as the defined kTimeStep.
TEST_F(SimulationRunnerTest, ElapsedTimeOnStep) {
  // Need to run a first step for the counters that handle real-time rate
  // to be initialized.
  sim_runner_->RunSimulationStep();

  // Run a step and record the delta time.
  auto step_start = std::chrono::steady_clock::now();

  sim_runner_->RunSimulationStep();

  auto step_end = std::chrono::steady_clock::now();

  const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      step_end - step_start);

  // Expected max/min duration in microseconds.
  std::chrono::microseconds min_simulation_time(9500);
  std::chrono::microseconds max_simulation_time(10500);

  EXPECT_GE(duration, min_simulation_time);
  EXPECT_LE(duration, max_simulation_time);
}

// @brief Asserts that an incoming message has
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
  node_.Request(service, robot_model_request_msg, timeout, response, result);

  EXPECT_TRUE(result);
  EXPECT_FALSE(callback_called_);

  sim_runner_->RunSimulationStep();

  EXPECT_TRUE(callback_called_);
}

// @brief Asserts that an incoming message is consumed
// from the queue even if the simulation is paused.
TEST_F(SimulationRunnerTest, ConsumedEventOnQueueWhenPaused) {
  sim_runner_->Start();
  sim_runner_->Pause();

  const std::string service_name{"test_service_name"};

  ignition::msgs::RobotModelRequest robot_model_request_msg;

  robot_model_request_msg.set_response_topic(service_name);

  AdvertiseRobotModelRequest(service_name);

  ignition::msgs::Boolean response;
  const unsigned int timeout = 100;
  bool result = false;
  const std::string service = "/get_robot_model";
  node_.Request(service, robot_model_request_msg, timeout, response, result);
  EXPECT_TRUE(result);

  // Wait until the currently running step of the loop finishes.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  EXPECT_TRUE(callback_called_);
}

// @brief Asserts that the Pause method
// pauses the advance of the simulation.
TEST_F(SimulationRunnerTest, TestPauseSetMethod) {
  sim_runner_->Start();

  EXPECT_FALSE(sim_runner_->IsPaused());

  sim_runner_->Pause();

  EXPECT_TRUE(sim_runner_->IsPaused());
}

// @brief Asserts that the Unpause method lets the
// simulator run again if it was previously paused.
TEST_F(SimulationRunnerTest, TestPauseResetMethod) {
  sim_runner_->Start();
  sim_runner_->Pause();

  EXPECT_TRUE(sim_runner_->IsPaused());

  sim_runner_->Unpause();

  EXPECT_FALSE(sim_runner_->IsPaused());

  // Calling the Unpause method when the simulation
  // is already unpaused leads to a no-op.
  sim_runner_->Unpause();

  EXPECT_FALSE(sim_runner_->IsPaused());
}

// @brief Asserts that the execution breaks if a RequestStep
// is received if the simulation hasn't started.
TEST_F(SimulationRunnerTest, TestRequestStepWhenNotStarted) {
  // We need this flag for safe multithreaded death tests
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_DEATH(sim_runner_->RequestStep(0.01), "condition 'enabled_' failed.");
}

// @brief Asserts that the execution breaks if a Start() is requested twice
TEST_F(SimulationRunnerTest, TestCantStartTwice) {
  // We need this flag for safe multithreaded death tests
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  sim_runner_->Start();
  EXPECT_DEATH(sim_runner_->Start(), "condition '!enabled_' failed.");
}

// @brief Asserts that the execution breaks if Stopt() is called and the
// simulation runner hasn't started yet.
TEST_F(SimulationRunnerTest, TestStopWithoutStartShouldFail) {
  // We need this flag for safe multithreaded death tests
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_DEATH(sim_runner_->Stop(), "condition 'enabled_' failed.");
}

// @brief Asserts that the execution breaks if a RequestStep
// is received if the simulation is paused.
TEST_F(SimulationRunnerTest, TestRequestStepWhenUnPaused) {
  // We need this flag for safe multithreaded death tests
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  sim_runner_->Start();

  EXPECT_FALSE(sim_runner_->IsPaused());

  EXPECT_DEATH(sim_runner_->RequestStep(0.1), "condition 'paused_' failed.");
}

// @brief Checks that RunSyncFor executes the simulation for the expected
// simulation time
TEST_F(SimulationRunnerTest, TestRunSyncFor) {
  // Make the simulation run twice as fast as wall clock.
  sim_runner_->SetRealtimeRate(2.0);

  sim_runner_->RunSyncFor(0.2);

  // Compare simulation time
  const auto elapsed_sim_time = sim_runner_->get_current_simulation_time();

  EXPECT_GE(elapsed_sim_time, 0.195);
  EXPECT_LE(elapsed_sim_time, 0.205);
}

// @brief Checks that RunSyncFor executes the simulation for the expected
// simulation time
TEST_F(SimulationRunnerTest, TestRunAsyncFor) {
  // Make the simulation run four times faster than wall clock.
  sim_runner_->SetRealtimeRate(4.0);

  // Variables requires to wait on the async callback
  bool callback_executed(false);
  std::mutex m;
  std::condition_variable cv;

  // Run the simulation and flag the condition variable when done.
  sim_runner_->RunAsyncFor(0.2, [&callback_executed, &m, &cv]() {
    std::unique_lock<std::mutex> lock(m);
    callback_executed = true;
    cv.notify_one();
  });

  std::unique_lock<std::mutex> lock(m);

  // While required to handle spurious wakeups
  while (!callback_executed) {
    cv.wait(lock);
  }

  // Compare simulation time
  const auto elapsed_sim_time = sim_runner_->get_current_simulation_time();

  EXPECT_GE(elapsed_sim_time, 0.195);
  EXPECT_LE(elapsed_sim_time, 0.205);
}

}  // namespace backend
}  // namespace delphyne
