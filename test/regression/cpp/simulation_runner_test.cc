// Copyright 2017 Toyota Research Institute

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "backend/automotive_simulator.h"
#include "backend/simulation_runner.h"
#include "delphyne/protobuf/scene_request.pb.h"

namespace delphyne {

class SimulationRunnerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto simulator = std::make_unique<delphyne::AutomotiveSimulator<double>>();
    sim_runner_ =
        std::make_unique<SimulatorRunner>(std::move(simulator), kTimeStep);
  }

  // Callback method for handlig SceneRequest service calls
  void SceneRequestCallback(const ignition::msgs::Scene& request) {
    callback_called_ = true;
  }

  // Advertises a service for a given service_name, with
  // the method SceneRequestCallback as callback
  void AdvertiseSceneRequest(std::string service_name) {
    node_.Advertise(service_name, &SimulationRunnerTest::SceneRequestCallback,
                    this);

    // Calling node_.Request() immediately after Advertise() sometimes causes
    // that call to hang: waiting fixes this.
    // TODO(nventuro): once ign-transport issue #84 is resolved, this should be
    // changed according to what comes out of that discussion.
    // https://bitbucket.org/ignitionrobotics/ign-transport/issues/84/hang-when-calling-noderequest
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
  // Run a step and record the delta time.
  auto step_start = std::chrono::steady_clock::now();

  sim_runner_->RunSyncFor(kTimeStep);

  auto step_end = std::chrono::steady_clock::now();

  const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      step_end - step_start);

  // Expected max/min duration in microseconds.
  std::chrono::microseconds min_simulation_time(9500);
  std::chrono::microseconds max_simulation_time(10500);

  InteractiveSimulationStats stats = sim_runner_->get_stats();

  EXPECT_GE(duration, min_simulation_time);
  EXPECT_LE(duration, max_simulation_time);
  EXPECT_EQ(1, stats.TotalExecutedSteps());
}

// @brief Asserts that an incoming message has
// been consumed from the incoming_msgs_ queue
TEST_F(SimulationRunnerTest, ConsumedEventOnQueue) {
  const std::string service_name{"test_service_name"};

  ignition::msgs::SceneRequest scene_request_msg;

  scene_request_msg.set_response_topic(service_name);

  AdvertiseSceneRequest(service_name);

  ignition::msgs::Boolean response;
  const unsigned int timeout = 100;
  bool result = false;
  node_.Request(SimulatorRunner::kSceneRequestServiceName, scene_request_msg,
                timeout, response, result);

  EXPECT_TRUE(result);
  EXPECT_FALSE(callback_called_);

  sim_runner_->RunSyncFor(kTimeStep);

  InteractiveSimulationStats stats = sim_runner_->get_stats();
  EXPECT_EQ(1, stats.TotalExecutedSteps());

  EXPECT_TRUE(callback_called_);
}

// @brief Asserts that an incoming message is consumed
// from the queue even if the simulation is paused.
TEST_F(SimulationRunnerTest, ConsumedEventOnQueueWhenPaused) {
  sim_runner_->Start();
  sim_runner_->PauseSimulation();

  const std::string service_name{"test_service_name"};

  ignition::msgs::SceneRequest scene_request_msg;

  scene_request_msg.set_response_topic(service_name);

  AdvertiseSceneRequest(service_name);

  ignition::msgs::Boolean response;
  const unsigned int timeout = 100;
  bool result = false;
  node_.Request(SimulatorRunner::kSceneRequestServiceName, scene_request_msg,
                timeout, response, result);
  EXPECT_TRUE(result);

  // Wait until the currently running step of the loop finishes.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  EXPECT_TRUE(callback_called_);
}

// @brief Asserts that the Pause method
// pauses the advance of the simulation.
TEST_F(SimulationRunnerTest, TestPauseSetMethod) {
  sim_runner_->Start();

  EXPECT_FALSE(sim_runner_->IsSimulationPaused());

  sim_runner_->PauseSimulation();

  EXPECT_TRUE(sim_runner_->IsSimulationPaused());
}

// @brief Asserts that the Unpause method lets the
// simulator run again if it was previously paused.
TEST_F(SimulationRunnerTest, TestPauseResetMethod) {
  sim_runner_->Start();
  sim_runner_->PauseSimulation();

  EXPECT_TRUE(sim_runner_->IsSimulationPaused());

  sim_runner_->UnpauseSimulation();

  EXPECT_FALSE(sim_runner_->IsSimulationPaused());
}

// @brief Asserts that the execution breaks if the runner is paused twice in
// a row
TEST_F(SimulationRunnerTest, TestCantPauseTwice) {
  // We need this flag for safe multithreaded death tests
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  sim_runner_->Start();
  sim_runner_->PauseSimulation();

  EXPECT_DEATH(sim_runner_->PauseSimulation(), "condition '!paused_' failed.");
}

// @brief Asserts that the execution breaks if the runner is unpaused twice in
// a row
TEST_F(SimulationRunnerTest, TestCantUnpauseTwice) {
  // We need this flag for safe multithreaded death tests
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  sim_runner_->Start();
  sim_runner_->PauseSimulation();
  sim_runner_->UnpauseSimulation();

  EXPECT_DEATH(sim_runner_->UnpauseSimulation(), "condition 'paused_' failed.");
}

// @brief Asserts that the execution breaks if a RequestSimulationStepExecution
// is received if the simulation hasn't started.
TEST_F(SimulationRunnerTest, TestRequestSimulationStepExecutionWhenNotStarted) {
  // We need this flag for safe multithreaded death tests
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_DEATH(sim_runner_->RequestSimulationStepExecution(1u),
               "condition 'interactive_loop_running_' failed.");
}

// @brief Asserts that the execution breaks if a Start() is requested twice
TEST_F(SimulationRunnerTest, TestCantStartTwice) {
  // We need this flag for safe multithreaded death tests
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  sim_runner_->Start();
  EXPECT_DEATH(sim_runner_->Start(),
               "condition '!interactive_loop_running_' failed.");
}

// @brief Asserts that the execution breaks if Stop() is called and the
// simulation runner hasn't started yet.
TEST_F(SimulationRunnerTest, TestStopWithoutStartShouldFail) {
  // We need this flag for safe multithreaded death tests
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_DEATH(sim_runner_->Stop(),
               "condition 'interactive_loop_running_' failed.");
}

// @brief Asserts that the execution breaks if a RequestSimulationStepExecution
// is received if the simulation is paused.
TEST_F(SimulationRunnerTest, TestRequestSimulationStepExecutionWhenUnPaused) {
  // We need this flag for safe multithreaded death tests
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  sim_runner_->Start();

  EXPECT_FALSE(sim_runner_->IsSimulationPaused());

  EXPECT_DEATH(sim_runner_->RequestSimulationStepExecution(1u),
               "condition 'paused_' failed.");
}

// @brief Checks that RunSyncFor executes the simulation for the expected
// simulation time
TEST_F(SimulationRunnerTest, TestRunSyncFor) {
  const double kDuration = 1.0;
  const double kRelativeTolerance = 0.025;

  // Make the simulation run twice as fast as wall clock.
  sim_runner_->SetRealtimeRate(2.0);

  sim_runner_->RunSyncFor(kDuration);

  // Compare simulation time
  const auto elapsed_sim_time = sim_runner_->get_current_simulation_time();

  EXPECT_GE(elapsed_sim_time, kDuration * (1. - kRelativeTolerance));
  EXPECT_LE(elapsed_sim_time, kDuration * (1. + kRelativeTolerance));
}

// @brief Checks that RunAsyncFor executes the simulation for the expected
// simulation time
TEST_F(SimulationRunnerTest, TestRunAsyncFor) {
  const double kDuration = 1.0;
  const double kRelativeTolerance = 0.025;

  // Make the simulation run four times faster than wall clock.
  sim_runner_->SetRealtimeRate(4.0);

  // Variables requires to wait on the async callback
  bool callback_executed(false);
  std::mutex m;
  std::condition_variable cv;

  // Run the simulation and flag the condition variable when done.
  sim_runner_->RunAsyncFor(kDuration, [&callback_executed, &m, &cv]() {
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

  EXPECT_GE(elapsed_sim_time, kDuration * (1. - kRelativeTolerance));
  EXPECT_LE(elapsed_sim_time, kDuration * (1. + kRelativeTolerance));
}

// @brief Checks that play/pause is properly handed on RunAsyncFor
TEST_F(SimulationRunnerTest, TestPlayPauseOnRunAsyncFor) {
  // Simulation time measured in seconds
  const double kSimulationDuration = 0.5;
  const double kSimulationRelativeTolerance = 0.05;

  // Wall clock measured in milliseconds
  const int kMinWallClockDuration = 600;
  const int kDelayForPause = 300;
  const int kMaxWallClockDuration =
      kMinWallClockDuration + kDelayForPause + kSimulationDuration * 1000;

  // Variables requires to wait on the async callback
  bool callback_executed(false);
  std::mutex m;
  std::condition_variable cv;

  // Record the wall-clock time when the simulation starts.
  const auto wall_clock_start = std::chrono::steady_clock::now();

  // Run the simulation and flag the condition variable when done.
  sim_runner_->RunAsyncFor(kSimulationDuration,
                           [&callback_executed, &m, &cv]() {
                             std::unique_lock<std::mutex> lock(m);
                             callback_executed = true;
                             cv.notify_one();
                           });

  // Wait for 50 milliseconds and pause the execution.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  sim_runner_->PauseSimulation();

  // Wait for 300 milliseconds and resume the execution.
  std::this_thread::sleep_for(std::chrono::milliseconds(kMinWallClockDuration));

  sim_runner_->UnpauseSimulation();

  // Wait for the callback.
  std::unique_lock<std::mutex> lock(m);

  // While required to handle spurious wakeups.
  while (!callback_executed) {
    cv.wait(lock);
  }

  // Record the wall-clock time when the simulation ends.
  const auto wall_clock_end = std::chrono::steady_clock::now();

  // Compare wall clock time.
  const auto wall_clock_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(wall_clock_end -
                                                            wall_clock_start);

  // Expected max/min duration in milliseconds.
  const std::chrono::milliseconds min_wall_clock_time(kMinWallClockDuration);
  const std::chrono::milliseconds max_wall_clock_time(kMaxWallClockDuration);

  EXPECT_GE(wall_clock_duration, min_wall_clock_time);
  EXPECT_LE(wall_clock_duration, max_wall_clock_time);

  // Compare simulation time
  const auto elapsed_sim_time = sim_runner_->get_current_simulation_time();

  EXPECT_GE(elapsed_sim_time,
            kSimulationDuration * (1. - kSimulationRelativeTolerance));
  EXPECT_LE(elapsed_sim_time,
            kSimulationDuration * (1. + kSimulationRelativeTolerance));
}

}  // namespace delphyne
