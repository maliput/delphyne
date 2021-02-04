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

#include "agents/simple_car.h"
#include "backend/agent_simulation_builder.h"
#include "backend/simulation_runner.h"
#include "delphyne/macros.h"
#include "delphyne/mi6/agent_simulation.h"
#include "test_utilities/helpers.h"

namespace delphyne {

class SimulationRunnerTest : public test::TestWithFiles {
 protected:
  void SetUp() override {
    AgentSimulationBuilder builder;
    builder.AddAgent<SimpleCarBlueprint>("moving_car", kCarX, kCarY, kCarHeading, kCarSpeed);
    builder.AddAgent<SimpleCarBlueprint>("stopped_car", kCarX + kCarsDistance, kCarY, kCarHeading, kCarSpeed);
    sim_runner_ = std::make_unique<SimulationRunner>(builder.Build(), kTimeStep);
    // Set environmental variable to define the logfile path
    setenv("DELPHYNE_LOGS_PATH", "/tmp/XXXXXX", 1);
  }

  void TearDown() override { unsetenv("DELPHYNE_LOGS_PATH"); }

  const double kTimeStep{0.01};     // 10 millis
  const double kCarX{0.};           // 0 meters
  const double kCarY{0.};           // 0 meters
  const double kCarsDistance{50.};  // 50 meters
  const double kCarHeading{0.};     // 0 degrees
  const double kCarSpeed{10.};      // 10 meters per sec

  std::unique_ptr<SimulationRunner> sim_runner_;

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

  const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(step_end - step_start);

  // Expected max/min duration in microseconds.
  std::chrono::microseconds min_simulation_time(9500);
  std::chrono::microseconds max_simulation_time(10500);

  const InteractiveSimulationStats& stats = sim_runner_->GetStats();

  EXPECT_GE(duration, min_simulation_time);
  EXPECT_LE(duration, max_simulation_time);
  EXPECT_EQ(1, stats.TotalExecutedSteps());
}

// @brief Asserts that an incoming message has
// been consumed from the incoming_msgs_ queue
TEST_F(SimulationRunnerTest, ConsumedEventOnQueue) {
  ignition::msgs::WorldControl world_control_msg;
  world_control_msg.set_pause(true);

  ignition::msgs::Boolean response;
  const unsigned int timeout = 100;
  bool result = false;
  node_.Request(SimulationRunner::kControlService, world_control_msg, timeout, response, result);

  EXPECT_TRUE(result);

  // tell it to run for 1 step and wait, but it won't take any steps because it's paused
  sim_runner_->RunAsyncFor(kTimeStep, [] {});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // expect no executed steps and that it is paused
  const InteractiveSimulationStats& stats = sim_runner_->GetStats();
  EXPECT_EQ(0, stats.TotalExecutedSteps());
  EXPECT_TRUE(sim_runner_->IsSimulationPaused());

  // explicitly call Stop to end the RunAsyncFor thread
  sim_runner_->Stop();
}

// @brief Asserts that an incoming message is consumed
// from the queue even if the simulation is paused.
TEST_F(SimulationRunnerTest, ConsumedEventOnQueueWhenPaused) {
  sim_runner_->Start();
  sim_runner_->PauseSimulation();

  // unpause with a WorldControl message
  ignition::msgs::WorldControl world_control_msg;
  world_control_msg.set_pause(false);

  ignition::msgs::Boolean response;
  const unsigned int timeout = 100;
  bool result = false;
  node_.Request(SimulationRunner::kControlService, world_control_msg, timeout, response, result);
  EXPECT_TRUE(result);

  // Wait until the currently running step of the loop finishes.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  const InteractiveSimulationStats& stats = sim_runner_->GetStats();
  EXPECT_EQ(1, stats.TotalExecutedSteps());
  EXPECT_FALSE(sim_runner_->IsSimulationPaused());
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

TEST_F(SimulationRunnerTest, TestPauseOnCollision) {
  sim_runner_->EnableCollisions();
  sim_runner_->AddCollisionCallback([this](auto) {
    EXPECT_TRUE(sim_runner_->IsSimulationPaused());
    sim_runner_->Stop();
  });
  EXPECT_FALSE(sim_runner_->IsSimulationPaused());
  sim_runner_->RunSyncFor(kCarsDistance / kCarSpeed);
}

// @brief Asserts that the execution breaks if the runner is paused twice in
// a row
TEST_F(SimulationRunnerTest, TestCantPauseTwice) {
  sim_runner_->Start();
  sim_runner_->PauseSimulation();

  EXPECT_RUNTIME_THROW(sim_runner_->PauseSimulation(), "Cannot pause already paused simulation");
}

// @brief Asserts that the execution breaks if the runner is unpaused twice in
// a row
TEST_F(SimulationRunnerTest, TestCantUnpauseTwice) {
  sim_runner_->Start();
  sim_runner_->PauseSimulation();
  sim_runner_->UnpauseSimulation();

  EXPECT_RUNTIME_THROW(sim_runner_->UnpauseSimulation(), "Cannot unpause already running simulation");
}

// @brief Asserts that the execution breaks if a RequestSimulationStepExecution
// is received if the simulation hasn't started.
TEST_F(SimulationRunnerTest, TestRequestSimulationStepExecutionWhenNotStarted) {
  EXPECT_RUNTIME_THROW(sim_runner_->RequestSimulationStepExecution(1u),
                       "Cannot step a simulation that is not yet running");
}

// @brief Asserts that the execution breaks if a Start() is requested twice
TEST_F(SimulationRunnerTest, TestCantStartTwice) {
  sim_runner_->Start();
  EXPECT_RUNTIME_THROW(sim_runner_->Start(), "Cannot run a simulation that is already running");
}

// @brief Asserts that the execution breaks if Stop() is called and the
// simulation runner hasn't started yet.
TEST_F(SimulationRunnerTest, TestStopWithoutStartShouldFail) {
  EXPECT_RUNTIME_THROW(sim_runner_->Stop(), "Cannot stop a simulation that is not running");
}

// @brief Asserts that the execution breaks if a RequestSimulationStepExecution
// is received if the simulation is paused.
TEST_F(SimulationRunnerTest, TestRequestSimulationStepExecutionWhenUnPaused) {
  sim_runner_->Start();

  EXPECT_FALSE(sim_runner_->IsSimulationPaused());

  EXPECT_RUNTIME_THROW(sim_runner_->RequestSimulationStepExecution(1u), "Cannot step a simulation that is not paused");
}

// @brief Asserts that logs can be sent to a custom destination path.
TEST_F(SimulationRunnerTest, TestLoggingToCustomPath) {
  sim_runner_->Start();

  // Simulation should not log by default.
  EXPECT_FALSE(sim_runner_->IsLogging());

  // Start logging at temporary full path.
  const std::string filename = test::MakeTemporaryDirectory("/tmp/XXXXXX") + "/test.log";
  sim_runner_->StartLogging(filename);

  // Checks that logging has been started.
  EXPECT_TRUE(sim_runner_->IsLogging());

  // Checks that the right path is being used for logging.
  EXPECT_EQ(sim_runner_->GetLogFilename(), filename);

  sim_runner_->StopLogging();

  // Checks that logging has been stopped.
  EXPECT_FALSE(sim_runner_->IsLogging());
}

// @brief Asserts that logs can be saved with a default filename if it's not
// explicitly provided or defined with an envvar.
TEST_F(SimulationRunnerTest, TestLoggingForNoGivenFilename) {
  sim_runner_->Start();

  // Simulation should not log by default.
  EXPECT_FALSE(sim_runner_->IsLogging());

  // Pass an empty string as filename so that system recognizes it as invalid
  // and sets the default name instead.
  sim_runner_->StartLogging("");

  // Checks that logging has been started.
  EXPECT_TRUE(sim_runner_->IsLogging());

  // Store full path to logfile before stopping.
  const std::string filepath{sim_runner_->GetLogFilename()};

  sim_runner_->StopLogging();

  // Checks that logging has been stopped.
  EXPECT_FALSE(sim_runner_->IsLogging());

  // Checks that the logfile exists in memory.
  EXPECT_TRUE(ignition::common::exists(filepath));
}

// @brief Asserts that simulation does not log by default, and logging can be
// started and stopped.
TEST_F(SimulationRunnerTest, TestStartStopLogging) {
  sim_runner_->Start();

  // Simulation should not log by default.
  EXPECT_FALSE(sim_runner_->IsLogging());

  sim_runner_->StartLogging();

  // Simulation should now be logging.
  EXPECT_TRUE(sim_runner_->IsLogging());

  EXPECT_NE(std::string::npos, sim_runner_->GetLogFilename().find("/tmp/XXXXXX/logs"));

  sim_runner_->StopLogging();

  // Simulation should no longer be logging.
  EXPECT_FALSE(sim_runner_->IsLogging());

  EXPECT_TRUE(sim_runner_->GetLogFilename().empty());

  sim_runner_->StartLogging();

  // Simulation should now be logging.
  EXPECT_TRUE(sim_runner_->IsLogging());

  EXPECT_NE(std::string::npos, sim_runner_->GetLogFilename().find("/tmp/XXXXXX/logs"));
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
  const auto elapsed_sim_time = sim_runner_->GetCurrentSimulationTime();

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
  const auto elapsed_sim_time = sim_runner_->GetCurrentSimulationTime();

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
  const int kMaxWallClockDuration = kMinWallClockDuration + kDelayForPause + kSimulationDuration * 1000;

  // Variables requires to wait on the async callback
  bool callback_executed(false);
  std::mutex m;
  std::condition_variable cv;

  // Record the wall-clock time when the simulation starts.
  const auto wall_clock_start = std::chrono::steady_clock::now();

  // Run the simulation and flag the condition variable when done.
  sim_runner_->RunAsyncFor(kSimulationDuration, [&callback_executed, &m, &cv]() {
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
      std::chrono::duration_cast<std::chrono::milliseconds>(wall_clock_end - wall_clock_start);

  // Expected max/min duration in milliseconds.
  const std::chrono::milliseconds min_wall_clock_time(kMinWallClockDuration);
  const std::chrono::milliseconds max_wall_clock_time(kMaxWallClockDuration);

  EXPECT_GE(wall_clock_duration, min_wall_clock_time);
  EXPECT_LE(wall_clock_duration, max_wall_clock_time);

  // Compare simulation time
  const auto elapsed_sim_time = sim_runner_->GetCurrentSimulationTime();

  EXPECT_GE(elapsed_sim_time, kSimulationDuration * (1. - kSimulationRelativeTolerance));
  EXPECT_LE(elapsed_sim_time, kSimulationDuration * (1. + kSimulationRelativeTolerance));
}

}  // namespace delphyne
