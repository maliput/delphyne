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
#include "delphyne/macros.h"
#include "delphyne/protobuf/scene_request.pb.h"

namespace delphyne {

class LoggerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto simulator = std::make_unique<delphyne::AutomotiveSimulator<double>>();
    sim_runner_ =
        std::make_unique<SimulatorRunner>(std::move(simulator), kTimeStep);
  }

  const double kTimeStep{0.01};  // 10 millis

  bool callback_called_{false};

  std::unique_ptr<SimulatorRunner> sim_runner_;

  ignition::transport::Node node_;
};

// @brief Asserts that simulation does not log by default, and logging can be
// started and stopped.
TEST_F(LoggerTest, TestStartStopLogging) {
  sim_runner_->Start();

  // Simulation should not log by default.
  EXPECT_FALSE(sim_runner_->IsLogging());

  sim_runner_->StartLogging();

  // Simulation should now be logging.
  EXPECT_TRUE(sim_runner_->IsLogging());

  EXPECT_NE(std::string::npos,
    sim_runner_->GetLogFilename().find(".delphyne/logs"));

  sim_runner_->StopLogging();

  // Simulation should no longer be logging.
  EXPECT_FALSE(sim_runner_->IsLogging());

  EXPECT_TRUE(sim_runner_->GetLogFilename().empty());

  sim_runner_->StartLogging();

  // Simulation should now be logging.
  EXPECT_TRUE(sim_runner_->IsLogging());

  std::time_t now = std::time(nullptr);
  std::tm tm = *std::localtime(&now);
  std::stringstream logPath;
  logPath << ".delphyne/logs/" << std::put_time(&tm, "%FT%H%M");

  EXPECT_NE(std::string::npos,
    sim_runner_->GetLogFilename().find(logPath.str()));
}
}  // namespace delphyne
