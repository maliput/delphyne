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


#include <csignal>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "backend/simulation_runner.h"
#include "backend/automotive_simulator.h"

namespace delphyne {
namespace backend {

//////////////////////////////////////////////////
// \brief Check that WaitForShutdown captures the SIGINT signal and the
// simulation terminates gracefully.
TEST(SimulationRunnerTest, sigIntTermination) {
  // Instantiate a simulator.
  auto simulator =
      std::make_unique<delphyne::backend::AutomotiveSimulator<double>>();

  // Instantiate the simulator runner and pass the simulator.
  const double kTimeStep = 0.001;
  SimulatorRunner simRunner(std::move(simulator), kTimeStep);
  simRunner.Start();

  std::thread t([]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::raise(SIGINT);
  });

  // Zzzzzz.
  WaitForShutdown();

  if (t.joinable()) t.join();
}

//////////////////////////////////////////////////
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace backend
}  // namespace delphyne
