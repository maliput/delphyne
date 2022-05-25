// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <drake/systems/framework/framework_common.h>
#include <gtest/gtest.h>

#include "test_utilities/helpers.h"
#include "translations/lcm_viewer_draw_to_ign_model_v.h"

namespace delphyne {

// @brief Checks that an LCM viewer draw message on the input port is correctly
// translated into an ignition Model_V message.
GTEST_TEST(LCMViewerDrawToIgnModelVTranslatorSystemTest, TestTranslation) {
  const drake::lcmt_viewer_draw lcm_msg{test::BuildPreloadedDrawMsg()};

  const LcmViewerDrawToIgnModelV translator;
  std::unique_ptr<drake::systems::Context<double>> context = translator.AllocateContext();
  const int kPortIndex{0};
  context->FixInputPort(kPortIndex, drake::Value<drake::lcmt_viewer_draw>(lcm_msg));

  std::unique_ptr<drake::systems::SystemOutput<double>> output = translator.AllocateOutput();
  translator.CalcOutput(*context, output.get());

  const auto& ign_msg = output->get_data(kPortIndex)->get_value<ignition::msgs::Model_V>();

  EXPECT_TRUE(test::CheckMsgTranslation(lcm_msg, ign_msg));
}

}  // namespace delphyne
