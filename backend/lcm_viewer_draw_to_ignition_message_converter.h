// Copyright 2018 Open Source Robotics Foundation
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

#pragma once

#include <cstdint>
#include <string>

#include "drake/lcmt_viewer_draw.hpp"
#include "ignition/msgs.hh"

#include "backend/abstract_value_to_ignition_message_converter.h"
#include "backend/lcm_to_ign_translation.h"
#include "backend/system.h"

namespace delphyne {
namespace backend {

/// This class is a specialization of AbstractValueToIgnitionMessageConverter
/// that knows how to populate a Model_V ignition message from an
/// LCM view draw message.
class DELPHYNE_BACKEND_VISIBLE LCMViewerDrawToIgnitionMessageConverter
    : public AbstractValueToIgnitionMessageConverter<ignition::msgs::Model_V,
                                                     drake::lcmt_viewer_draw> {
 protected:
  void LcmToIgn(const drake::lcmt_viewer_draw& robotDrawData,
                ignition::msgs::Model_V* robotModels) override;

  void IgnToLcm(const ignition::msgs::Model_V& robotModels,
                drake::lcmt_viewer_draw* robotDrawData) override;

 private:
  static const int kPositionVectorSize{3};
  static const int kOrientationVectorSize{4};
};

}  // namespace backend
}  // namespace delphyne
