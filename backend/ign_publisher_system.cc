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

#include "backend/ign_publisher_system.h"

#include <iostream>

namespace delphyne {
namespace backend {

IgnPublisherSystem::IgnPublisherSystem() {
  this->DeclareAbstractInputPort();
  publisher_ = node_.Advertise<ignition::msgs::Model_V>(topic_);
}

IgnPublisherSystem::~IgnPublisherSystem() {}

void IgnPublisherSystem::DoPublish(
    const Context<double>& context,
    const std::vector<const PublishEvent<double>*>&) const {
  const AbstractValue* input = this->EvalAbstractInput(context, 0);
  const auto& viewer_draw = input->GetValue<drake::lcmt_viewer_draw>();
  ignition::msgs::Model_V ignMsg;

  // Translate the lcm message into an ignition-transport message
  delphyne::bridge::lcmToIgn(viewer_draw, &ignMsg);

  // TODO(apojomovsky): we might want replace this with a cleaner approach
  // Creates a local copy of the publisher, since an instance
  // variable cannot be modified from inside a const method
  auto tempPublisher = publisher_;

  // Publishes onto the specified ign-transport channel.
  tempPublisher.Publish(ignMsg);
}

}  // namespace backend
}  // namespace delphyne
