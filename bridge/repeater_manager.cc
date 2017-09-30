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

#include <sstream>

#include <ignition/common/Console.hh>

#include "repeater_factory.hh"
#include "repeater_manager.hh"

namespace delphyne {
namespace bridge {

/////////////////////////////////////////////////
void RepeaterManager::Start() {
  if (!node_.Advertise(ignitionRepeaterServiceName_,
                       &RepeaterManager::IgnitionRepeaterServiceHandler,
                       this)) {
    std::stringstream message;
    message << "Error while advertising service ["
            << ignitionRepeaterServiceName_ << "]";
    throw std::runtime_error(message.str());
  }
}

/////////////////////////////////////////////////
void RepeaterManager::IgnitionRepeaterServiceHandler(
    const ignition::msgs::StringMsg_V& request,
    ignition::msgs::Boolean& response, bool& result) {
  switch (request.data_size()) {
    case 0:
      ignerr << "Couldn't create repeater: missing topic name and "
             << "message type parameters" << std::endl;
      return;
    case 1:
      ignerr << "Couldn't create repeater for " << request.data(0)
             << ". Missing message type parameter" << std::endl;
      return;
    case 2:
      break;
    default:
      igndbg << "Service called with extra parameters, ignoring them"
             << std::endl;
      break;
  }

  // We are handling the message. If this fails or not will be recorded in the
  // response
  result = true;

  std::string topicName = request.data(0);
  std::string messageType = request.data(1);

  // If we are already repeating this topic, do nothing
  if (repeaters_.count(topicName)) {
    igndbg << "Already repeating " << topicName << ". Nothing to do here."
           << std::endl;
    response.set_data(true);
    return;
  }

  std::shared_ptr<delphyne::bridge::AbstractRepeater> repeater =
      delphyne::bridge::RepeaterFactory::New(messageType, lcm_, topicName);

  if (!repeater) {
    ignerr << "Couldn't create repeater for " << messageType << std::endl;
    response.set_data(false);
  } else {
    try {
      repeater->Start();
      repeaters_[topicName] = repeater;
      igndbg << "Repeater for " << topicName << " started." << std::endl;
      response.set_data(true);
    } catch (const std::runtime_error& error) {
      ignerr << "Failed to start ignition channel repeater for " << topicName
             << std::endl;
      ignerr << "Details: " << error.what() << std::endl;
      response.set_data(false);
    }
  }
}

}  // namespace bridge
}  // namespace delphyne
