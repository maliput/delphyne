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

#include "repeater_factory.h"
#include "repeater_manager.h"

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

  if (!node_.Advertise(lcmRepeaterServiceName_,
                       &RepeaterManager::LCMRepeaterServiceHandler, this)) {
    std::stringstream message;
    message << "Error while advertising service [" << lcmRepeaterServiceName_
            << "]";
    throw std::runtime_error(message.str());
  }
}

/////////////////////////////////////////////////
void RepeaterManager::EnableLCMAutodiscovery() {
  if (!lcmAutodiscoveryEnabled_) {
    lcmAutodiscoveryEnabled_ = true;
    lcm_->subscribe(".*", &RepeaterManager::LCMMessageHandler, this);
  }
}

/////////////////////////////////////////////////
void RepeaterManager::IgnitionRepeaterServiceHandler(
    const ignition::msgs::StringMsg& request, ignition::msgs::Boolean& response,
    bool& result) {
  if (!request.has_data()) {
    ignerr << "Couldn't create repeater: missing topic name" << std::endl;
    return;
  }

  // We are handling the message. If this fails or not will be recorded in the
  // response
  result = true;

  const std::string topicName = request.data();

  response.set_data(StartRepeater(topicName));
}

/////////////////////////////////////////////////
void RepeaterManager::LCMRepeaterServiceHandler(
    const ignition::msgs::StringMsg& request, ignition::msgs::Boolean& response,
    bool& result) {
  if (!request.has_data()) {
    ignerr << "Couldn't create repeater: missing channel name" << std::endl;
    return;
  }

  // We are handling the message. If this fails or not will be recorded in the
  // response
  result = true;

  const std::string channelName = request.data();

  response.set_data(StartRepeater(channelName));
}

/////////////////////////////////////////////////
void RepeaterManager::LCMMessageHandler(const lcm::ReceiveBuffer* rbuf,
                                        const std::string& channel) {
  if (!IsRepeating(channel) && blacklistedChannels_.count(channel) == 0) {
    if (!StartRepeater(channel)) {
      blacklistedChannels_.insert(channel);
    }
  }
}

/////////////////////////////////////////////////
bool RepeaterManager::IsRepeating(const std::string& channelOrTopic) {
  return repeaters_.count(channelOrTopic) > 0;
}

/////////////////////////////////////////////////
bool RepeaterManager::StartRepeater(const std::string& channelOrTopic) {
  // If we are already repeating this topic, do nothing
  if (IsRepeating(channelOrTopic)) {
    igndbg << "Already repeating " << channelOrTopic << ". Nothing to do here."
           << std::endl;
    return true;
  }

  std::shared_ptr<delphyne::bridge::AbstractRepeater> repeater =
      delphyne::bridge::RepeaterFactory::New(channelOrTopic, lcm_);

  if (!repeater) {
    ignerr << "Couldn't create repeater for " << channelOrTopic << std::endl;
    return false;
  } else {
    try {
      repeater->Start();
      repeaters_[channelOrTopic] = repeater;
      igndbg << "Repeater for " << channelOrTopic << " started." << std::endl;
      return true;
    } catch (const std::runtime_error& error) {
      ignerr << "Failed to start ignition channel repeater for "
             << channelOrTopic << std::endl;
      ignerr << "Details: " << error.what() << std::endl;
      return false;
    }
  }
}

}  // namespace bridge
}  // namespace delphyne
