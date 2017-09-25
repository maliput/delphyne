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

#ifndef DELPHYNE_BRIDGE_IGNTOPICREPEATER_HH_
#define DELPHYNE_BRIDGE_IGNTOPICREPEATER_HH_

#include <memory>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <lcm/lcm-cpp.hpp>

#include "abstract_repeater.hh"

#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"

#include "ign_to_lcm_translation.hh"

#include "translate_exception.hh"

namespace delphyne {
namespace bridge {

// \brief IgnTopicRepeater listens to all the messages that arrive
// to an ignition topic, translates them into their lcm message counterpart
// and re-publishes them into an lcm channel of the same name.
// IGN_TYPE is the type of the ignition message that we want to consume
// whereas LCM_TYPE is the type of the LCM message that we will be generating.
template <class IGN_TYPE, class LCM_TYPE>
class IgnTopicRepeater : public AbstractRepeater {
 public:
  IgnTopicRepeater(std::shared_ptr<lcm::LCM> lcm, const std::string& topicName)
      : lcm_(lcm), topicName_(topicName) {}

  /// \brief Subscribe to the ignition topic and echo every message
  /// into the LCM channel as new messages arrive.
  void Start() {
    if (!lcm_->good()) {
      throw std::runtime_error("LCM is not ready");
    }

    std::string topic = "/" + topicName_;

    if (!node_.Subscribe(topic,
                         &IgnTopicRepeater<IGN_TYPE, LCM_TYPE>::handleMessage,
                         this)) {
      throw std::runtime_error("Error subscribing to topic: " + topic);
    }
  }

 private:
  /// \internal
  /// \brief The LCM manager
  std::shared_ptr<lcm::LCM> lcm_;

  /// \internal
  /// \brief The topic this repeater subscribes to
  const std::string topicName_;

  /// \internal
  /// \brief The ignition node used to create the subscription
  ignition::transport::Node node_;

  /// \brief Callback to be triggered when a new message arrives to the
  /// ignition topic channel.
  /// \param[in] ignMsg The ignition message that arrived to the topic
  void handleMessage(const IGN_TYPE& ignMsg) {
    LCM_TYPE lcmMsg;
    try {
      //ignerr << "Trying to do translation" << std::endl;
      ignToLcm(ignMsg, &lcmMsg);
      lcm_->publish(topicName_, &lcmMsg);
    } catch (const delphyne::bridge::TranslateException& e) {
      ignerr
          << "An error occurred while trying to translate a message in channel "
          << topicName_ << ": " << std::endl;
      ignerr << e.what() << std::endl;
    }
  }
};

}  // namespace bridge
}  // namespace delphyne

#endif
