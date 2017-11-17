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

#ifndef DELPHYNE_BRIDGE_LCMCHANNELREPEATER_HH_
#define DELPHYNE_BRIDGE_LCMCHANNELREPEATER_HH_

#include <string>

#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <lcm/lcm-cpp.hpp>

#include "abstract_repeater.h"

#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"

#include "lcm_to_ign_translation.h"

namespace delphyne {
namespace bridge {

// \brief LcmChannelRepeater listens to all the messages that arrive
// to an LCM channel, translates them into their ign-msgs counterpart
// and re-publishes them into an ign-transport topic of the same
// name. LCM_TYPE is the type of the LCM message that we want to consume
// whereas IGN_TYPE is the type of the ignition message that we will be
// generating.
template <class LCM_TYPE, class IGN_TYPE>
class LcmChannelRepeater : public AbstractRepeater {
 public:
  LcmChannelRepeater(std::shared_ptr<lcm::LCM> lcm,
                     const std::string& topicName)
      : lcm_(lcm), topicName_(topicName) {}

  /// \brief Subscribe to the LCM channel and echo into the
  /// ignition topic as new messages arrive.
  void Start() {
    publisher_ = node_.Advertise<IGN_TYPE>("/" + topicName_);

    if (!lcm_->good()) {
      throw std::runtime_error("LCM is not ready");
    }

    if (!publisher_) {
      throw std::runtime_error("Error advertising topic: " + topicName_);
    }

    lcm_->subscribe(topicName_, &LcmChannelRepeater::handleMessage, this);
  }

 private:
  /// \internal
  /// \brief The LCM manager
  std::shared_ptr<lcm::LCM> lcm_;

  /// \internal
  /// \brief The topic this repeater subscribes to LCM and
  /// repeats on ignition
  const std::string topicName_;

  /// \internal
  /// \brief The ignition node used to create the publisher
  ignition::transport::Node node_;

  /// \internal
  /// \brief The ignition publisher used to echo the LCM messages
  ignition::transport::Node::Publisher publisher_;

  /// \brief Callback to be triggered when a new message arrives to the
  /// LCM channel.
  /// \param[in] rbuf A buffer with the raw bytes of the publication
  /// \param[in] chan The channel where the message arrived
  /// \param[in] lcmMsg The actual LCM message
  void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                     const LCM_TYPE* lcmMsg) {
    IGN_TYPE ignMsg;
    try {
      lcmToIgn(*lcmMsg, &ignMsg);
      publisher_.Publish(ignMsg);
    } catch (const delphyne::bridge::TranslateException& e) {
      ignerr
          << "An error occurred while trying to translate a message in channel "
          << chan << ": " << std::endl;
      ignerr << e.what() << std::endl;
    }
  }
};

}  // namespace bridge
}  // namespace delphyne

#endif
