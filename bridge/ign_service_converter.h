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

#ifndef DELPHYNE_BRIDGE_IGNSERVICECONVERTER_HH_
#define DELPHYNE_BRIDGE_IGNSERVICECONVERTER_HH_

#include <functional>
#include <memory>
#include <string>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_viewer_command.hpp"
#include "bridge/service_to_channel_translation.h"

namespace delphyne {
namespace bridge {

//////////////////////////////////////////////////
/// \brief IgnitionServiceConverter will listen to an
/// ignition service with a request type of IGN_REQ_TYPE
/// so that when it receives a request, an LCM
/// message of type LCM_TYPE will be sent
template <class IGN_REQ_TYPE, class LCM_TYPE>
class IgnitionServiceConverter {
 public:
  //////////////////////////////////////////////////
  /// \brief Creates an ignition service converter
  /// \param[in] lcm The LCM instance we are using to dispatch messages
  /// \param[in] notifierServiceName The ignition service name
  /// \param[in] channelName The LCM channel name
  IgnitionServiceConverter(std::shared_ptr<lcm::LCM> lcm,
                           const std::string& notifierServiceName,
                           const std::string& channelName)
      : ignServiceName_(notifierServiceName),
        lcm_(lcm),
        lcmChannelName_(channelName) {}

  //////////////////////////////////////////////////
  /// \brief Start advertising the ignition service call
  void Start() {
    if (!ignNode_.Advertise(ignServiceName_,
                            &IgnitionServiceConverter::IgnitionConverterHandler,
                            this)) {
      std::stringstream errorMsg;
      errorMsg << "Error advertising service [" << ignServiceName_ << "]"
               << std::endl;
      throw std::runtime_error(errorMsg.str());
    }
  }

 private:
  /// \internal
  /// \brief The ignition node
  ignition::transport::Node ignNode_;

  /// \internal
  /// \brief The ignition service listened
  const std::string ignServiceName_;

  /// \internal
  /// \brief The LCM manager
  std::shared_ptr<lcm::LCM> lcm_;

  /// \internal
  /// \brief The topic this converter publishes to
  const std::string lcmChannelName_;

  //////////////////////////////////////////////////
  /// \brief Service handler function, this will call a
  /// convertServiceToMsg overloaded function and publish the
  /// result into an lcm channel
  void IgnitionConverterHandler(const IGN_REQ_TYPE& request,
                                // NOLINTNEXTLINE(runtime/references) due to ign-transport API
                                ignition::msgs::Boolean& response,
                                // NOLINTNEXTLINE(runtime/references) due to ign-transport API
                                bool& result) {
    LCM_TYPE message = delphyne::bridge::convertServiceToMsg(request);
    if (lcm_->publish(lcmChannelName_, &message) != -1) {
      // The response succeed
      response.set_data(true);
    } else {
      // The response failed
      response.set_data(false);
    }
    result = true;
  }
};

}  // namespace bridge
}  // namespace delphyne

#endif
