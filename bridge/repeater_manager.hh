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

#ifndef DELPHYNE_BRIDGE_REPEATERMANAGER_HH_
#define DELPHYNE_BRIDGE_REPEATERMANAGER_HH_

#include <map>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "abstract_repeater.hh"
#include "lcm/lcm-cpp.hpp"

namespace delphyne {
namespace bridge {

/// \brief A class that is in charge of spawning new repeaters and manage
/// their life cycle.
class RepeaterManager {
 public:
  /// \brief Constructor
  /// \param[in] lcm The LCM manager that will be in turn passed to the
  /// repeaters we create.
  /// \param[in] ignitionRepeaterServiceName The name of the ignition service
  /// that the manager will expose.
  RepeaterManager(
      std::shared_ptr<lcm::LCM> lcm,
      std::string ignitionRepeaterServiceName = "/repeat_ignition_topic")
      : lcm_(lcm), ignitionRepeaterServiceName_(ignitionRepeaterServiceName) {}

  /// \brief Start the manager by registering the ignition service
  /// @throws std::runtime_error if there is a problem while advertising the
  /// ignition service.
  void Start();

 private:
  /// \brief This method is set as a callback of the published service to
  /// start a new repeater.
  /// \param[in] request An array of two strings. The first string dictates the
  /// ignition topic name to repeat and the second one the ignition type that
  /// will be delivered in that topic
  /// \param[out] response A boolean indicating if the manager was able to
  /// properly setup the repeater or not.
  /// \param[out] result Always true
  void IgnitionRepeaterServiceHandler(
      const ignition::msgs::StringMsg_V& request,
      ignition::msgs::Boolean& response, bool& result);

  /// \internal
  /// \brief The LCM manager
  std::shared_ptr<lcm::LCM> lcm_;

  /// \internal
  /// \brief The map between topics and repeaters
  std::map<std::string, std::shared_ptr<delphyne::bridge::AbstractRepeater>> repeaters_;

  /// \internal
  /// \brief The transport node used to advertise the ignition service
  ignition::transport::Node node_;

  /// \internal
  /// \brief The name of the service used to create a new ignition topic
  /// repeater
  std::string ignitionRepeaterServiceName_;
};

}  // namespace bridge
}  // namespace delphyne

#endif
