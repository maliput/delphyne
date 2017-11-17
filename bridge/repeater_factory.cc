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

#include <regex>

#include "bridge/repeater_factory.h"

namespace delphyne {
namespace bridge {

static std::map<std::string, RepeaterFactoryFunction>* repeaterMap = nullptr;

/////////////////////////////////////////////////
void RepeaterFactory::Register(const std::string& namePattern,
                               RepeaterFactoryFunction factoryFunction) {
  // Create the repeaterMap if it's null
  if (!repeaterMap) {
    repeaterMap = new std::map<std::string, RepeaterFactoryFunction>;
  }

  (*repeaterMap)[namePattern] = factoryFunction;
}

/////////////////////////////////////////////////
std::shared_ptr<delphyne::bridge::AbstractRepeater> RepeaterFactory::New(
    const std::string& topicOrChannelName, std::shared_ptr<lcm::LCM> lcm) {
  if (repeaterMap) {
    for (const auto& pair : *repeaterMap) {
      std::string namePattern = pair.first;
      if (std::regex_match(topicOrChannelName, std::regex(namePattern))) {
        RepeaterFactoryFunction factoryFunction = pair.second;
        return factoryFunction(lcm, topicOrChannelName);
      }
    }
    return nullptr;
  } else {
    return nullptr;
  }
}
}
}
