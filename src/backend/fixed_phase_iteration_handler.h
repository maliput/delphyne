// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
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
#pragma once

#include <atomic>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <maliput/api/road_network.h>
#include <maliput/common/maliput_copyable.h>

#include "backend/dynamic_environment_handler.h"
#include "delphyne/macros.h"

namespace delphyne {

/// DynamicEnvironmentHandler class implementation.
/// Each rule state is expected to last a fixed amount of time.
/// An ignition service is provided for modifying the phase duration.
/// A topic is advertised for publishing the current phase for each PhaseRing. The message being published
/// is a ignition::msgs::StringMsg.
/// topic_name : "/current_phase
///
class FixedPhaseIterationHandler : public DynamicEnvironmentHandler {
 public:
  /// Name of service for modifying phase duration.
  static constexpr char kCurrentPhaseTopic[] = "/current_phase";
  static constexpr char kSetPhaseDurationSrvName[] = "/set_phase_duration";

  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(FixedPhaseIterationHandler)
  FixedPhaseIterationHandler() = delete;

  /// Constructs a FixedPhaseIterationHandler.
  /// @param road_network maliput::api::RoadNetwork pointer.
  /// @param phase_duration The duration of the rule's states in seconds.
  FixedPhaseIterationHandler(maliput::api::RoadNetwork* road_network, double phase_duration = 10.);

  ~FixedPhaseIterationHandler() override = default;

  void Update(double sim_time) override;

  /// @returns The duration of the rule's states in seconds.
  double get_phase_duration() const;

  /// @param phase_duration The duration of the rule's states in seconds.
  void set_phase_duration(double phase_duration);

 protected:
  void SetPhaseDurationSvCb(const ignition::msgs::Double& phase_duration);

 private:
  std::atomic<double> phase_duration_{10.};
  ignition::transport::Node node_;
  double last_sim_time_{0};
  ignition::transport::Node::Publisher pub_{};
};

}  // namespace delphyne
