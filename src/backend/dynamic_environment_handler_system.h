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

#include <memory>

#include <drake/systems/framework/event_status.h>
#include <drake/systems/framework/leaf_system.h>
#include <maliput/api/road_network.h>

#include "backend/dynamic_environment_handler.h"
#include "delphyne/macros.h"

namespace delphyne {

class DynamicEnvironmentHandlerSystem : public drake::systems::LeafSystem<double> {
 public:
  DynamicEnvironmentHandlerSystem(std::unique_ptr<DynamicEnvironmentHandler> deh) : deh_(std::move(deh)) {
    DELPHYNE_VALIDATE(deh_ != nullptr, std::invalid_argument, "Invalid DynamicEnvironmentHandler.");
    this->DeclarePerStepUnrestrictedUpdateEvent(&DynamicEnvironmentHandlerSystem::UpdatePhases);
  }

  /// Default destructor.
  ~DynamicEnvironmentHandlerSystem() override {}

 private:
  drake::systems::EventStatus UpdatePhases(const drake::systems::Context<double>& context,
                                           drake::systems::State<double>*) const {
    deh_->Update(context.get_time());
    return drake::systems::EventStatus::Succeeded();
  }

  mutable std::unique_ptr<DynamicEnvironmentHandler> deh_;
};

}  // namespace delphyne
