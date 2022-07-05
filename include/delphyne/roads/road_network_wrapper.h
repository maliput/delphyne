// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
// All rights reserved.
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

#include <maliput/api/road_network.h>

#include "delphyne/macros.h"

namespace delphyne {
namespace roads {

/// Wrapper around a maliput::api::RoadNetwork.
/// This implementation is needed for the python bindings due to a lack of support when handling smart pointers.
/// For more information see:
/// - https://github.com/maliput/delphyne/issues/846
/// - https://github.com/maliput/maliput_infrastructure/issues/225
///
/// TODO(https://github.com/maliput/delphyne/issues/847): Remove this wrapper once the above issues are resolved.
class RoadNetworkWrapper {
 public:
  explicit RoadNetworkWrapper(std::unique_ptr<maliput::api::RoadNetwork> rn) : rn_(std::move(rn)) {
    DELPHYNE_VALIDATE(rn_ != nullptr, std::runtime_error, "RoadNetworkWrapper: nullptr RoadNetwork");
  }

  /// Releases the unique_ptr to the maliput::api::RoadNetwork.
  /// Note: The unique_ptr object is released from the responsibility of deleting the object. Some other entity must
  /// take responsibility for deleting the object at some point.
  /// @returns A pointer to the maliput::api::RoadNetwork.
  maliput::api::RoadNetwork* release() { return rn_.release(); }

  /// @returns A pointer to the underlying maliput::api::RoadNetwork.
  maliput::api::RoadNetwork* operator->() const noexcept { return rn_.get(); }

  /// @returns A reference to the underlying maliput::api::RoadNetwork.
  maliput::api::RoadNetwork& operator*() const noexcept { return *rn_; }

 private:
  std::unique_ptr<maliput::api::RoadNetwork> rn_;
};

}  // namespace roads
}  // namespace delphyne
