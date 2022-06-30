#pragma once

#include <memory>

#include <maliput/api/road_network.h>

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
    DELPHYNE_DEMAND(rn_ != nullptr);
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
