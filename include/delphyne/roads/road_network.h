#pragma once

#include <memory>

#include <maliput/api/road_network.h>

namespace delphyne {
namespace roads {

class RoadNetwork {
 public:
  RoadNetwork(std::unique_ptr<maliput::api::RoadNetwork> maliput_rn) { rn_ = std::move(maliput_rn); }

  maliput::api::RoadNetwork* road_network() const { return rn_.get(); }
  maliput::api::RoadNetwork* release() { return rn_.release(); }
  bool Contains(const maliput::api::RoadPosition& road_position) const { return rn_->Contains(road_position); }

  bool Contains(const maliput::api::LaneId& lane_id) const { return rn_->Contains(lane_id); }

  const maliput::api::RoadGeometry* road_geometry() const { return rn_->road_geometry(); }

  const maliput::api::rules::RoadRulebook* rulebook() const { return rn_->rulebook(); }

  const maliput::api::rules::TrafficLightBook* traffic_light_book() const { return rn_->traffic_light_book(); }

  maliput::api::IntersectionBook* intersection_book() { return rn_->intersection_book(); }

  const maliput::api::rules::PhaseRingBook* phase_ring_book() const { return rn_->phase_ring_book(); }
  maliput::api::rules::RightOfWayRuleStateProvider* right_of_way_rule_state_provider() {
    return rn_->right_of_way_rule_state_provider();
  }
  maliput::api::rules::PhaseProvider* phase_provider() { return rn_->phase_provider(); }

  const maliput::api::rules::RuleRegistry* rule_registry() const { return rn_->rule_registry(); }

  maliput::api::rules::DiscreteValueRuleStateProvider* discrete_value_rule_state_provider() {
    return rn_->discrete_value_rule_state_provider();
  }

  maliput::api::rules::RangeValueRuleStateProvider* range_value_rule_state_provider() {
    return rn_->range_value_rule_state_provider();
  }

 private:
  std::unique_ptr<maliput::api::RoadNetwork> rn_;
};

}  // namespace roads
}  // namespace delphyne
