#include "systems/calc_ongoing_road_position.h"

#include <gtest/gtest.h>

#include <dragway/road_geometry.h>
#include <drake/common/autodiff.h>
#include <multilane/multilane_onramp_merge.h>

#include "multilane_test_utilities/eigen_matrix_compare.h"

namespace delphyne {

using drake::systems::rendering::FrameVelocity;
using drake::systems::rendering::PoseVector;
using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LanePosition;
using maliput::api::RoadPosition;
using maliput::api::Rotation;

namespace {

const LanePosition kSomeLanePosition{20., 0., 0.};

enum class LanePolarity { kWithS, kAgainstS };

// Sets the pose at s = 1 and sets velocity according to the orientation of the
// lane, at the requested polarity.  Returns the ground truth LanePosition.
template <typename T>
void SetOnrampPoses(const Lane* lane, LanePolarity polarity, const T& speed, PoseVector<T>* pose,
                    FrameVelocity<T>* velocity) {
  const GeoPosition xyz = lane->ToGeoPosition(kSomeLanePosition);
  pose->set_translation(drake::Translation3<T>(T(xyz.x()), T(xyz.y()), T(xyz.z())));
  const Rotation rotation = lane->GetOrientation(kSomeLanePosition);
  const double roll = rotation.roll();
  const double pitch = rotation.pitch();
  const double yaw = rotation.yaw() - ((polarity == LanePolarity::kWithS) ? 0. : M_PI);
  const drake::math::RollPitchYaw<double> new_rotation(roll, pitch, yaw);
  pose->set_rotation(new_rotation.ToQuaternion());

  const drake::math::RotationMatrix<double> rotmat(new_rotation);
  drake::Vector6<T> velocity_vector{};
  velocity_vector.head(3) = drake::Vector3<T>::Zero();            // ω
  velocity_vector.tail(3) = speed * rotmat.matrix().leftCols(1);  // v
  velocity->set_velocity(drake::multibody::SpatialVelocity<T>(velocity_vector));
}

const Lane* GetLaneFromId(const maliput::api::RoadGeometry& road, const std::string& lane_id) {
  for (int i = 0; i < road.num_junctions(); ++i) {
    if (road.junction(i)->segment(0)->lane(0)->id().string() == lane_id) {
      return road.junction(i)->segment(0)->lane(0);
    }
  }
  throw std::runtime_error("No matching lane name in the road network");
}

// Sets up poses in the requested lane and performs the tests based on a
// hypothesized RoadPosition.  If the requested lane is nullptr, then the
// expected result is the default RoadPosition.
void PerformTest(const maliput::api::RoadGeometry& rg, const Lane* lane, LanePolarity polarity, double speed,
                 const Lane* expected_lane, const LanePosition& expected_lp) {
  DRAKE_DEMAND(lane != nullptr);

  // Set the hypothetical pose at s = 10 in `post0`.
  RoadPosition rp(GetLaneFromId(rg, "l:post0_0"), {10., 0., 0.});

  // Set the actual pose in the requested lane.
  PoseVector<double> pose;
  FrameVelocity<double> velocity;
  SetOnrampPoses<double>(lane, polarity, speed, &pose, &velocity);

  // The DUT.
  CalcOngoingRoadPosition(pose, velocity, rg, &rp);
  EXPECT_TRUE(CompareMatrices(drake::Vector3<double>{expected_lp.s(), expected_lp.r(), expected_lp.h()},
                              drake::Vector3<double>{rp.pos.s(), rp.pos.r(), rp.pos.h()}, 1e-10));
  if (!expected_lane) {
    EXPECT_EQ(RoadPosition().lane, rp.lane);
  } else {
    EXPECT_EQ(expected_lane->id(), rp.lane->id());
  }
}

GTEST_TEST(CalcOngoingRoadPosition, TestOngoingLanes) {
  // N.B. In this road, `post0` branches into `pre0` and `onramp1`.
  auto merge_road = std::make_unique<maliput::multilane::MultilaneOnrampMerge>();
  std::unique_ptr<const maliput::api::RoadGeometry> rg = merge_road->BuildOnramp();

  // Set speed to be zero and the car facing both along and against the
  // s-direction.
  for (const auto polarity : {LanePolarity::kWithS, LanePolarity::kAgainstS}) {
    for (const double speed : {10., 0.}) {
      const Lane* post0 = GetLaneFromId(*rg, "l:post0_0");
      PerformTest(*rg, post0, polarity, speed, post0, kSomeLanePosition);

      const Lane* pre0 = GetLaneFromId(*rg, "l:pre0_0");
      PerformTest(*rg, pre0, polarity, speed, pre0, kSomeLanePosition);

      const Lane* onramp1 = GetLaneFromId(*rg, "l:onramp1_0");
      PerformTest(*rg, onramp1, polarity, speed, onramp1, kSomeLanePosition);
    }
  }
}

GTEST_TEST(CalcOngoingRoadPosition, TestInvalidLanes) {
  auto merge_road = std::make_unique<maliput::multilane::MultilaneOnrampMerge>();
  std::unique_ptr<const maliput::api::RoadGeometry> rg = merge_road->BuildOnramp();

  PoseVector<double> pose;
  FrameVelocity<double> velocity;
  const double speed = 10.;

  // A nullptr RoadGeometry should throw.
  EXPECT_THROW(CalcOngoingRoadPosition(pose, velocity, *rg, nullptr), std::runtime_error);

  // Set a hypothetical RoadPosition with nullptr Lane.
  RoadPosition rp(nullptr, LanePosition{0., 0., 0.});

  // Set the actual pose somewhere well outside the RoadGeometry.
  SetOnrampPoses(GetLaneFromId(*rg, "l:post5_0"), LanePolarity::kWithS, speed, &pose, &velocity);
  pose.set_translation(Eigen::Translation3d(1000., 1000., 0.));

  CalcOngoingRoadPosition(pose, velocity, *rg, &rp);

  // Expect RoadPosition to be closest to `onramp0`.
  EXPECT_EQ(GetLaneFromId(*rg, "l:onramp0_0"), rp.lane);
  EXPECT_TRUE(CompareMatrices(drake::Vector3<double>{LanePosition{100., -4., 0.}.s(), LanePosition{100., -4., 0.}.r(),
                                                     LanePosition{100., -4., 0.}.h()},
                              drake::Vector3<double>{rp.pos.s(), rp.pos.r(), rp.pos.h()}, 1e-10));
}

GTEST_TEST(CalcOngoingRoadPosition, TestAutoDiff) {
  maliput::dragway::RoadGeometry rg(
      maliput::api::RoadGeometryId("1-lane dragway"), 1 /* num_lanes */, 100. /* length */, 2. /* lane_width */,
      0. /* shoulder_width */, 5. /* maximum_height */, std::numeric_limits<double>::epsilon() /* linear_tolerance */,
      std::numeric_limits<double>::epsilon() /* angular_tolerance */);

  // AutoDiffXd only appear at the inputs; only check that computation succeeds.
  const LanePolarity polarity = LanePolarity::kWithS;
  const drake::AutoDiffXd speed = 10.;

  const Lane* lane = rg.junction(0)->segment(0)->lane(0);

  // Set the hypothetical pose at s = 10.
  RoadPosition rp(lane, {10., 0., 0.});

  // Set the actual pose in the requested lane.
  PoseVector<drake::AutoDiffXd> pose;
  FrameVelocity<drake::AutoDiffXd> velocity;
  SetOnrampPoses<drake::AutoDiffXd>(lane, polarity, speed, &pose, &velocity);

  // The DUT.
  CalcOngoingRoadPosition(pose, velocity, rg, &rp);

  EXPECT_TRUE(
      CompareMatrices(drake::Vector3<double>{kSomeLanePosition.s(), kSomeLanePosition.r(), kSomeLanePosition.h()},
                      drake::Vector3<double>{rp.pos.s(), rp.pos.r(), rp.pos.h()}, 1e-10));
  EXPECT_EQ(lane->id(), rp.lane->id());
}

}  // namespace
}  // namespace delphyne
