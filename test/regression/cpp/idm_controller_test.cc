#include "systems/idm_controller.h"

#include <gtest/gtest.h>

#include "delphyne/roads/road_builder.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_velocity.h"

#include "maliput/api/road_geometry.h"
#include "maliput_multilane_test_utilities/eigen_matrix_compare.h"

#include "test_utilities/scalar_conversion.h"

namespace delphyne {

// There seems to be an issue with INSTANTIATE_TEST_CASE_P and gmock 1.7.0 when
// using an unnamed namespace, so using a name for this one
// https://groups.google.com/forum/#!topic/googletestframework/gVPQmRfJ1m8
namespace test_p {

using drake::AutoDiffXd;
using drake::math::RigidTransform;
using drake::systems::rendering::FrameVelocity;
using drake::systems::rendering::PoseVector;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::RoadPosition;

static constexpr double kEgoSPosition{10.};
static constexpr int kLeadIndex{0};
static constexpr int kEgoIndex{1};

class IDMControllerTest : public ::testing::TestWithParam<RoadPositionStrategy> {
 protected:
  void SetUpIdm(ScanStrategy path_or_branches) {
    // Create a straight road with one lane.
    road_ = roads::CreateDragway("Single-Lane Dragway", 1 /* num_lanes */, 100. /* length */, 2. /* lane_width */,
                                 0. /* shoulder_width */, 5. /* maximum_height */,
                                 std::numeric_limits<double>::epsilon() /* linear_tolerance */,
                                 std::numeric_limits<double>::epsilon() /* angular_tolerance */);

    cache_or_search_ = this->GetParam();
    period_sec_ = 1.;

    // Initialize IDMController with the road.
    dut_.reset(new IDMController<double>(*road_, path_or_branches, cache_or_search_, period_sec_));
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();

    const auto idm = dynamic_cast<const IDMController<double>*>(dut_.get());
    DRAKE_DEMAND(idm != nullptr);
    ego_pose_input_index_ = idm->ego_pose_input().get_index();
    ego_velocity_input_index_ = idm->ego_velocity_input().get_index();
    traffic_input_index_ = idm->traffic_input().get_index();
    acceleration_output_index_ = idm->acceleration_output().get_index();
  }

  // Set the default poses according to the desired offset position and speeds
  // for the lead car.  `s_offset = s_lead - s_ego`, `relative_sdot =
  // sdot_lead - sdot_ego`, and `relative_rdot = rdot_lead - rdot_ego`.
  void SetDefaultPoses(const double ego_speed, const double s_offset = 0., const double relative_sdot = 0.,
                       const double relative_rdot = 0.) {
    DRAKE_DEMAND(ego_pose_input_index_ >= 0 && ego_pose_input_index_ < dut_->num_input_ports());
    DRAKE_DEMAND(ego_velocity_input_index_ >= 0 && ego_velocity_input_index_ < dut_->num_input_ports());
    DRAKE_DEMAND(traffic_input_index_ >= 0 && traffic_input_index_ < dut_->num_input_ports());

    auto ego_pose = std::make_unique<PoseVector<double>>();
    auto ego_velocity = std::make_unique<FrameVelocity<double>>();
    drake::systems::rendering::PoseBundle<double> traffic_poses(2);

    DRAKE_DEMAND(s_offset >= 0.);
    EXPECT_LE(0., ego_speed);

    // Configure the ego car pose and velocity.
    const Eigen::Translation3d translation_ego(kEgoSPosition /* x */, 0. /* y */, 0. /* z */);
    ego_pose->set_translation(translation_ego);
    context_->FixInputPort(ego_pose_input_index_, std::move(ego_pose));

    drake::Vector6<double> velocity{};
    velocity << 0. /* ωx */, 0. /* ωy */, 0. /* ωz */, ego_speed /* vx */, 0. /* vy */, 0. /* vz */;
    ego_velocity->set_velocity(drake::multibody::SpatialVelocity<double>(velocity));
    context_->FixInputPort(ego_velocity_input_index_, std::move(ego_velocity));

    // Configure the traffic poses, inclusive of the ego car.
    FrameVelocity<double> lead_velocity;
    const Eigen::Translation3d translation_lead(kEgoSPosition + s_offset /* x */, 0. /* y */, 0. /* z */);
    traffic_poses.set_transform(kLeadIndex, RigidTransform<double>(Eigen::Isometry3d(translation_lead)));
    velocity[3] += relative_sdot;
    velocity[4] += relative_rdot;
    lead_velocity.set_velocity(drake::multibody::SpatialVelocity<double>(velocity));
    traffic_poses.set_velocity(kLeadIndex, lead_velocity);
    traffic_poses.set_transform(kEgoIndex, RigidTransform<double>(Eigen::Isometry3d(translation_ego)));
    context_->FixInputPort(traffic_input_index_, drake::AbstractValue::Make(traffic_poses));
  }

  std::unique_ptr<drake::systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<drake::systems::Context<double>> context_;
  std::unique_ptr<drake::systems::SystemOutput<double>> output_;
  std::unique_ptr<const maliput::api::RoadGeometry> road_;

  int ego_pose_input_index_;
  int ego_velocity_input_index_;
  int traffic_input_index_;
  int acceleration_output_index_;

  RoadPositionStrategy cache_or_search_;
  double period_sec_{};
};

TEST_P(IDMControllerTest, Topology) {
  SetUpIdm(ScanStrategy::kPath);

  ASSERT_EQ(3, dut_->num_input_ports());
  const auto& ego_pose_input_port = dut_->get_input_port(ego_pose_input_index_);
  EXPECT_EQ(drake::systems::kVectorValued, ego_pose_input_port.get_data_type());
  EXPECT_EQ(7 /* PoseVector input */, ego_pose_input_port.size());
  const auto& ego_velocity_input_port = dut_->get_input_port(ego_velocity_input_index_);
  EXPECT_EQ(drake::systems::kVectorValued, ego_velocity_input_port.get_data_type());
  EXPECT_EQ(6 /* FrameVelocity input */, ego_velocity_input_port.size());
  const auto& traffic_input_port = dut_->get_input_port(traffic_input_index_);
  EXPECT_EQ(drake::systems::kAbstractValued, traffic_input_port.get_data_type());

  ASSERT_EQ(1, dut_->num_output_ports());
  const auto& output_port = dut_->get_output_port(acceleration_output_index_);
  EXPECT_EQ(drake::systems::kVectorValued, output_port.get_data_type());
  EXPECT_EQ(1 /* accleration output */, output_port.size());
}

// Tests that the unrestricted update has been registered and updates to the
// state are correctly made.
TEST_P(IDMControllerTest, UnrestrictedUpdate) {
  SetUpIdm(ScanStrategy::kPath);
  if (cache_or_search_ == RoadPositionStrategy::kCache) {
    EXPECT_EQ(1, context_->num_abstract_states());

    SetDefaultPoses(10. /* ego_speed */, 0. /* s_offset */, -5. /* rel_sdot */);

    // Check that the unrestricted event has been registered.
    drake::systems::LeafCompositeEventCollection<double> events;
    double t = dut_->CalcNextUpdateTime(*context_, &events);
    EXPECT_EQ(t, period_sec_);
    const drake::systems::EventCollection<drake::systems::UnrestrictedUpdateEvent<double>>& e =
        events.get_unrestricted_update_events();
    const auto& leaf_events =
        static_cast<const drake::systems::LeafEventCollection<drake::systems::UnrestrictedUpdateEvent<double>>&>(e);
    EXPECT_EQ(static_cast<int>(leaf_events.get_events().size()), 1);

    drake::systems::State<double>& state = context_->get_mutable_state();
    dut_->CalcUnrestrictedUpdate(*context_, &state);
    const RoadPosition& rp = state.get_abstract_state<RoadPosition>(0);
    const Lane* expected_lane = road_->junction(0)->segment(0)->lane(0);
    EXPECT_EQ(expected_lane->id(), rp.lane->id());
    const maliput::math::Vector3 lane_position_srh{LanePosition{kEgoSPosition, 0., 0.}.srh()};
    const maliput::math::Vector3 rp_pos_srh{rp.pos.srh()};
    EXPECT_TRUE(
        CompareMatrices(drake::Vector3<double>{lane_position_srh.x(), lane_position_srh.y(), lane_position_srh.z()},
                        drake::Vector3<double>{rp_pos_srh.x(), rp_pos_srh.y(), rp_pos_srh.z()}));
  }
}

TEST_P(IDMControllerTest, Output) {
  SetUpIdm(ScanStrategy::kPath);
  // Define a pointer to where the BasicVector results end up.
  const auto result = output_->get_vector_data(acceleration_output_index_);

  // Set the lead car to be immediately ahead of the ego car and moving
  // slower.
  SetDefaultPoses(10. /* ego_speed */, 6. /* s_offset */, -5. /* rel_sdot */);
  dut_->CalcOutput(*context_, output_.get());
  const double closing_accel = (*result)[0];  // A negative number.

  // Expect the car to decelerate in this configuration.
  EXPECT_GT(0., closing_accel);

  // Set the same conditions as above, but with the lead car having a nonzero
  // r-component in its velocity.
  SetDefaultPoses(10. /* ego_speed */, 6. /* s_offset */, -5. /* rel_sdot */, 5. /* rel_rdot */);
  dut_->CalcOutput(*context_, output_.get());

  // Expect no change to the previous results.
  EXPECT_EQ(closing_accel, (*result)[0]);

  // Set the lead car to be immediately ahead of the ego car and moving
  // faster.
  SetDefaultPoses(10. /* ego_speed */, 6. /* s_offset */, 5. /* rel_sdot */);
  dut_->CalcOutput(*context_, output_.get());

  // Expect the magnitude of the deceleration to be smaller when the ego is
  // not closing in on the lead car.
  EXPECT_GT((*result)[0], closing_accel);

  // Set the ego car to be alone on the road.  We effectively enable this by
  // setting the poses of all traffic cars to be that of the ego car.
  SetDefaultPoses(10. /* ego_speed */);
  dut_->CalcOutput(*context_, output_.get());

  // Expect zero acceleration (input velocity is at the desired velocity).
  EXPECT_NEAR(0., (*result)[0], 1e-4);

  // Set the lead car well ahead of the ego, with the ego moving slower than
  // the desired velocity.
  SetDefaultPoses(4. /* ego_speed */, 30. /* s_offset */, 0. /* rel_sdot */);
  dut_->CalcOutput(*context_, output_.get());

  // Expect a positive acceleration in this configuration.
  EXPECT_LT(0., (*result)[0]);

  // Set the lead car to be well within `distance_lower_limit`.
  SetDefaultPoses(10. /* ego_speed */, 1e-3 /* s_offset */, -5. /* rel_sdot */);
  dut_->CalcOutput(*context_, output_.get());

  // Expect an enormous deceleration.
  EXPECT_GT(closing_accel, (*result)[0]);
}

TEST_P(IDMControllerTest, ToAutoDiff) {
  SetUpIdm(ScanStrategy::kPath);
  SetDefaultPoses(10. /* ego_speed */, 6. /* s_offset */, -5. /* rel_sdot */);

  EXPECT_TRUE(is_autodiffxd_convertible(*dut_, [&](const auto& other_dut) {
    const auto other_context = other_dut.CreateDefaultContext();
    const auto other_output = other_dut.AllocateOutput();

    // Verify that CalcOutput returns a result and validate its AutoDiff
    // derivatives.
    const AutoDiffXd kZeroDerivative{0., drake::Vector1d(0.)};

    auto pose = std::make_unique<PoseVector<AutoDiffXd>>();
    const drake::Translation3<AutoDiffXd> translation(AutoDiffXd(0., drake::Vector1d(1.)), /* x */
                                                      kZeroDerivative,                     /* y */
                                                      kZeroDerivative);                    /* z */
    pose->set_translation(translation);
    other_context->FixInputPort(ego_pose_input_index_, std::move(pose));

    auto velocity = std::make_unique<FrameVelocity<AutoDiffXd>>();
    const drake::multibody::SpatialVelocity<AutoDiffXd> velocity_vector(
        drake::Vector3<AutoDiffXd>(kZeroDerivative /* ωx */, kZeroDerivative /* ωy */, kZeroDerivative /* ωz */),
        drake::Vector3<AutoDiffXd>(kZeroDerivative /* vx */, kZeroDerivative /* vy */, kZeroDerivative /* vz */));
    velocity->set_velocity(velocity_vector);
    other_context->FixInputPort(ego_velocity_input_index_, std::move(velocity));

    drake::systems::rendering::PoseBundle<AutoDiffXd> poses(1);
    FrameVelocity<AutoDiffXd> traffic_velocity;
    traffic_velocity.set_velocity(velocity_vector);
    poses.set_velocity(0, traffic_velocity);
    poses.set_transform(0, RigidTransform<AutoDiffXd>(drake::Isometry3<AutoDiffXd>(translation)));
    other_context->FixInputPort(traffic_input_index_, drake::AbstractValue::Make(poses));

    const auto result = other_output->get_vector_data(acceleration_output_index_);
    other_dut.CalcOutput(*other_context, other_output.get());

    // It suffices to check that the autodiff derivative seeded at the inputs
    // produces a sane value and that the derivatives field is correctly sized.
    EXPECT_LT(0., (*result)[0]);
    // We expect the derivative to be zero. Therefore the derivatives vector
    // either:
    //   1. It has zero size or,
    //   2. Has size of one (1) and first entry equal to zero.
    // Note: C++'s "short-circuit evaluation" ensures that the second expression
    //       is never evaluated if the derivatives vector has zero size.
    EXPECT_TRUE((*result)[0].derivatives().size() == 0 || (*result)[0].derivatives()(0) == 0.);
  }));
}

// Check that, when path_or_branches == ScanStrategy::kBranches, we can
// instantiate an IDMController and CalcOutput and it produces an expected
// result.
TEST_P(IDMControllerTest, CheckBranches) {
  SetUpIdm(ScanStrategy::kBranches);

  // Set the lead car to be immediately ahead of the ego car and moving
  // slower than it.
  SetDefaultPoses(10. /* ego_speed */, 6. /* s_offset */, -5. /* rel_sdot */);
  dut_->CalcOutput(*context_, output_.get());
  const auto result = output_->get_vector_data(acceleration_output_index_);
  const double closing_accel = (*result)[0];

  // Expect the car to decelerate.
  EXPECT_GT(0., closing_accel);
}

// Perform all tests with cache and exhaustive search options.
INSTANTIATE_TEST_CASE_P(RoadPositionStrategy, IDMControllerTest,
                        testing::Values(RoadPositionStrategy::kCache, RoadPositionStrategy::kExhaustiveSearch));

}  // namespace test_p
}  // namespace delphyne
