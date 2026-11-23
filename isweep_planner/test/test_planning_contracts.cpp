#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "isweep_planner/core/common.h"
#include "isweep_planner/core/trajectory_footprint_validator.h"
#include "isweep_planner/core/trajectory_utils.h"
#include "isweep_planner/env/collision_checker.h"
#include "isweep_planner/env/footprint_model.h"
#include "isweep_planner/env/grid_map.h"
#include "isweep_planner/framework/svsdf_runtime.h"
#include "isweep_planner/global_search/hybrid_astar.h"
#include "isweep_planner/optimization/backend/trajectory_optimizer.h"
#include "isweep_planner/optimization/midend/se2_sequence_generator.h"

namespace isweep_planner {
namespace {

PolyPiece MakePositionPiece(double duration,
                            const Eigen::Matrix<double, 2, 6>& coefficients) {
  PolyPiece piece;
  piece.duration = duration;
  piece.coeffs = coefficients;
  return piece;
}

YawPolyPiece MakeYawPiece(
    double duration, const Eigen::Matrix<double, 1, 6>& coefficients) {
  YawPolyPiece piece;
  piece.duration = duration;
  piece.coeffs = coefficients;
  return piece;
}

TEST(RiskLevelContract, UpgradeIsMonotoneAndSaturates) {
  EXPECT_EQ(RiskLevel::MEDIUM, UpgradeRiskLevel(RiskLevel::LOW));
  EXPECT_EQ(RiskLevel::HIGH, UpgradeRiskLevel(RiskLevel::MEDIUM));
  EXPECT_EQ(RiskLevel::HIGH, UpgradeRiskLevel(RiskLevel::HIGH));
  EXPECT_LT(RiskLevelRank(RiskLevel::LOW),
            RiskLevelRank(RiskLevel::MEDIUM));
  EXPECT_LT(RiskLevelRank(RiskLevel::MEDIUM),
            RiskLevelRank(RiskLevel::HIGH));
}

TEST(BaselineContract, ForcedR2CannotEscalateItsInitialSolverLevel) {
  RuntimeExperimentOptions options;
  EXPECT_TRUE(InitialLevelEscalationAllowed(options));
  options.force_all_low = true;
  EXPECT_FALSE(InitialLevelEscalationAllowed(options));
}

TEST(BaselineContract, ForcedStrictPreservesSegmentsAndMarksEverySegmentHigh) {
  std::vector<MotionSegment> segments(3);
  segments[0].risk = RiskLevel::LOW;
  segments[1].risk = RiskLevel::MEDIUM;
  segments[2].risk = RiskLevel::HIGH;
  segments[0].waypoints.resize(2);
  segments[1].waypoints.resize(4);
  segments[2].waypoints.resize(3);

  ForceAllSegmentsStrict(&segments);

  ASSERT_EQ(segments.size(), 3u);
  EXPECT_EQ(segments[0].waypoints.size(), 2u);
  EXPECT_EQ(segments[1].waypoints.size(), 4u);
  EXPECT_EQ(segments[2].waypoints.size(), 3u);
  for (const MotionSegment& segment : segments) {
    EXPECT_EQ(segment.risk, RiskLevel::HIGH);
  }
}

TEST(BaselineContract, ForcedStrictDoesNotOverwriteAssessedRiskLabels) {
  std::vector<MotionSegment> assessed(3);
  assessed[0].risk = RiskLevel::LOW;
  assessed[1].risk = RiskLevel::MEDIUM;
  assessed[2].risk = RiskLevel::HIGH;
  std::vector<MotionSegment> solver_segments = assessed;

  ForceAllSegmentsStrict(&solver_segments);

  EXPECT_EQ(assessed[0].risk, RiskLevel::LOW);
  EXPECT_EQ(assessed[1].risk, RiskLevel::MEDIUM);
  EXPECT_EQ(assessed[2].risk, RiskLevel::HIGH);
  for (const MotionSegment& segment : solver_segments) {
    EXPECT_EQ(segment.risk, RiskLevel::HIGH);
  }
}

TEST(BaselineContract, ForcedStrictAlsoOverridesRecoveryAndRefinementLevels) {
  RuntimeExperimentOptions options;
  EXPECT_EQ(EffectiveSolverLevel(options, RiskLevel::LOW), RiskLevel::LOW);
  EXPECT_EQ(EffectiveSolverLevel(options, RiskLevel::MEDIUM), RiskLevel::MEDIUM);

  options.force_all_strict = true;
  EXPECT_EQ(EffectiveSolverLevel(options, RiskLevel::LOW), RiskLevel::HIGH);
  EXPECT_EQ(EffectiveSolverLevel(options, RiskLevel::MEDIUM), RiskLevel::HIGH);
  EXPECT_EQ(EffectiveSolverLevel(options, RiskLevel::HIGH), RiskLevel::HIGH);
}

TEST(BaselineContract, PaperComparisonMethodsSelectOnlyDeclaredBranches) {
  RuntimeExperimentOptions base;
  base.total_time_budget_sec = 8.0;
  base.random_seed = 7;

  RuntimeExperimentOptions adaptive = base;
  ASSERT_TRUE(ConfigureRuntimeMethod("current_full", &adaptive));
  EXPECT_FALSE(adaptive.force_all_low);
  EXPECT_FALSE(adaptive.force_all_strict);
  EXPECT_TRUE(adaptive.recovery_enabled);
  EXPECT_TRUE(adaptive.candidate_switch);

  RuntimeExperimentOptions r2 = base;
  ASSERT_TRUE(ConfigureRuntimeMethod("full_r2", &r2));
  EXPECT_TRUE(r2.force_all_low);
  EXPECT_FALSE(r2.hybrid_frontend_fallback);
  EXPECT_FALSE(r2.recovery_enabled);
  EXPECT_FALSE(InitialLevelEscalationAllowed(r2));

  RuntimeExperimentOptions strict = base;
  ASSERT_TRUE(ConfigureRuntimeMethod("full_strict", &strict));
  EXPECT_TRUE(strict.force_all_strict);
  EXPECT_TRUE(strict.force_full_svsdf);
  EXPECT_TRUE(strict.recovery_enabled);

  RuntimeExperimentOptions hybrid = base;
  ASSERT_TRUE(ConfigureRuntimeMethod("hybrid_astar", &hybrid));
  EXPECT_TRUE(hybrid.hybrid_astar_only);
  EXPECT_FALSE(hybrid.recovery_enabled);

  EXPECT_DOUBLE_EQ(adaptive.total_time_budget_sec, r2.total_time_budget_sec);
  EXPECT_EQ(adaptive.random_seed, hybrid.random_seed);

  RuntimeExperimentOptions invalid = base;
  EXPECT_FALSE(ConfigureRuntimeMethod("unknown", &invalid));
  EXPECT_EQ(invalid.method_name, base.method_name);
}

TEST(HybridAStarContract, FailedSearchExposesOnlyDiagnosticPartialChain) {
  nav_msgs::OccupancyGrid occupancy;
  occupancy.info.resolution = 0.10;
  occupancy.info.width = 80;
  occupancy.info.height = 30;
  occupancy.data.assign(occupancy.info.width * occupancy.info.height, 0);
  for (unsigned int y = 0; y < occupancy.info.height; ++y) {
    occupancy.data[y * occupancy.info.width + 40] = 100;
  }
  nav_msgs::OccupancyGrid::Ptr occupancy_ptr(
      new nav_msgs::OccupancyGrid(occupancy));

  GridMap map;
  map.fromOccupancyGrid(occupancy_ptr);
  FootprintModel footprint;
  footprint.setPolygon({Eigen::Vector2d(-0.15, -0.15),
                        Eigen::Vector2d(0.15, -0.15),
                        Eigen::Vector2d(0.15, 0.15),
                        Eigen::Vector2d(-0.15, 0.15)});
  CollisionChecker checker;
  checker.init(map, footprint, 18);

  HybridAStarParams params;
  params.step_size = 0.25;
  params.wheel_base = 0.50;
  params.max_expansions = 5000;
  params.fast_pass_max_expansions = 500;
  HybridAStarPlanner planner;
  planner.init(map, checker, params);

  std::vector<SE2State> executable_path;
  std::vector<SE2State> diagnostic_partial;
  EXPECT_FALSE(planner.plan(SE2State(1.0, 1.5, 0.0),
                            SE2State(7.0, 1.5, 0.0), executable_path,
                            2.0, &diagnostic_partial));
  EXPECT_TRUE(executable_path.empty());
  ASSERT_GE(diagnostic_partial.size(), 2u);
  EXPECT_LT(diagnostic_partial.back().x, 4.0);
  EXPECT_GT(diagnostic_partial.back().x, diagnostic_partial.front().x);
}

TEST(LocalReuseContract, UsesActuallyReplannedArcLength) {
  EXPECT_DOUBLE_EQ(0.75, LocalTrajectoryReuseRatio(8.0, 2.0));
  EXPECT_DOUBLE_EQ(0.0, LocalTrajectoryReuseRatio(8.0, 20.0));
  EXPECT_TRUE(std::isnan(LocalTrajectoryReuseRatio(0.0, 0.0)));
  EXPECT_TRUE(std::isnan(LocalTrajectoryReuseRatio(8.0, -1.0)));
}

TEST(FootprintValidationContract, AdaptiveSamplingFindsRawGridCollision) {
  nav_msgs::OccupancyGrid occupancy;
  occupancy.info.resolution = 0.05;
  occupancy.info.width = 120;
  occupancy.info.height = 120;
  occupancy.info.origin.position.x = 0.0;
  occupancy.info.origin.position.y = 0.0;
  occupancy.data.assign(occupancy.info.width * occupancy.info.height, 0);
  const int occupied_x = 50;
  const int occupied_y = 50;
  occupancy.data[occupied_y * occupancy.info.width + occupied_x] = 100;
  nav_msgs::OccupancyGrid::Ptr occupancy_ptr(
      new nav_msgs::OccupancyGrid(occupancy));

  GridMap map;
  map.fromOccupancyGrid(occupancy_ptr);
  FootprintModel footprint;
  footprint.setPolygon({Eigen::Vector2d(-0.02, -0.02),
                        Eigen::Vector2d(0.02, -0.02),
                        Eigen::Vector2d(0.02, 0.02),
                        Eigen::Vector2d(-0.02, 0.02)});

  Trajectory trajectory;
  Eigen::Matrix<double, 2, 6> position =
      Eigen::Matrix<double, 2, 6>::Zero();
  position(0, 0) = 1.0;
  position(0, 1) = 3.0;
  position(1, 0) = 2.525;
  trajectory.pos_pieces.push_back(MakePositionPiece(1.0, position));
  trajectory.yaw_pieces.push_back(MakeYawPiece(
      1.0, Eigen::Matrix<double, 1, 6>::Zero()));

  TrajectoryFootprintValidationConfig config;
  config.time_step_sec = 1.0;
  config.max_translation_step_m = 0.01;
  config.max_yaw_step_rad = 1.0;
  config.footprint_boundary_step_m = 0.01;
  config.footprint_interior_step_m = 0.01;
  const TrajectoryFootprintValidationResult result =
      ValidateTrajectoryFootprint(map, footprint, trajectory, config);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ("occupied_grid", result.collision_type);
  EXPECT_GT(result.sample_times.size(), 2u);
  EXPECT_TRUE(std::isfinite(result.collision_time_sec));
}

TEST(StitchContract, PreservesEndpointsAndConditionsCrossSegmentBoundary) {
  Trajectory first;
  Eigen::Matrix<double, 2, 6> first_position =
      Eigen::Matrix<double, 2, 6>::Zero();
  first_position(0, 1) = 1.0;  // x(t) = t
  first.pos_pieces.push_back(MakePositionPiece(1.0, first_position));
  Eigen::Matrix<double, 1, 6> first_yaw =
      Eigen::Matrix<double, 1, 6>::Zero();
  first_yaw(0, 1) = 0.2;
  first.yaw_pieces.push_back(MakeYawPiece(1.0, first_yaw));

  Trajectory second;
  Eigen::Matrix<double, 2, 6> second_position =
      Eigen::Matrix<double, 2, 6>::Zero();
  second_position(0, 0) = 1.0;
  second_position(0, 1) = 2.0;  // deliberately mismatched start velocity
  second_position(1, 1) = 0.5;
  second.pos_pieces.push_back(MakePositionPiece(1.0, second_position));
  Eigen::Matrix<double, 1, 6> second_yaw =
      Eigen::Matrix<double, 1, 6>::Zero();
  second_yaw(0, 0) = 0.4;       // deliberately mismatched start yaw
  second_yaw(0, 1) = 0.3;
  second.yaw_pieces.push_back(MakeYawPiece(1.0, second_yaw));

  std::vector<MotionSegment> segments(2);
  segments[0].risk = RiskLevel::LOW;
  segments[1].risk = RiskLevel::MEDIUM;

  TrajectoryOptimizer optimizer;
  const Trajectory stitched = optimizer.stitch(segments, {first, second});
  ASSERT_EQ(2u, stitched.pos_pieces.size());
  ASSERT_EQ(2u, stitched.yaw_pieces.size());

  const PolyPiece& left = stitched.pos_pieces[0];
  const PolyPiece& right = stitched.pos_pieces[1];
  EXPECT_NEAR(0.0, (left.evaluate(left.duration) - right.evaluate(0.0)).norm(),
              1e-10);
  EXPECT_NEAR(0.0, (left.velocity(left.duration) - right.velocity(0.0)).norm(),
              1e-10);
  EXPECT_NEAR(
      0.0,
      (left.acceleration(left.duration) - right.acceleration(0.0)).norm(),
      1e-10);

  const YawPolyPiece& left_yaw_piece = stitched.yaw_pieces[0];
  const YawPolyPiece& right_yaw_piece = stitched.yaw_pieces[1];
  EXPECT_NEAR(
      0.0,
      normalizeAngle(left_yaw_piece.evaluate(left_yaw_piece.duration) -
                     right_yaw_piece.evaluate(0.0)),
      1e-10);
  EXPECT_NEAR(left_yaw_piece.velocity(left_yaw_piece.duration),
              right_yaw_piece.velocity(0.0), 1e-10);

  EXPECT_NEAR(0.0, stitched.sample(0.0).x, 1e-10);
  const SE2State terminal = stitched.sample(stitched.totalDuration());
  EXPECT_NEAR(3.0, terminal.x, 1e-9);
  EXPECT_NEAR(0.5, terminal.y, 1e-9);
  EXPECT_NEAR(0.7, terminal.yaw, 1e-9);
}

TEST(StitchContract, ReplacesZeroSpeedRiskBoundaryCuspWithSharedTangent) {
  // Both input segments use the quintic smooth-step 10t^3-15t^4+6t^5, so
  // their boundary velocities and accelerations are exactly zero.  Direct
  // concatenation would therefore pass a C2 jump check while retaining a
  // geometric 90-degree cusp.
  Trajectory first;
  Eigen::Matrix<double, 2, 6> first_position =
      Eigen::Matrix<double, 2, 6>::Zero();
  first_position(0, 3) = 10.0;
  first_position(0, 4) = -15.0;
  first_position(0, 5) = 6.0;
  first.pos_pieces.push_back(MakePositionPiece(1.0, first_position));
  first.yaw_pieces.push_back(MakeYawPiece(
      1.0, Eigen::Matrix<double, 1, 6>::Zero()));

  Trajectory second;
  Eigen::Matrix<double, 2, 6> second_position =
      Eigen::Matrix<double, 2, 6>::Zero();
  second_position(0, 0) = 1.0;
  second_position(1, 3) = 10.0;
  second_position(1, 4) = -15.0;
  second_position(1, 5) = 6.0;
  second.pos_pieces.push_back(MakePositionPiece(1.0, second_position));
  second.yaw_pieces.push_back(MakeYawPiece(
      1.0, Eigen::Matrix<double, 1, 6>::Zero()));

  std::vector<MotionSegment> segments(2);
  segments[0].risk = RiskLevel::HIGH;
  segments[1].risk = RiskLevel::LOW;

  TrajectoryOptimizer optimizer;
  const Trajectory stitched = optimizer.stitch(segments, {first, second});
  ASSERT_EQ(2u, stitched.pos_pieces.size());

  const PolyPiece& left = stitched.pos_pieces[0];
  const PolyPiece& right = stitched.pos_pieces[1];
  const Eigen::Vector2d left_velocity = left.velocity(left.duration);
  const Eigen::Vector2d right_velocity = right.velocity(0.0);
  EXPECT_GT(left_velocity.norm(), 0.10);
  EXPECT_GT(left_velocity.x(), 0.0);
  EXPECT_GT(left_velocity.y(), 0.0);
  EXPECT_NEAR(0.0, (left_velocity - right_velocity).norm(), 1e-10);
  EXPECT_NEAR(
      0.0,
      (left.acceleration(left.duration) - right.acceleration(0.0)).norm(),
      1e-10);
  EXPECT_NEAR(0.0, left.acceleration(left.duration).norm(), 1e-9);

  EXPECT_NEAR(0.0, stitched.sample(0.0).x, 1e-10);
  EXPECT_NEAR(0.0, stitched.sample(0.0).y, 1e-10);
  const SE2State terminal = stitched.sample(stitched.totalDuration());
  EXPECT_NEAR(1.0, terminal.x, 1e-9);
  EXPECT_NEAR(1.0, terminal.y, 1e-9);
}

TEST(ContinuousRiskContract, UncertaintyErosionCanTriggerStrictLevel) {
  nav_msgs::OccupancyGrid occupancy;
  occupancy.info.resolution = 0.05;
  occupancy.info.width = 200;
  occupancy.info.height = 200;
  occupancy.data.assign(occupancy.info.width * occupancy.info.height, 0);
  for (unsigned int x = 0; x < occupancy.info.width; ++x) {
    occupancy.data[x] = 100;
    occupancy.data[(occupancy.info.height - 1) * occupancy.info.width + x] = 100;
  }
  for (unsigned int y = 0; y < occupancy.info.height; ++y) {
    occupancy.data[y * occupancy.info.width] = 100;
    occupancy.data[y * occupancy.info.width + occupancy.info.width - 1] = 100;
  }
  nav_msgs::OccupancyGrid::Ptr occupancy_ptr(
      new nav_msgs::OccupancyGrid(occupancy));

  GridMap map;
  map.fromOccupancyGrid(occupancy_ptr);
  FootprintModel footprint;
  footprint.setPolygon({Eigen::Vector2d(-0.10, -0.10),
                        Eigen::Vector2d(0.10, -0.10),
                        Eigen::Vector2d(0.10, 0.10),
                        Eigen::Vector2d(-0.10, 0.10)});
  footprint.setInscribedRadius(0.10);
  CollisionChecker checker;
  checker.init(map, footprint, 18);

  ContinuousRiskParams params;
  params.low_threshold = 0.25;
  params.high_threshold = 0.60;
  params.localization_sigma = 0.0;
  params.localization_sigma_scale = 0.0;
  params.control_error = 100.0;
  params.map_error = 0.0;
  params.shape_error = 0.0;

  TopoPath path;
  path.waypoints.emplace_back(Eigen::Vector2d(2.0, 2.0), 0.0);
  path.waypoints.emplace_back(Eigen::Vector2d(7.0, 2.0), 0.0);
  path.computeLength();

  SE2SequenceGenerator generator;
  generator.init(map, checker, 0.20, 3, params);
  SE2GenerationDiagnostics diagnostics;
  const std::vector<MotionSegment> segments = generator.generate(
      path, SE2State(2.0, 2.0, 0.0), SE2State(7.0, 2.0, 0.0),
      &diagnostics);

  ASSERT_FALSE(segments.empty());
  EXPECT_GT(diagnostics.risk_sample_count, 1);
  for (const MotionSegment& segment : segments) {
    EXPECT_EQ(RiskLevel::HIGH, segment.risk);
    ASSERT_FALSE(segment.risk_profile.empty());
    for (const RiskSample& sample : segment.risk_profile) {
      EXPECT_DOUBLE_EQ(100.0, sample.uncertainty_erosion);
      EXPECT_DOUBLE_EQ(1.0, sample.score);
      EXPECT_EQ(RiskLevel::HIGH, sample.level);
    }
  }
}

TEST(AckermannPreSmoothingContract, ReducesCornerHeadingJumpWithoutCollision) {
  nav_msgs::OccupancyGrid occupancy;
  occupancy.info.resolution = 0.05;
  occupancy.info.width = 220;
  occupancy.info.height = 220;
  occupancy.data.assign(occupancy.info.width * occupancy.info.height, 0);
  for (unsigned int x = 0; x < occupancy.info.width; ++x) {
    occupancy.data[x] = 100;
    occupancy.data[(occupancy.info.height - 1) * occupancy.info.width + x] =
        100;
  }
  for (unsigned int y = 0; y < occupancy.info.height; ++y) {
    occupancy.data[y * occupancy.info.width] = 100;
    occupancy.data[y * occupancy.info.width + occupancy.info.width - 1] = 100;
  }
  nav_msgs::OccupancyGrid::Ptr occupancy_ptr(
      new nav_msgs::OccupancyGrid(occupancy));

  GridMap map;
  map.fromOccupancyGrid(occupancy_ptr);
  FootprintModel footprint;
  footprint.setPolygon({Eigen::Vector2d(-0.20, -0.12),
                        Eigen::Vector2d(0.20, -0.12),
                        Eigen::Vector2d(0.20, 0.12),
                        Eigen::Vector2d(-0.20, 0.12)});
  footprint.setInscribedRadius(0.12);
  CollisionChecker checker;
  checker.init(map, footprint, 36);

  ContinuousRiskParams params;
  params.enable_three_level = true;
  params.low_threshold = 0.99;
  params.high_threshold = 1.0;
  params.localization_sigma = 0.0;
  params.localization_sigma_scale = 0.0;
  params.control_error = 0.0;
  params.map_error = 0.0;
  params.shape_error = 0.0;

  TopoPath path;
  path.waypoints.emplace_back(Eigen::Vector2d(2.0, 2.0), 0.0);
  path.waypoints.emplace_back(Eigen::Vector2d(7.0, 2.0), 0.0);
  path.waypoints.emplace_back(Eigen::Vector2d(7.0, 7.0), 0.5 * M_PI);
  path.computeLength();

  const auto flatten = [](const std::vector<MotionSegment>& segments) {
    std::vector<SE2State> states;
    for (const MotionSegment& segment : segments) {
      for (const SE2State& state : segment.waypoints) {
        if (!states.empty() &&
            (states.back().position() - state.position()).norm() < 1e-8) {
          continue;
        }
        states.push_back(state);
      }
    }
    return states;
  };
  const auto maximum_heading_jump = [](const std::vector<SE2State>& states) {
    double maximum = 0.0;
    for (size_t i = 1; i + 1 < states.size(); ++i) {
      const Eigen::Vector2d incoming =
          states[i].position() - states[i - 1].position();
      const Eigen::Vector2d outgoing =
          states[i + 1].position() - states[i].position();
      if (incoming.norm() < 1e-8 || outgoing.norm() < 1e-8) continue;
      maximum = std::max(
          maximum,
          std::abs(normalizeAngle(std::atan2(outgoing.y(), outgoing.x()) -
                                  std::atan2(incoming.y(), incoming.x()))));
    }
    return maximum;
  };

  SE2SequenceGenerator raw_generator;
  raw_generator.init(map, checker, 0.15, 3, params, 0, 0.5);
  const std::vector<SE2State> raw_states = flatten(raw_generator.generate(
      path, SE2State(2.0, 2.0, 0.0), SE2State(7.0, 7.0, 0.5 * M_PI)));

  SE2SequenceGenerator smoothed_generator;
  smoothed_generator.init(map, checker, 0.15, 3, params, 5, 0.5);
  const std::vector<SE2State> smoothed_states = flatten(
      smoothed_generator.generate(path, SE2State(2.0, 2.0, 0.0),
                                  SE2State(7.0, 7.0, 0.5 * M_PI)));

  ASSERT_GT(raw_states.size(), 3u);
  ASSERT_EQ(raw_states.size(), smoothed_states.size());
  EXPECT_GT(maximum_heading_jump(raw_states), 1.0);
  EXPECT_LT(maximum_heading_jump(smoothed_states),
            0.75 * maximum_heading_jump(raw_states));
  for (const SE2State& state : smoothed_states) {
    EXPECT_TRUE(checker.isFree(state));
  }
}

}  // namespace
}  // namespace isweep_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
