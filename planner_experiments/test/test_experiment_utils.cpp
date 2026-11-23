#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include "isweep_planner/env/footprint_model.h"
#include "planner_experiments/experiment_types.hpp"
#include "planner_experiments/geometry_consistency_checker.hpp"
#include "planner_experiments/metrics_collector.hpp"
#include "planner_experiments/result_writer.hpp"
#include "planner_experiments/trajectory_validator.hpp"

namespace pe = planner_experiments;

namespace {

nav_msgs::OccupancyGrid EmptyMap() {
  nav_msgs::OccupancyGrid map;
  map.info.resolution = 0.05;
  map.info.width = 200;
  map.info.height = 200;
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;
  map.data.assign(map.info.width * map.info.height, 0);
  return map;
}

isweep_planner::FootprintModel SmallFootprint() {
  isweep_planner::FootprintModel footprint;
  footprint.setPolygon({{-0.1, -0.1}, {0.1, -0.1},
                        {0.1, 0.1}, {-0.1, 0.1}});
  return footprint;
}

}  // namespace

TEST(YawUtilities, UnwrapCrossesPiContinuously) {
  const std::vector<double> values = pe::UnwrapYaw({3.10, -3.10, -3.00});
  ASSERT_EQ(values.size(), 3u);
  EXPECT_NEAR(values[1] - values[0], 0.083185307179586, 1e-9);
  EXPECT_NEAR(values[2] - values[1], 0.10, 1e-9);
}

TEST(YawUtilities, CircularBinDistance) {
  EXPECT_EQ(pe::CircularBinDistance(0, 17, 18), 1);
  EXPECT_EQ(pe::CircularBinDistance(2, 10, 18), 8);
}

TEST(YawUtilities, LargestCircularSafeInterval) {
  EXPECT_EQ(pe::LargestContiguousSafeYawBins({16, 17, 0, 1, 5}, 18), 4);
  EXPECT_EQ(pe::LargestContiguousSafeYawBins({}, 18), 0);
}

TEST(Footprint, RotateAndDenseSample) {
  isweep_planner::FootprintModel footprint;
  footprint.setPolygon({{-1.0, -0.5}, {1.0, -0.5}, {1.0, 0.5}, {-1.0, 0.5}});
  const auto rotated = footprint.rotatedVertices(isweep_planner::kPi / 2.0);
  EXPECT_NEAR(rotated[0].x(), 0.5, 1e-12);
  EXPECT_NEAR(rotated[0].y(), -1.0, 1e-12);
  EXPECT_GT(footprint.denseBodySamples(0.1, 0.1).size(), 100u);
}

TEST(Trajectory, LengthAndDynamicsSampling) {
  isweep_planner::Trajectory trajectory;
  isweep_planner::PolyPiece piece;
  piece.duration = 1.0;
  piece.coeffs(0, 1) = 1.0;
  trajectory.pos_pieces.push_back(piece);
  isweep_planner::YawPolyPiece yaw;
  yaw.duration = 1.0;
  trajectory.yaw_pieces.push_back(yaw);
  EXPECT_NEAR(trajectory.sampleVelocity(0.5).norm(), 1.0, 1e-12);
  std::vector<pe::TrajectorySample> samples(2);
  samples[1].x = 1.0;
  EXPECT_NEAR(pe::TrajectoryValidator::TrajectoryLength(samples), 1.0, 1e-12);
}

TEST(TrajectoryValidator, DetectsDynamicsViolation) {
  const nav_msgs::OccupancyGrid occupancy = EmptyMap();
  nav_msgs::OccupancyGrid::Ptr pointer(new nav_msgs::OccupancyGrid(occupancy));
  isweep_planner::GridMap map;
  map.fromOccupancyGrid(pointer);
  isweep_planner::Trajectory trajectory;
  isweep_planner::PolyPiece piece;
  piece.duration = 1.0;
  piece.coeffs(0, 0) = 1.0;
  piece.coeffs(1, 0) = 1.0;
  piece.coeffs(0, 1) = 2.0;
  trajectory.pos_pieces.push_back(piece);
  isweep_planner::YawPolyPiece yaw;
  yaw.duration = 1.0;
  trajectory.yaw_pieces.push_back(yaw);
  pe::DynamicsLimits limits;
  limits.max_linear_velocity = 1.0;
  limits.max_linear_acceleration = 10.0;
  limits.max_angular_velocity = 10.0;
  limits.max_angular_acceleration = 10.0;
  pe::ValidationConfig config;
  config.time_step_sec = 0.1;
  config.max_translation_step_m = 0.1;
  const pe::ValidationResult result = pe::TrajectoryValidator().Validate(
      occupancy, map, SmallFootprint(), trajectory, limits, config);
  EXPECT_TRUE(result.external_footprint_valid);
  EXPECT_FALSE(result.dynamics_valid);
  EXPECT_GT(result.velocity_violation_count, 0);
  EXPECT_NEAR(result.max_linear_velocity, 2.0, 1e-9);
}

TEST(TrajectoryValidator, DetectsPieceBoundaryJump) {
  const nav_msgs::OccupancyGrid occupancy = EmptyMap();
  nav_msgs::OccupancyGrid::Ptr pointer(new nav_msgs::OccupancyGrid(occupancy));
  isweep_planner::GridMap map;
  map.fromOccupancyGrid(pointer);
  isweep_planner::Trajectory trajectory;
  for (double start : {1.0, 3.0}) {
    isweep_planner::PolyPiece piece;
    piece.duration = 1.0;
    piece.coeffs(0, 0) = start;
    piece.coeffs(1, 0) = 1.0;
    piece.coeffs(0, 1) = 1.0;
    trajectory.pos_pieces.push_back(piece);
    isweep_planner::YawPolyPiece yaw;
    yaw.duration = 1.0;
    trajectory.yaw_pieces.push_back(yaw);
  }
  pe::DynamicsLimits limits;
  limits.max_linear_velocity = 10.0;
  limits.max_linear_acceleration = 10.0;
  limits.max_angular_velocity = 10.0;
  limits.max_angular_acceleration = 10.0;
  pe::ValidationConfig config;
  config.time_step_sec = 0.1;
  config.max_translation_step_m = 0.1;
  const pe::ValidationResult result = pe::TrajectoryValidator().Validate(
      occupancy, map, SmallFootprint(), trajectory, limits, config);
  EXPECT_FALSE(result.boundary_continuous);
  EXPECT_NEAR(result.max_position_jump_m, 1.0, 1e-9);
  EXPECT_EQ(result.worst_boundary_index, 0);
}

TEST(TrajectoryValidator, RejectsTrajectoryThatDoesNotReachRequestedGoal) {
  const nav_msgs::OccupancyGrid occupancy = EmptyMap();
  nav_msgs::OccupancyGrid::Ptr pointer(new nav_msgs::OccupancyGrid(occupancy));
  isweep_planner::GridMap map;
  map.fromOccupancyGrid(pointer);
  isweep_planner::Trajectory trajectory;
  isweep_planner::PolyPiece piece;
  piece.duration = 1.0;
  piece.coeffs(0, 0) = 1.0;
  piece.coeffs(1, 0) = 1.0;
  piece.coeffs(0, 1) = 2.0;
  trajectory.pos_pieces.push_back(piece);
  isweep_planner::YawPolyPiece yaw;
  yaw.duration = 1.0;
  trajectory.yaw_pieces.push_back(yaw);
  pe::DynamicsLimits limits;
  limits.max_linear_velocity = 10.0;
  limits.max_linear_acceleration = 10.0;
  limits.max_angular_velocity = 10.0;
  limits.max_angular_acceleration = 10.0;
  pe::ValidationConfig config;
  config.time_step_sec = 0.1;
  config.max_translation_step_m = 0.1;
  const pe::ValidationResult result = pe::TrajectoryValidator().Validate(
      occupancy, map, SmallFootprint(), trajectory, limits, config,
      isweep_planner::SE2State(1.0, 1.0, 0.0),
      isweep_planner::SE2State(4.0, 1.0, 0.0));
  EXPECT_TRUE(result.external_footprint_valid);
  EXPECT_TRUE(result.dynamics_valid);
  EXPECT_TRUE(result.endpoint_check_performed);
  EXPECT_TRUE(result.start_matches);
  EXPECT_FALSE(result.goal_reached);
  EXPECT_FALSE(result.endpoints_match);
  EXPECT_NEAR(result.goal_position_error_m, 1.0, 1e-9);
}

TEST(ResultWriter, StableRunKeyAndJsonNull) {
  pe::PlanningRequest request;
  request.method_name = "current_full";
  request.map_hash = "abc";
  request.robot_shape_name = "t_shape";
  request.task_id = "task_001";
  EXPECT_EQ(pe::ResultWriter::StableRunKey(request, "config"),
            pe::ResultWriter::StableRunKey(request, "config"));
  pe::ExperimentMetrics metrics;
  const auto json = pe::ToJson(metrics);
  EXPECT_EQ(json["schema_version"], 4);
  EXPECT_TRUE(json["t_esdf_ms"].is_null());
}

TEST(Serialization, SeparatesEscalationFallbackRecoveryAndSvsdfSources) {
  pe::ExperimentMetrics metrics;
  metrics.initial_level_upgrade_attempt_count = 2;
  metrics.frontend_fallback_attempt_count = 1;
  metrics.validation_recovery_attempt_count = 1;
  metrics.validation_recovery_success_count = 1;
  metrics.post_incumbent_svsdf_call_count = 2;
  metrics.post_incumbent_svsdf_budget_reject_count = 1;
  metrics.tail_refinement_svsdf_budget_reject_count = 1;
  metrics.external_equivalent_gate_attempt_count = 3;
  metrics.external_equivalent_gate_reject_count = 1;
  metrics.full_svsdf_call_count = 3;
  metrics.full_svsdf_initial_assigned_call_count = 1;
  metrics.full_svsdf_initial_escalation_call_count = 1;
  metrics.full_svsdf_validation_recovery_call_count = 1;
  const auto json = pe::ToJson(metrics);
  EXPECT_EQ(json["schema_version"], 4);
  EXPECT_EQ(json["initial_level_upgrade_attempt_count"], 2);
  EXPECT_EQ(json["frontend_fallback_attempt_count"], 1);
  EXPECT_EQ(json["validation_recovery_attempt_count"], 1);
  EXPECT_EQ(json["validation_recovery_success_count"], 1);
  EXPECT_EQ(json["post_incumbent_svsdf_call_count"], 2);
  EXPECT_EQ(json["post_incumbent_svsdf_budget_reject_count"], 1);
  EXPECT_EQ(json["tail_refinement_svsdf_budget_reject_count"], 1);
  EXPECT_EQ(json["external_equivalent_gate_attempt_count"], 3);
  EXPECT_EQ(json["external_equivalent_gate_reject_count"], 1);
  EXPECT_EQ(json["full_svsdf_initial_assigned_call_count"], 1);
  EXPECT_EQ(json["full_svsdf_initial_escalation_call_count"], 1);
  EXPECT_EQ(json["full_svsdf_validation_recovery_call_count"], 1);
}

TEST(Serialization, IncumbentEventCarriesQualityTimeVector) {
  pe::IncumbentEventMetric event;
  event.run_id = "run";
  event.update_index = 2;
  event.elapsed_ms = 125.0;
  event.minimum_clearance_m = 0.18;
  event.trajectory_length_m = 4.2;
  event.high_segment_count = 1;
  const auto json = pe::ToJson(event);
  EXPECT_EQ(json["run_id"], "run");
  EXPECT_EQ(json["update_index"], 2);
  EXPECT_DOUBLE_EQ(json["elapsed_ms"], 125.0);
  EXPECT_DOUBLE_EQ(json["trajectory_length_m"], 4.2);
  EXPECT_TRUE(json["internally_validated"]);
}

TEST(Serialization, RiskGroundTruthCarriesThreeLevelIndependentLabel) {
  pe::RiskGroundTruthMetric metric;
  metric.run_id = "run";
  metric.segment_index = 3;
  metric.assigned_risk_label = "HIGH";
  metric.maximum_risk_score = 0.81;
  metric.low_solver_attempted = true;
  metric.medium_solver_attempted = true;
  metric.strict_solver_attempted = true;
  metric.full_svsdf_used = true;
  metric.full_svsdf_success = true;
  metric.label_available = true;
  metric.l3_required = true;
  metric.strict_required = true;
  metric.minimum_required_level = "HIGH";
  metric.outcome = "strict_required";
  const auto json = pe::ToJson(metric);
  EXPECT_EQ(json["segment_index"], 3);
  EXPECT_EQ(json["outcome"], "strict_required");
  EXPECT_TRUE(json["full_svsdf_used"]);
  EXPECT_TRUE(json["label_available"]);
  EXPECT_TRUE(json.contains("low_validation"));
  EXPECT_TRUE(json.contains("medium_validation"));
  EXPECT_TRUE(json.contains("strict_validation"));
}

TEST(RiskGroundTruthContract, DistinguishesRequiredOptimizationLevel) {
  pe::RiskGroundTruthMetric metric;
  metric.low_solver_attempted = true;
  metric.medium_solver_attempted = true;
  metric.strict_solver_attempted = true;
  metric.full_svsdf_used = true;
  metric.full_svsdf_success = true;

  metric.medium_executable = true;
  metric.strict_executable = true;
  pe::FinalizeRiskGroundTruthLabel(&metric);
  EXPECT_TRUE(metric.label_available);
  EXPECT_TRUE(metric.l2_required);
  EXPECT_FALSE(metric.l3_required);
  EXPECT_EQ("MEDIUM", metric.minimum_required_level);

  metric.medium_executable = false;
  pe::FinalizeRiskGroundTruthLabel(&metric);
  EXPECT_FALSE(metric.l2_required);
  EXPECT_TRUE(metric.l3_required);
  EXPECT_TRUE(metric.strict_required);
  EXPECT_EQ("HIGH", metric.minimum_required_level);

  metric.low_executable = true;
  pe::FinalizeRiskGroundTruthLabel(&metric);
  EXPECT_EQ("LOW", metric.minimum_required_level);
  EXPECT_FALSE(metric.strict_required);
}

TEST(IncumbentContract, DeadlineFlagDoesNotInvalidateReturnedTrajectory) {
  pe::PlanningResult result;
  result.planner_returned = true;
  result.timeout = true;
  pe::ValidationResult validation;
  validation.external_footprint_valid = true;
  validation.dynamics_valid = true;
  validation.boundary_continuous = true;
  validation.endpoint_check_performed = true;
  validation.start_matches = true;
  validation.goal_reached = true;
  validation.endpoints_match = true;
  pe::MetricsCollector::ApplyValidation(validation, &result);
  EXPECT_TRUE(result.metrics.executable);
  EXPECT_EQ(pe::PlannerStatus::EXECUTABLE, result.status);
  EXPECT_TRUE(result.timeout);
}

TEST(IncumbentContract, MissingGoalInvalidatesReturnedTrajectory) {
  pe::PlanningResult result;
  result.planner_returned = true;
  pe::ValidationResult validation;
  validation.external_footprint_valid = true;
  validation.dynamics_valid = true;
  validation.boundary_continuous = true;
  validation.endpoint_check_performed = true;
  validation.start_matches = true;
  validation.goal_reached = false;
  validation.endpoints_match = false;
  pe::MetricsCollector::ApplyValidation(validation, &result);
  EXPECT_FALSE(result.metrics.executable);
  EXPECT_EQ("endpoint_validation", result.failure_stage);
  EXPECT_EQ(pe::PlannerStatus::DYNAMICS_VALIDATED, result.status);
}

TEST(GeometryConsistency, MatchingRectangleMesh) {
  const std::string package = ros::package::getPath("planner_experiments");
  pe::GeometryCheckConfig config;
  const auto result = pe::GeometryConsistencyChecker().Check(
      {{-0.60, -0.25}, {0.60, -0.25}, {0.60, 0.25}, {-0.60, 0.25}},
      package + "/meshes/long_rectangle.obj", config);
  EXPECT_TRUE(result.consistency_passed) << result.failure_reason;
  EXPECT_NEAR(result.polygon_area, result.mesh_projected_area, 1e-9);
}

TEST(GeometryConsistency, RejectsMismatchedPolygon) {
  const std::string package = ros::package::getPath("planner_experiments");
  pe::GeometryCheckConfig config;
  const auto result = pe::GeometryConsistencyChecker().Check(
      {{-1.2, -0.5}, {1.2, -0.5}, {1.2, 0.5}, {-1.2, 0.5}},
      package + "/meshes/long_rectangle.obj", config);
  EXPECT_FALSE(result.consistency_passed);
  EXPECT_FALSE(result.failure_reason.empty());
}

TEST(GeometryConsistency, AllShippedRobotConfigurationsMatchTheirMeshes) {
  const std::string package = ros::package::getPath("planner_experiments");
  pe::GeometryCheckConfig check;
  for (const std::string& name : {"circle", "long_rectangle", "t_shape", "l_shape"}) {
    const YAML::Node robot = YAML::LoadFile(
        package + "/config/robot_shapes/" + name + ".yaml");
    std::vector<Eigen::Vector2d> polygon;
    for (const YAML::Node& vertex : robot["footprint"]) {
      polygon.emplace_back(vertex[0].as<double>(), vertex[1].as<double>());
    }
    const std::string uri = robot["mesh_path"].as<std::string>();
    const std::string prefix = "package://planner_experiments/";
    ASSERT_EQ(uri.compare(0, prefix.size(), prefix), 0u);
    const auto result = pe::GeometryConsistencyChecker().Check(
        polygon, package + "/" + uri.substr(prefix.size()), check);
    EXPECT_TRUE(result.consistency_passed) << name << ": " << result.failure_reason;
  }
}

TEST(Serialization, StateAndSegmentHaveRunId) {
  pe::StateMetric state;
  state.run_id = "run";
  pe::SegmentMetric segment;
  segment.run_id = "run";
  EXPECT_EQ(pe::ToJson(state)["run_id"], "run");
  EXPECT_EQ(pe::ToJson(segment)["run_id"], "run");
}

TEST(Serialization, WritesReadableJsonAndCsv) {
  pe::PlanningResult result;
  result.metrics.run_id = "serialization_test";
  pe::TrajectorySample sample;
  sample.t = 0.25;
  sample.x = 1.5;
  result.metrics.validation.samples.push_back(sample);
  pe::IncumbentEventMetric event;
  event.run_id = result.metrics.run_id;
  result.incumbent_events.push_back(event);
  pe::RiskGroundTruthMetric risk_metric;
  risk_metric.run_id = result.metrics.run_id;
  result.risk_ground_truth_metrics.push_back(risk_metric);
  std::string error;
  ASSERT_TRUE(pe::ResultWriter().WriteRunDirectory(
      "/tmp/planner_experiments_serialization_test", result, &error)) << error;
  std::ifstream json_input("/tmp/planner_experiments_serialization_test/result.json");
  nlohmann::json json;
  json_input >> json;
  EXPECT_EQ(json["run_id"], "serialization_test");
  std::ifstream csv_input("/tmp/planner_experiments_serialization_test/trajectory.csv");
  std::string header;
  std::getline(csv_input, header);
  EXPECT_EQ(header, "t,x,y,yaw,vx,vy,ax,ay,yaw_rate,yaw_acceleration");
  std::ifstream incumbent_input(
      "/tmp/planner_experiments_serialization_test/incumbent_events.jsonl");
  nlohmann::json incumbent;
  incumbent_input >> incumbent;
  EXPECT_EQ(incumbent["run_id"], "serialization_test");
  std::ifstream risk_input(
      "/tmp/planner_experiments_serialization_test/risk_ground_truth.jsonl");
  nlohmann::json risk;
  risk_input >> risk;
  EXPECT_EQ(risk["run_id"], "serialization_test");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
