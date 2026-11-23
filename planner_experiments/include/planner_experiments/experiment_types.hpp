#pragma once

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "isweep_planner/core/common.h"

namespace planner_experiments {

inline double MissingDouble() {
  return std::numeric_limits<double>::quiet_NaN();
}

enum class PlannerStatus {
  NO_PATH,
  PLANNER_RETURNED,
  FOOTPRINT_VALIDATED,
  DYNAMICS_VALIDATED,
  EXECUTABLE,
  DEGRADED_DIAGNOSTIC,
  TIMEOUT,
  INTERNAL_ERROR,
  CONFIGURATION_ERROR,
};

const char* ToString(PlannerStatus status);

struct PlanningRequest {
  std::string run_id;
  std::string configuration_hash;
  std::string task_id;
  std::string method_name;
  std::string map_name;
  std::string map_hash;
  std::string robot_shape_name;
  geometry_msgs::Pose2D start;
  geometry_msgs::Pose2D goal;
  uint32_t random_seed = 1;
  double total_time_budget_sec = 20.0;
  bool risk_ground_truth_enabled = false;
  bool risk_ground_truth_run_low = true;
  bool risk_ground_truth_run_medium = true;
  bool risk_ground_truth_run_strict = true;
};

struct DynamicsLimits {
  double max_linear_velocity = MissingDouble();
  double max_linear_acceleration = MissingDouble();
  double max_angular_velocity = MissingDouble();
  double max_angular_acceleration = MissingDouble();
};

struct ValidationConfig {
  double time_step_sec = 0.01;
  double max_translation_step_m = 0.01;
  double max_yaw_step_rad = 0.01;
  double footprint_boundary_step_m = 0.01;
  double footprint_interior_step_m = 0.02;
  double collision_clearance_m = 0.0;
  bool require_dynamics = true;
  bool require_boundary_continuity = true;
  bool require_endpoint_match = true;
  double max_endpoint_position_error_m = 1e-3;
  double max_endpoint_yaw_error_rad = 1e-2;
  double max_position_jump_m = 1e-4;
  double max_velocity_jump_mps = 1e-3;
  double max_acceleration_jump_mps2 = 1e-2;
  double max_yaw_jump_rad = 1e-3;
  double max_yaw_rate_jump_radps = 1e-2;
  int max_subdivision_depth = 24;
};

struct RecoveryConfig {
  bool enabled = true;
  bool local_upgrade = true;
  bool strict_local_patch = true;
  bool swept_astar = true;
  bool hybrid_astar = true;
  bool candidate_switch = true;
  bool tail_refinement = true;
  bool hybrid_frontend_fallback = true;
  int post_incumbent_svsdf_budget = 4;
  int tail_refinement_svsdf_budget = 1;
};

struct MethodConfig {
  std::string name = "current_full";
  bool force_all_low = false;
  bool force_all_strict = false;
  bool force_full_svsdf = false;
  bool hybrid_astar_only = false;
  RecoveryConfig recovery;
};

struct TrajectorySample {
  double t = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double yaw_rate = 0.0;
  double yaw_acceleration = 0.0;
};

struct ValidationResult {
  bool external_footprint_valid = false;
  double minimum_external_clearance_m = MissingDouble();
  double collision_time_sec = MissingDouble();
  double collision_x = MissingDouble();
  double collision_y = MissingDouble();
  double collision_yaw = MissingDouble();
  std::string collision_type;

  bool endpoint_check_performed = false;
  bool start_matches = false;
  bool goal_reached = false;
  bool endpoints_match = false;
  double start_position_error_m = MissingDouble();
  double start_yaw_error_rad = MissingDouble();
  double goal_position_error_m = MissingDouble();
  double goal_yaw_error_rad = MissingDouble();

  bool dynamics_valid = false;
  double max_linear_velocity = 0.0;
  double max_linear_acceleration = 0.0;
  double max_angular_velocity = 0.0;
  double max_angular_acceleration = 0.0;
  int velocity_violation_count = 0;
  int acceleration_violation_count = 0;
  int angular_violation_count = 0;

  bool boundary_continuous = false;
  double max_position_jump_m = 0.0;
  double max_velocity_jump_mps = 0.0;
  double max_acceleration_jump_mps2 = 0.0;
  double max_yaw_jump_rad = 0.0;
  double max_yaw_rate_jump_radps = 0.0;
  int worst_boundary_index = -1;
  std::vector<TrajectorySample> samples;
};

struct GeometryConsistencyResult {
  double polygon_area = MissingDouble();
  double mesh_projected_area = MissingDouble();
  double centroid_error_m = MissingDouble();
  double bidirectional_outline_distance_m = MissingDouble();
  double hausdorff_distance_m = MissingDouble();
  double maximum_radial_error_m = MissingDouble();
  bool consistency_passed = false;
  std::string failure_reason;
};

struct StateMetric {
  std::string run_id;
  int candidate_index = -1;
  int state_index = -1;
  double arc_length_m = 0.0;
  double x = 0.0;
  double y = 0.0;
  double reference_yaw = MissingDouble();
  double selected_yaw = 0.0;
  double center_clearance_m = MissingDouble();
  double footprint_clearance_m = MissingDouble();
  int safe_yaw_count = -1;
  int largest_contiguous_safe_yaw_bins = -1;
  double yaw_bin_error = MissingDouble();
  bool adjust_attempted = false;
  bool adjust_success = false;
  bool adjust_metrics_available = false;
  int adjust_iteration_count = -1;
  double adjust_distance_m = MissingDouble();
  int recursive_depth = -1;
  double risk_score = MissingDouble();
  double effective_clearance_m = MissingDouble();
  double feasible_yaw_width = MissingDouble();
  double feasible_yaw_overlap = MissingDouble();
  double correction_distance_m = MissingDouble();
  double uncertainty_erosion_m = MissingDouble();
  double clearance_risk = MissingDouble();
  double orientation_width_risk = MissingDouble();
  double orientation_continuity_risk = MissingDouble();
  double correction_risk = MissingDouble();
  double uncertainty_risk = MissingDouble();
  std::string risk_label;
  int source_segment_index = -1;
};

struct SegmentMetric {
  std::string run_id;
  int candidate_index = -1;
  int segment_index = -1;
  std::string risk_label;
  int start_state_index = -1;
  int end_state_index = -1;
  double segment_length_m = MissingDouble();
  int state_count = 0;
  double minimum_center_clearance_m = MissingDouble();
  double minimum_footprint_clearance_m = MissingDouble();
  double mean_safe_yaw_count = MissingDouble();
  bool strict_called = false;
  bool full_svsdf_used = false;
  bool solver_success = false;
  bool full_svsdf_metric_available = false;
  double solver_time_ms = MissingDouble();
  bool recovery_escalated = false;
  bool recovery_metric_available = false;
};

struct IncumbentEventMetric {
  std::string run_id;
  int update_index = 0;
  double elapsed_ms = MissingDouble();
  int candidate_index = -1;
  double minimum_clearance_m = MissingDouble();
  double smoothness_cost = MissingDouble();
  double trajectory_length_m = MissingDouble();
  double trajectory_duration_sec = MissingDouble();
  int piece_count = 0;
  int low_segment_count = 0;
  int medium_segment_count = 0;
  int high_segment_count = 0;
  bool recovery_success = false;
  bool internally_validated = true;
  std::string validation_scope = "runtime_publication_contract";
};

struct RiskGroundTruthMetric {
  std::string run_id;
  int candidate_index = -1;
  int segment_index = -1;
  std::string assigned_risk_label;
  double maximum_risk_score = MissingDouble();
  double mean_risk_score = MissingDouble();
  int state_count = 0;
  double segment_length_m = MissingDouble();
  isweep_planner::SE2State expected_start;
  isweep_planner::SE2State expected_goal;

  bool low_solver_attempted = false;
  bool low_solver_success = false;
  double low_solve_ms = MissingDouble();
  bool low_endpoints_match = false;
  bool low_executable = false;
  ValidationResult low_validation;

  bool medium_solver_attempted = false;
  bool medium_solver_success = false;
  double medium_solve_ms = MissingDouble();
  bool medium_endpoints_match = false;
  bool medium_executable = false;
  ValidationResult medium_validation;

  bool strict_solver_attempted = false;
  bool strict_solver_success = false;
  bool full_svsdf_used = false;
  bool full_svsdf_success = false;
  double strict_solve_ms = MissingDouble();
  double full_svsdf_ms = MissingDouble();
  bool strict_endpoints_match = false;
  bool strict_executable = false;
  ValidationResult strict_validation;

  bool label_available = false;
  bool l2_required = false;
  bool l3_required = false;
  bool strict_required = false;
  std::string minimum_required_level;
  std::string outcome;

  // Transient trajectories are carried from the runtime to the independent
  // validator.  ToJson intentionally excludes them from the JSONL record.
  isweep_planner::Trajectory low_trajectory;
  isweep_planner::Trajectory medium_trajectory;
  isweep_planner::Trajectory strict_trajectory;
};

struct ExperimentMetrics {
  std::string run_id;
  std::string timestamp;
  std::string git_commit;
  bool git_dirty = false;
  std::string configuration_hash;
  std::string build_type;
  std::string compiler_version;
  std::string ros_distro;
  std::string cpu_model;
  uint64_t memory_total = 0;
  int thread_count = 1;
  std::string method_name;
  std::string map_name;
  std::string map_hash;
  double map_resolution = MissingDouble();
  std::string robot_shape;
  std::string polygon_id;
  std::string mesh_id;
  bool geometry_consistency_passed = false;
  std::string task_id;
  geometry_msgs::Pose2D start;
  geometry_msgs::Pose2D goal;
  uint32_t random_seed = 1;
  double total_time_budget_sec = 20.0;

  PlannerStatus planner_status = PlannerStatus::NO_PATH;
  std::string failure_stage;
  std::string failure_reason;
  bool planner_returned = false;
  bool internal_footprint_valid = false;
  bool external_footprint_valid = false;
  bool dynamics_valid = false;
  bool boundary_continuous = false;
  bool endpoint_check_performed = false;
  bool start_matches = false;
  bool goal_reached = false;
  bool endpoints_match = false;
  bool executable = false;
  bool degraded = false;
  bool timeout = false;
  int candidate_count = 0;
  int selected_candidate_index = -1;

  double t_map_load_ms = MissingDouble();
  double t_esdf_ms = MissingDouble();
  double t_geometry_precompute_ms = MissingDouble();
  double t_roadmap_ms = MissingDouble();
  double t_dijkstra_ms = MissingDouble();
  double t_shortcut_ms = MissingDouble();
  double t_path_adjust_ms = MissingDouble();
  double t_se2_sequence_ms = MissingDouble();
  double t_risk_classification_ms = MissingDouble();
  double t_low_solve_ms = MissingDouble();
  double t_medium_solve_ms = MissingDouble();
  double t_strict_solve_ms = MissingDouble();
  double t_full_svsdf_ms = MissingDouble();
  double t_stitch_ms = MissingDouble();
  double t_internal_validation_ms = MissingDouble();
  double t_external_validation_ms = MissingDouble();
  double t_recovery_ms = MissingDouble();
  double t_total_ms = MissingDouble();
  double t_first_feasible_ms = MissingDouble();
  double t_risk_ground_truth_ms = MissingDouble();

  int state_count = 0;
  int low_state_count = 0;
  int medium_state_count = 0;
  int high_state_count = 0;
  double high_state_ratio = MissingDouble();
  int segment_count = 0;
  int low_segment_count = 0;
  int medium_segment_count = 0;
  int high_segment_count = 0;
  int segment_boundary_count = 0;
  double minimum_center_clearance_m = MissingDouble();
  double minimum_internal_footprint_clearance_m = MissingDouble();
  int minimum_safe_yaw_count = -1;
  double mean_safe_yaw_count = MissingDouble();
  double maximum_yaw_bin_error = MissingDouble();
  int adjust_attempt_count = 0;
  int adjust_success_count = 0;

  int low_solver_call_count = 0;
  int low_solver_success_count = 0;
  int medium_solver_call_count = 0;
  int medium_solver_success_count = 0;
  int strict_solver_call_count = 0;
  int strict_solver_success_count = 0;
  int full_svsdf_call_count = 0;
  int full_svsdf_success_count = 0;
  bool full_svsdf_used = false;
  int full_svsdf_initial_assigned_call_count = 0;
  int full_svsdf_initial_escalation_call_count = 0;
  int full_svsdf_frontend_fallback_call_count = 0;
  int full_svsdf_validation_recovery_call_count = 0;
  int full_svsdf_tail_refinement_call_count = 0;
  int full_svsdf_other_call_count = 0;
  int preserve_shape_bypass_count = 0;
  int lbfgs_iteration_count = -1;
  int lmbm_iteration_count = -1;

  std::string failure_type;
  double bottleneck_time_sec = MissingDouble();
  double bottleneck_clearance_m = MissingDouble();
  int bottleneck_segment_index = -1;
  int initial_level_upgrade_attempt_count = 0;
  int initial_level_upgrade_success_count = 0;
  int frontend_fallback_attempt_count = 0;
  int frontend_fallback_success_count = 0;
  int validation_recovery_attempt_count = 0;
  int validation_recovery_success_count = 0;
  int local_upgrade_attempt_count = 0;
  int local_upgrade_success_count = 0;
  int strict_patch_attempt_count = 0;
  int strict_patch_success_count = 0;
  int swept_astar_attempt_count = 0;
  int swept_astar_success_count = 0;
  int swept_astar_expanded_nodes = 0;
  int hybrid_astar_attempt_count = 0;
  int hybrid_astar_success_count = 0;
  int hybrid_astar_expanded_nodes = -1;
  int candidate_switch_count = 0;
  int post_incumbent_svsdf_call_count = 0;
  int post_incumbent_svsdf_budget_reject_count = 0;
  int tail_refinement_svsdf_budget_reject_count = 0;
  int external_equivalent_gate_attempt_count = 0;
  int external_equivalent_gate_reject_count = 0;
  int incumbent_update_count = 0;
  int incumbent_reject_count = 0;
  int validation_failure_count = 0;
  bool risk_ground_truth_enabled = false;
  int risk_ground_truth_trial_count = 0;
  int risk_ground_truth_labeled_count = 0;
  int risk_ground_truth_l2_required_count = 0;
  int risk_ground_truth_l3_required_count = 0;
  int risk_ground_truth_strict_required_count = 0;
  bool recovery_success = false;
  double recovery_extra_time_ms = MissingDouble();
  double patch_length_m = MissingDouble();
  double replanned_length_m = MissingDouble();
  double original_trajectory_length_m = MissingDouble();
  double trajectory_reuse_ratio = MissingDouble();

  double trajectory_length_m = MissingDouble();
  double trajectory_duration_sec = MissingDouble();
  double minimum_internal_clearance_m = MissingDouble();
  double minimum_external_clearance_m = MissingDouble();
  double smoothness_cost = MissingDouble();
  double control_effort = MissingDouble();
  ValidationResult validation;
  GeometryConsistencyResult geometry;
};

struct PlanningResult {
  PlannerStatus status = PlannerStatus::NO_PATH;
  bool planner_returned = false;
  bool internal_footprint_valid = false;
  bool timeout = false;
  bool degraded = false;
  std::string failure_stage;
  std::string failure_reason;
  isweep_planner::Trajectory trajectory;
  ExperimentMetrics metrics;
  std::vector<StateMetric> state_metrics;
  std::vector<SegmentMetric> segment_metrics;
  std::vector<IncumbentEventMetric> incumbent_events;
  std::vector<RiskGroundTruthMetric> risk_ground_truth_metrics;
};

nlohmann::json ToJson(const GeometryConsistencyResult& value);
nlohmann::json ToJson(const ValidationResult& value);
nlohmann::json ToJson(const StateMetric& value);
nlohmann::json ToJson(const SegmentMetric& value);
nlohmann::json ToJson(const IncumbentEventMetric& value);
nlohmann::json ToJson(const RiskGroundTruthMetric& value);
nlohmann::json ToJson(const ExperimentMetrics& value);
void FinalizeRiskGroundTruthLabel(RiskGroundTruthMetric* value);

int CircularBinDistance(int lhs, int rhs, int bins);
int LargestContiguousSafeYawBins(const std::vector<int>& safe_bins, int bins);
std::vector<double> UnwrapYaw(const std::vector<double>& yaw);

}  // namespace planner_experiments
