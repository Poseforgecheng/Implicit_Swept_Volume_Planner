#include "planner_experiments/experiment_types.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace planner_experiments {
namespace {

void PutDouble(nlohmann::json* json, const char* key, double value) {
  if (std::isfinite(value)) {
    (*json)[key] = value;
  } else {
    (*json)[key] = nullptr;
  }
}

}  // namespace

const char* ToString(PlannerStatus status) {
  switch (status) {
    case PlannerStatus::NO_PATH: return "NO_PATH";
    case PlannerStatus::PLANNER_RETURNED: return "PLANNER_RETURNED";
    case PlannerStatus::FOOTPRINT_VALIDATED: return "FOOTPRINT_VALIDATED";
    case PlannerStatus::DYNAMICS_VALIDATED: return "DYNAMICS_VALIDATED";
    case PlannerStatus::EXECUTABLE: return "EXECUTABLE";
    case PlannerStatus::DEGRADED_DIAGNOSTIC: return "DEGRADED_DIAGNOSTIC";
    case PlannerStatus::TIMEOUT: return "TIMEOUT";
    case PlannerStatus::INTERNAL_ERROR: return "INTERNAL_ERROR";
    case PlannerStatus::CONFIGURATION_ERROR: return "CONFIGURATION_ERROR";
  }
  return "INTERNAL_ERROR";
}

int CircularBinDistance(int lhs, int rhs, int bins) {
  if (bins <= 0) return -1;
  lhs = ((lhs % bins) + bins) % bins;
  rhs = ((rhs % bins) + bins) % bins;
  const int raw = std::abs(lhs - rhs);
  return std::min(raw, bins - raw);
}

int LargestContiguousSafeYawBins(const std::vector<int>& safe_bins, int bins) {
  if (bins <= 0 || safe_bins.empty()) return 0;
  std::vector<bool> safe(static_cast<size_t>(bins), false);
  for (int bin : safe_bins) {
    safe[static_cast<size_t>(((bin % bins) + bins) % bins)] = true;
  }
  if (std::all_of(safe.begin(), safe.end(), [](bool value) { return value; })) {
    return bins;
  }
  int best = 0;
  int current = 0;
  for (int i = 0; i < 2 * bins; ++i) {
    if (safe[static_cast<size_t>(i % bins)]) {
      current = std::min(bins, current + 1);
      best = std::max(best, current);
    } else {
      current = 0;
    }
  }
  return best;
}

std::vector<double> UnwrapYaw(const std::vector<double>& yaw) {
  if (yaw.empty()) return {};
  std::vector<double> result;
  result.reserve(yaw.size());
  result.push_back(yaw.front());
  for (size_t i = 1; i < yaw.size(); ++i) {
    result.push_back(result.back() +
                     isweep_planner::normalizeAngle(yaw[i] - yaw[i - 1]));
  }
  return result;
}

nlohmann::json ToJson(const GeometryConsistencyResult& value) {
  nlohmann::json json;
  PutDouble(&json, "polygon_area", value.polygon_area);
  PutDouble(&json, "mesh_projected_area", value.mesh_projected_area);
  PutDouble(&json, "centroid_error_m", value.centroid_error_m);
  PutDouble(&json, "bidirectional_outline_distance_m",
            value.bidirectional_outline_distance_m);
  PutDouble(&json, "hausdorff_distance_m", value.hausdorff_distance_m);
  PutDouble(&json, "maximum_radial_error_m", value.maximum_radial_error_m);
  json["consistency_passed"] = value.consistency_passed;
  json["failure_reason"] = value.failure_reason;
  return json;
}

nlohmann::json ToJson(const ValidationResult& value) {
  nlohmann::json json;
  json["external_footprint_valid"] = value.external_footprint_valid;
  PutDouble(&json, "minimum_external_clearance_m",
            value.minimum_external_clearance_m);
  PutDouble(&json, "collision_time_sec", value.collision_time_sec);
  PutDouble(&json, "collision_x", value.collision_x);
  PutDouble(&json, "collision_y", value.collision_y);
  PutDouble(&json, "collision_yaw", value.collision_yaw);
  json["collision_type"] = value.collision_type;
  json["endpoint_check_performed"] = value.endpoint_check_performed;
  json["start_matches"] = value.start_matches;
  json["goal_reached"] = value.goal_reached;
  json["endpoints_match"] = value.endpoints_match;
  PutDouble(&json, "start_position_error_m", value.start_position_error_m);
  PutDouble(&json, "start_yaw_error_rad", value.start_yaw_error_rad);
  PutDouble(&json, "goal_position_error_m", value.goal_position_error_m);
  PutDouble(&json, "goal_yaw_error_rad", value.goal_yaw_error_rad);
  json["dynamics_valid"] = value.dynamics_valid;
  json["max_linear_velocity"] = value.max_linear_velocity;
  json["max_linear_acceleration"] = value.max_linear_acceleration;
  json["max_angular_velocity"] = value.max_angular_velocity;
  json["max_angular_acceleration"] = value.max_angular_acceleration;
  json["velocity_violation_count"] = value.velocity_violation_count;
  json["acceleration_violation_count"] = value.acceleration_violation_count;
  json["angular_violation_count"] = value.angular_violation_count;
  json["boundary_continuous"] = value.boundary_continuous;
  json["max_position_jump_m"] = value.max_position_jump_m;
  json["max_velocity_jump_mps"] = value.max_velocity_jump_mps;
  json["max_acceleration_jump_mps2"] = value.max_acceleration_jump_mps2;
  json["max_yaw_jump_rad"] = value.max_yaw_jump_rad;
  json["max_yaw_rate_jump_radps"] = value.max_yaw_rate_jump_radps;
  json["worst_boundary_index"] = value.worst_boundary_index;
  return json;
}

nlohmann::json ToJson(const StateMetric& value) {
  nlohmann::json json = {
      {"run_id", value.run_id}, {"candidate_index", value.candidate_index},
      {"state_index", value.state_index}, {"arc_length_m", value.arc_length_m},
      {"x", value.x}, {"y", value.y}, {"selected_yaw", value.selected_yaw},
      {"safe_yaw_count", value.safe_yaw_count},
      {"largest_contiguous_safe_yaw_bins", value.largest_contiguous_safe_yaw_bins},
      {"risk_label", value.risk_label},
      {"source_segment_index", value.source_segment_index}};
  if (value.adjust_metrics_available) {
    json["adjust_attempted"] = value.adjust_attempted;
    json["adjust_success"] = value.adjust_success;
  } else {
    json["adjust_attempted"] = nullptr;
    json["adjust_success"] = nullptr;
  }
  json["adjust_iteration_count"] = value.adjust_iteration_count >= 0
                                       ? nlohmann::json(value.adjust_iteration_count)
                                       : nlohmann::json(nullptr);
  json["recursive_depth"] = value.recursive_depth >= 0
                                ? nlohmann::json(value.recursive_depth)
                                : nlohmann::json(nullptr);
  PutDouble(&json, "reference_yaw", value.reference_yaw);
  PutDouble(&json, "center_clearance_m", value.center_clearance_m);
  PutDouble(&json, "footprint_clearance_m", value.footprint_clearance_m);
  PutDouble(&json, "yaw_bin_error", value.yaw_bin_error);
  PutDouble(&json, "adjust_distance_m", value.adjust_distance_m);
  PutDouble(&json, "risk_score", value.risk_score);
  PutDouble(&json, "effective_clearance_m", value.effective_clearance_m);
  PutDouble(&json, "feasible_yaw_width", value.feasible_yaw_width);
  PutDouble(&json, "feasible_yaw_overlap", value.feasible_yaw_overlap);
  PutDouble(&json, "correction_distance_m", value.correction_distance_m);
  PutDouble(&json, "uncertainty_erosion_m", value.uncertainty_erosion_m);
  PutDouble(&json, "clearance_risk", value.clearance_risk);
  PutDouble(&json, "orientation_width_risk", value.orientation_width_risk);
  PutDouble(&json, "orientation_continuity_risk",
            value.orientation_continuity_risk);
  PutDouble(&json, "correction_risk", value.correction_risk);
  PutDouble(&json, "uncertainty_risk", value.uncertainty_risk);
  return json;
}

nlohmann::json ToJson(const SegmentMetric& value) {
  nlohmann::json json = {
      {"run_id", value.run_id}, {"candidate_index", value.candidate_index},
      {"segment_index", value.segment_index}, {"risk_label", value.risk_label},
      {"start_state_index", value.start_state_index},
      {"end_state_index", value.end_state_index}, {"state_count", value.state_count},
      {"strict_called", value.strict_called},
      {"solver_success", value.solver_success}};
  json["full_svsdf_used"] = value.full_svsdf_metric_available
                                 ? nlohmann::json(value.full_svsdf_used)
                                 : nlohmann::json(nullptr);
  json["recovery_escalated"] = value.recovery_metric_available
                                    ? nlohmann::json(value.recovery_escalated)
                                    : nlohmann::json(nullptr);
  PutDouble(&json, "segment_length_m", value.segment_length_m);
  PutDouble(&json, "minimum_center_clearance_m", value.minimum_center_clearance_m);
  PutDouble(&json, "minimum_footprint_clearance_m", value.minimum_footprint_clearance_m);
  PutDouble(&json, "mean_safe_yaw_count", value.mean_safe_yaw_count);
  PutDouble(&json, "solver_time_ms", value.solver_time_ms);
  return json;
}

nlohmann::json ToJson(const IncumbentEventMetric& value) {
  nlohmann::json json = {
      {"schema_version", 1},
      {"run_id", value.run_id},
      {"update_index", value.update_index},
      {"candidate_index", value.candidate_index},
      {"piece_count", value.piece_count},
      {"low_segment_count", value.low_segment_count},
      {"medium_segment_count", value.medium_segment_count},
      {"high_segment_count", value.high_segment_count},
      {"recovery_success", value.recovery_success},
      {"internally_validated", value.internally_validated},
      {"validation_scope", value.validation_scope}};
  PutDouble(&json, "elapsed_ms", value.elapsed_ms);
  PutDouble(&json, "minimum_clearance_m", value.minimum_clearance_m);
  PutDouble(&json, "smoothness_cost", value.smoothness_cost);
  PutDouble(&json, "trajectory_length_m", value.trajectory_length_m);
  PutDouble(&json, "trajectory_duration_sec", value.trajectory_duration_sec);
  return json;
}

nlohmann::json ToJson(const RiskGroundTruthMetric& value) {
  nlohmann::json json = {
      {"schema_version", 2},
      {"run_id", value.run_id},
      {"candidate_index", value.candidate_index},
      {"segment_index", value.segment_index},
      {"assigned_risk_label", value.assigned_risk_label},
      {"state_count", value.state_count},
      {"expected_start_x", value.expected_start.x},
      {"expected_start_y", value.expected_start.y},
      {"expected_start_yaw", value.expected_start.yaw},
      {"expected_goal_x", value.expected_goal.x},
      {"expected_goal_y", value.expected_goal.y},
      {"expected_goal_yaw", value.expected_goal.yaw},
      {"low_solver_attempted", value.low_solver_attempted},
      {"low_solver_success", value.low_solver_success},
      {"low_endpoints_match", value.low_endpoints_match},
      {"low_executable", value.low_executable},
      {"medium_solver_attempted", value.medium_solver_attempted},
      {"medium_solver_success", value.medium_solver_success},
      {"medium_endpoints_match", value.medium_endpoints_match},
      {"medium_executable", value.medium_executable},
      {"strict_solver_attempted", value.strict_solver_attempted},
      {"strict_solver_success", value.strict_solver_success},
      {"full_svsdf_used", value.full_svsdf_used},
      {"full_svsdf_success", value.full_svsdf_success},
      {"strict_endpoints_match", value.strict_endpoints_match},
      {"strict_executable", value.strict_executable},
      {"label_available", value.label_available},
      {"l2_required", value.l2_required},
      {"l3_required", value.l3_required},
      {"strict_required", value.strict_required},
      {"minimum_required_level", value.minimum_required_level},
      {"outcome", value.outcome},
      {"low_validation", ToJson(value.low_validation)},
      {"medium_validation", ToJson(value.medium_validation)},
      {"strict_validation", ToJson(value.strict_validation)}};
  PutDouble(&json, "maximum_risk_score", value.maximum_risk_score);
  PutDouble(&json, "mean_risk_score", value.mean_risk_score);
  PutDouble(&json, "segment_length_m", value.segment_length_m);
  PutDouble(&json, "low_solve_ms", value.low_solve_ms);
  PutDouble(&json, "medium_solve_ms", value.medium_solve_ms);
  PutDouble(&json, "strict_solve_ms", value.strict_solve_ms);
  PutDouble(&json, "full_svsdf_ms", value.full_svsdf_ms);
  return json;
}

void FinalizeRiskGroundTruthLabel(RiskGroundTruthMetric* value) {
  if (value == nullptr) return;
  value->label_available =
      value->low_solver_attempted && value->medium_solver_attempted &&
      value->strict_solver_attempted && value->full_svsdf_used &&
      value->full_svsdf_success;
  value->l2_required = false;
  value->l3_required = false;
  value->strict_required = false;
  if (!value->label_available) {
    value->minimum_required_level = "INCOMPLETE";
    value->outcome = "incomplete_three_level_solve";
    return;
  }

  value->l2_required =
      !value->low_executable && value->medium_executable;
  value->l3_required =
      !value->medium_executable && value->strict_executable;
  if (value->low_executable) {
    value->minimum_required_level = "LOW";
    value->outcome = "low_sufficient";
  } else if (value->medium_executable) {
    value->minimum_required_level = "MEDIUM";
    value->outcome = "medium_required";
  } else if (value->strict_executable) {
    value->strict_required = true;
    value->minimum_required_level = "HIGH";
    value->outcome = "strict_required";
  } else {
    value->minimum_required_level = "NONE";
    value->outcome = "all_failed";
  }
}

nlohmann::json ToJson(const ExperimentMetrics& v) {
  nlohmann::json j = {
      {"schema_version", 4}, {"run_id", v.run_id}, {"timestamp", v.timestamp},
      {"git_commit", v.git_commit}, {"git_dirty", v.git_dirty},
      {"configuration_hash", v.configuration_hash},
      {"build_type", v.build_type}, {"compiler_version", v.compiler_version},
      {"ros_distro", v.ros_distro}, {"cpu_model", v.cpu_model},
      {"memory_total", v.memory_total}, {"thread_count", v.thread_count},
      {"method_name", v.method_name}, {"map_name", v.map_name},
      {"map_hash", v.map_hash}, {"robot_shape", v.robot_shape},
      {"polygon_id", v.polygon_id}, {"mesh_id", v.mesh_id},
      {"geometry_consistency_passed", v.geometry_consistency_passed},
      {"task_id", v.task_id}, {"start_x", v.start.x}, {"start_y", v.start.y},
      {"start_yaw", v.start.theta}, {"goal_x", v.goal.x}, {"goal_y", v.goal.y},
      {"goal_yaw", v.goal.theta}, {"random_seed", v.random_seed},
      {"total_time_budget_sec", v.total_time_budget_sec},
      {"planner_status", ToString(v.planner_status)}, {"failure_stage", v.failure_stage},
      {"failure_reason", v.failure_reason}, {"planner_returned", v.planner_returned},
      {"internal_footprint_valid", v.internal_footprint_valid},
      {"external_footprint_valid", v.external_footprint_valid},
      {"dynamics_valid", v.dynamics_valid},
      {"boundary_continuous", v.boundary_continuous},
      {"endpoint_check_performed", v.endpoint_check_performed},
      {"start_matches", v.start_matches}, {"goal_reached", v.goal_reached},
      {"endpoints_match", v.endpoints_match}, {"executable", v.executable},
      {"degraded", v.degraded}, {"timeout", v.timeout},
      {"candidate_count", v.candidate_count},
      {"selected_candidate_index", v.selected_candidate_index},
      {"state_count", v.state_count}, {"low_state_count", v.low_state_count},
      {"medium_state_count", v.medium_state_count},
      {"high_state_count", v.high_state_count}, {"segment_count", v.segment_count},
      {"low_segment_count", v.low_segment_count},
      {"medium_segment_count", v.medium_segment_count},
      {"high_segment_count", v.high_segment_count},
      {"segment_boundary_count", v.segment_boundary_count},
      {"minimum_safe_yaw_count", v.minimum_safe_yaw_count},
      {"adjust_attempt_count", v.adjust_attempt_count},
      {"adjust_success_count", v.adjust_success_count},
      {"low_solver_call_count", v.low_solver_call_count},
      {"low_solver_success_count", v.low_solver_success_count},
      {"medium_solver_call_count", v.medium_solver_call_count},
      {"medium_solver_success_count", v.medium_solver_success_count},
      {"strict_solver_call_count", v.strict_solver_call_count},
      {"strict_solver_success_count", v.strict_solver_success_count},
      {"full_svsdf_call_count", v.full_svsdf_call_count},
      {"full_svsdf_success_count", v.full_svsdf_success_count},
      {"full_svsdf_used", v.full_svsdf_used},
      {"full_svsdf_initial_assigned_call_count",
       v.full_svsdf_initial_assigned_call_count},
      {"full_svsdf_initial_escalation_call_count",
       v.full_svsdf_initial_escalation_call_count},
      {"full_svsdf_frontend_fallback_call_count",
       v.full_svsdf_frontend_fallback_call_count},
      {"full_svsdf_validation_recovery_call_count",
       v.full_svsdf_validation_recovery_call_count},
      {"full_svsdf_tail_refinement_call_count",
       v.full_svsdf_tail_refinement_call_count},
      {"full_svsdf_other_call_count", v.full_svsdf_other_call_count},
      {"preserve_shape_bypass_count", v.preserve_shape_bypass_count},
      {"failure_type", v.failure_type},
      {"bottleneck_segment_index", v.bottleneck_segment_index},
      {"initial_level_upgrade_attempt_count",
       v.initial_level_upgrade_attempt_count},
      {"initial_level_upgrade_success_count",
       v.initial_level_upgrade_success_count},
      {"frontend_fallback_attempt_count", v.frontend_fallback_attempt_count},
      {"frontend_fallback_success_count", v.frontend_fallback_success_count},
      {"validation_recovery_attempt_count",
       v.validation_recovery_attempt_count},
      {"validation_recovery_success_count",
       v.validation_recovery_success_count},
      {"local_upgrade_attempt_count", v.local_upgrade_attempt_count},
      {"local_upgrade_success_count", v.local_upgrade_success_count},
      {"strict_patch_attempt_count", v.strict_patch_attempt_count},
      {"strict_patch_success_count", v.strict_patch_success_count},
      {"swept_astar_attempt_count", v.swept_astar_attempt_count},
      {"swept_astar_success_count", v.swept_astar_success_count},
      {"swept_astar_expanded_nodes", v.swept_astar_expanded_nodes},
      {"hybrid_astar_attempt_count", v.hybrid_astar_attempt_count},
      {"hybrid_astar_success_count", v.hybrid_astar_success_count},
      {"hybrid_astar_expanded_nodes", v.hybrid_astar_expanded_nodes},
      {"candidate_switch_count", v.candidate_switch_count},
      {"post_incumbent_svsdf_call_count",
       v.post_incumbent_svsdf_call_count},
      {"post_incumbent_svsdf_budget_reject_count",
       v.post_incumbent_svsdf_budget_reject_count},
      {"tail_refinement_svsdf_budget_reject_count",
       v.tail_refinement_svsdf_budget_reject_count},
      {"external_equivalent_gate_attempt_count",
       v.external_equivalent_gate_attempt_count},
      {"external_equivalent_gate_reject_count",
       v.external_equivalent_gate_reject_count},
      {"incumbent_update_count", v.incumbent_update_count},
      {"incumbent_reject_count", v.incumbent_reject_count},
      {"validation_failure_count", v.validation_failure_count},
      {"risk_ground_truth_enabled", v.risk_ground_truth_enabled},
      {"risk_ground_truth_trial_count", v.risk_ground_truth_trial_count},
      {"risk_ground_truth_labeled_count", v.risk_ground_truth_labeled_count},
      {"risk_ground_truth_l2_required_count",
       v.risk_ground_truth_l2_required_count},
      {"risk_ground_truth_l3_required_count",
       v.risk_ground_truth_l3_required_count},
      {"risk_ground_truth_strict_required_count",
       v.risk_ground_truth_strict_required_count},
      {"recovery_success", v.recovery_success},
      {"max_linear_velocity", v.validation.max_linear_velocity},
      {"max_linear_acceleration", v.validation.max_linear_acceleration},
      {"max_angular_velocity", v.validation.max_angular_velocity},
      {"max_angular_acceleration", v.validation.max_angular_acceleration},
      {"velocity_violation_count", v.validation.velocity_violation_count},
      {"acceleration_violation_count", v.validation.acceleration_violation_count},
      {"angular_violation_count", v.validation.angular_violation_count},
      {"max_position_jump_m", v.validation.max_position_jump_m},
      {"max_velocity_jump_mps", v.validation.max_velocity_jump_mps},
      {"max_acceleration_jump_mps2", v.validation.max_acceleration_jump_mps2},
      {"max_yaw_jump_rad", v.validation.max_yaw_jump_rad},
      {"max_yaw_rate_jump_radps", v.validation.max_yaw_rate_jump_radps},
      {"worst_boundary_index", v.validation.worst_boundary_index},
      {"validation", ToJson(v.validation)},
      {"geometry", ToJson(v.geometry)}};

  const std::pair<const char*, double> doubles[] = {
      {"map_resolution", v.map_resolution}, {"t_map_load_ms", v.t_map_load_ms},
      {"t_esdf_ms", v.t_esdf_ms}, {"t_geometry_precompute_ms", v.t_geometry_precompute_ms},
      {"t_roadmap_ms", v.t_roadmap_ms}, {"t_dijkstra_ms", v.t_dijkstra_ms},
      {"t_shortcut_ms", v.t_shortcut_ms}, {"t_path_adjust_ms", v.t_path_adjust_ms},
      {"t_se2_sequence_ms", v.t_se2_sequence_ms},
      {"t_risk_classification_ms", v.t_risk_classification_ms},
      {"t_low_solve_ms", v.t_low_solve_ms},
      {"t_medium_solve_ms", v.t_medium_solve_ms},
      {"t_strict_solve_ms", v.t_strict_solve_ms},
      {"t_full_svsdf_ms", v.t_full_svsdf_ms}, {"t_stitch_ms", v.t_stitch_ms},
      {"t_internal_validation_ms", v.t_internal_validation_ms},
      {"t_external_validation_ms", v.t_external_validation_ms},
      {"t_recovery_ms", v.t_recovery_ms}, {"t_total_ms", v.t_total_ms},
      {"t_first_feasible_ms", v.t_first_feasible_ms},
      {"t_risk_ground_truth_ms", v.t_risk_ground_truth_ms},
      {"high_state_ratio", v.high_state_ratio},
      {"minimum_center_clearance_m", v.minimum_center_clearance_m},
      {"minimum_internal_footprint_clearance_m", v.minimum_internal_footprint_clearance_m},
      {"mean_safe_yaw_count", v.mean_safe_yaw_count},
      {"maximum_yaw_bin_error", v.maximum_yaw_bin_error},
      {"bottleneck_time_sec", v.bottleneck_time_sec},
      {"bottleneck_clearance_m", v.bottleneck_clearance_m},
      {"recovery_extra_time_ms", v.recovery_extra_time_ms},
      {"patch_length_m", v.patch_length_m}, {"replanned_length_m", v.replanned_length_m},
      {"original_trajectory_length_m", v.original_trajectory_length_m},
      {"trajectory_reuse_ratio", v.trajectory_reuse_ratio},
      {"trajectory_length_m", v.trajectory_length_m},
      {"trajectory_duration_sec", v.trajectory_duration_sec},
      {"minimum_internal_clearance_m", v.minimum_internal_clearance_m},
      {"minimum_external_clearance_m", v.minimum_external_clearance_m},
      {"start_position_error_m", v.validation.start_position_error_m},
      {"start_yaw_error_rad", v.validation.start_yaw_error_rad},
      {"goal_position_error_m", v.validation.goal_position_error_m},
      {"goal_yaw_error_rad", v.validation.goal_yaw_error_rad},
      {"smoothness_cost", v.smoothness_cost}, {"control_effort", v.control_effort}};
  for (const auto& item : doubles) PutDouble(&j, item.first, item.second);
  if (v.lbfgs_iteration_count >= 0) j["lbfgs_iteration_count"] = v.lbfgs_iteration_count;
  else j["lbfgs_iteration_count"] = nullptr;
  if (v.lmbm_iteration_count >= 0) j["lmbm_iteration_count"] = v.lmbm_iteration_count;
  else j["lmbm_iteration_count"] = nullptr;
  return j;
}

}  // namespace planner_experiments
