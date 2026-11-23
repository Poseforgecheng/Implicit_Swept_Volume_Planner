#include "planner_experiments/planner_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>

#include "isweep_planner/core/trajectory_utils.h"
#include "planner_experiments/trajectory_validator.hpp"

namespace planner_experiments {
namespace {

double FootprintClearance(const isweep_planner::GridMap& map,
                          const isweep_planner::FootprintModel& footprint,
                          double x, double y, double yaw) {
  const auto& samples = footprint.denseBodySamples(
      std::max(0.005, 0.5 * map.resolution()),
      std::max(0.01, map.resolution()));
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  double minimum = isweep_planner::kInf;
  for (const Eigen::Vector2d& point : samples) {
    const double wx = x + c * point.x() - s * point.y();
    const double wy = y + s * point.x() + c * point.y();
    const isweep_planner::GridIndex cell = map.worldToGrid(wx, wy);
    if (!map.isInside(cell.x, cell.y)) return -isweep_planner::kInf;
    minimum = std::min(minimum, map.getEsdf(wx, wy));
  }
  return minimum;
}

}  // namespace

bool PlannerAdapter::Initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                                const nav_msgs::OccupancyGrid& map,
                                const MethodConfig& method,
                                const ValidationConfig& validation,
                                std::string* error) {
  method_ = method;
  validation_config_ = validation;
  validation_map_ = map;
  runtime_.Initialize(nh, pnh);
  isweep_planner::TrajectoryFootprintValidationConfig footprint_validation;
  footprint_validation.time_step_sec = validation.time_step_sec;
  footprint_validation.max_translation_step_m =
      validation.max_translation_step_m;
  footprint_validation.max_yaw_step_rad = validation.max_yaw_step_rad;
  footprint_validation.footprint_boundary_step_m =
      validation.footprint_boundary_step_m;
  footprint_validation.footprint_interior_step_m =
      validation.footprint_interior_step_m;
  footprint_validation.collision_clearance_m =
      validation.collision_clearance_m;
  footprint_validation.max_subdivision_depth =
      validation.max_subdivision_depth;
  runtime_.SetFootprintValidationConfig(footprint_validation);
  if (!runtime_.UpdateMap(map)) {
    if (error) *error = "SvsdfRuntime::UpdateMap failed";
    initialized_ = false;
    return false;
  }
  planner_limits_.max_linear_velocity = runtime_.optimizer_params().max_vel;
  planner_limits_.max_linear_acceleration = runtime_.optimizer_params().max_acc;
  planner_limits_.max_angular_velocity = runtime_.optimizer_params().max_yaw_rate;
  double angular_acceleration = MissingDouble();
  pnh.getParam("thetamax", angular_acceleration);
  planner_limits_.max_angular_acceleration = angular_acceleration;
  external_validation_limits_ = planner_limits_;
  runtime_.SetExternalValidationGate(
      [this](const isweep_planner::Trajectory& trajectory,
             const isweep_planner::SE2State& expected_start,
             const isweep_planner::SE2State& expected_goal,
             double* minimum_clearance, double* maximum_velocity,
             double* maximum_acceleration, std::string* failure_reason,
             double* failure_time) {
        const ValidationResult validation = TrajectoryValidator().Validate(
            validation_map_, runtime_.grid_map(), runtime_.footprint(),
            trajectory, external_validation_limits_, validation_config_,
            expected_start, expected_goal);
        if (minimum_clearance != nullptr) {
          *minimum_clearance = validation.minimum_external_clearance_m;
        }
        if (maximum_velocity != nullptr) {
          *maximum_velocity = validation.max_linear_velocity;
        }
        if (maximum_acceleration != nullptr) {
          *maximum_acceleration = validation.max_linear_acceleration;
        }
        if (failure_time != nullptr) {
          *failure_time = validation.collision_time_sec;
        }
        const bool valid = validation.external_footprint_valid &&
                           validation.dynamics_valid &&
                           validation.boundary_continuous &&
                           validation.endpoint_check_performed &&
                           validation.endpoints_match;
        if (!valid && failure_reason != nullptr) {
          if (!validation.external_footprint_valid) {
            *failure_reason = "external-equivalent gate: " +
                (validation.collision_type.empty()
                     ? std::string("footprint collision")
                     : validation.collision_type);
          } else if (!validation.dynamics_valid) {
            *failure_reason =
                "external-equivalent gate: dynamics limit violation";
          } else if (!validation.boundary_continuous) {
            *failure_reason =
                "external-equivalent gate: piece boundary discontinuity";
          } else {
            *failure_reason =
                "external-equivalent gate: requested endpoint mismatch";
            if (failure_time != nullptr) {
              *failure_time = trajectory.totalDuration();
            }
          }
        } else if (failure_reason != nullptr) {
          failure_reason->clear();
        }
        return valid;
      });
  initialized_ = true;
  return true;
}

PlanningResult PlannerAdapter::Plan(const PlanningRequest& request) {
  PlanningResult output;
  output.metrics.run_id = request.run_id;
  output.metrics.configuration_hash = request.configuration_hash;
  output.metrics.task_id = request.task_id;
  output.metrics.method_name = request.method_name;
  output.metrics.map_name = request.map_name;
  output.metrics.map_hash = request.map_hash;
  output.metrics.robot_shape = request.robot_shape_name;
  output.metrics.start = request.start;
  output.metrics.goal = request.goal;
  output.metrics.random_seed = request.random_seed;
  output.metrics.total_time_budget_sec = request.total_time_budget_sec;

  if (!initialized_) {
    output.status = PlannerStatus::INTERNAL_ERROR;
    output.failure_stage = "adapter_initialize";
    output.failure_reason = "PlannerAdapter was not initialized";
    output.metrics.planner_status = output.status;
    output.metrics.failure_stage = output.failure_stage;
    output.metrics.failure_reason = output.failure_reason;
    return output;
  }

  isweep_planner::RuntimeExperimentOptions options;
  options.method_name = request.method_name;
  options.random_seed = request.random_seed;
  options.total_time_budget_sec = request.total_time_budget_sec;
  options.force_all_low = method_.force_all_low;
  options.force_all_strict = method_.force_all_strict;
  options.force_full_svsdf = method_.force_full_svsdf;
  options.hybrid_astar_only = method_.hybrid_astar_only;
  options.hybrid_frontend_fallback = method_.recovery.hybrid_frontend_fallback;
  options.recovery_enabled = method_.recovery.enabled;
  options.local_upgrade = method_.recovery.local_upgrade;
  options.strict_local_patch = method_.recovery.strict_local_patch;
  options.swept_astar = method_.recovery.swept_astar;
  options.hybrid_astar_recovery = method_.recovery.hybrid_astar;
  options.candidate_switch = method_.recovery.candidate_switch;
  options.tail_refinement = method_.recovery.tail_refinement;
  options.post_incumbent_svsdf_budget =
      method_.recovery.post_incumbent_svsdf_budget;
  options.tail_refinement_svsdf_budget =
      method_.recovery.tail_refinement_svsdf_budget;
  options.collect_risk_ground_truth = request.risk_ground_truth_enabled;
  options.risk_ground_truth_run_low = request.risk_ground_truth_run_low;
  options.risk_ground_truth_run_medium = request.risk_ground_truth_run_medium;
  options.risk_ground_truth_run_strict = request.risk_ground_truth_run_strict;
  runtime_.SetExperimentOptions(options);

  const std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();
  isweep_planner::SvsdfPlanResult raw;
  try {
    raw = runtime_.Plan(
        Eigen::Vector3d(request.start.x, request.start.y, request.start.theta),
        Eigen::Vector3d(request.goal.x, request.goal.y, request.goal.theta));
  } catch (const std::exception& exception) {
    output.status = PlannerStatus::INTERNAL_ERROR;
    output.failure_stage = "planner_call";
    output.failure_reason = exception.what();
  } catch (...) {
    output.status = PlannerStatus::INTERNAL_ERROR;
    output.failure_stage = "planner_call";
    output.failure_reason = "unknown exception";
  }
  output.metrics.t_total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - begin).count();

  output.planner_returned = raw.success && !raw.raw_trajectory.empty();
  output.internal_footprint_valid =
      output.planner_returned && std::isfinite(raw.stats.min_clearance) &&
      raw.stats.min_clearance > -0.051;
  output.timeout = raw.timeout;
  output.degraded = raw.degraded;
  output.trajectory = raw.raw_trajectory;
  if (output.degraded) {
    output.status = PlannerStatus::DEGRADED_DIAGNOSTIC;
    output.failure_stage = "internal_validation";
    output.failure_reason = "planner returned a degraded diagnostic trajectory";
  } else if (output.planner_returned) {
    // A validated incumbent remains executable when returned exactly at the
    // deadline.  The timeout field records budget pressure, not failure.
    output.status = PlannerStatus::PLANNER_RETURNED;
  } else if (output.timeout) {
    output.status = PlannerStatus::TIMEOUT;
    output.failure_stage = "planning_budget";
    output.failure_reason = "planner exhausted its configured budget without a validated incumbent";
  } else if (output.status != PlannerStatus::INTERNAL_ERROR) {
    output.status = PlannerStatus::NO_PATH;
    output.failure_stage = "planner";
    output.failure_reason = raw.success && output.trajectory.empty()
                                ? "planner returned only a discrete fallback path"
                                : "planner found no trajectory";
  }

  const isweep_planner::RuntimeExperimentTelemetry& telemetry =
      runtime_.last_experiment_telemetry();
  ExperimentMetrics& m = output.metrics;
  m.planner_status = output.status;
  m.failure_stage = output.failure_stage;
  m.failure_reason = output.failure_reason;
  m.planner_returned = output.planner_returned;
  m.internal_footprint_valid = output.internal_footprint_valid;
  m.degraded = output.degraded;
  m.timeout = output.timeout;
  m.map_resolution = runtime_.grid_map().resolution();
  m.minimum_internal_clearance_m = raw.stats.min_clearance;
  m.candidate_count = telemetry.candidate_count;
  m.selected_candidate_index = telemetry.selected_candidate_index;
  m.t_roadmap_ms = telemetry.t_roadmap_ms;
  m.t_dijkstra_ms = telemetry.t_dijkstra_ms;
  m.t_shortcut_ms = telemetry.t_shortcut_ms;
  m.t_path_adjust_ms = telemetry.t_path_adjust_ms;
  m.t_se2_sequence_ms = telemetry.t_se2_sequence_ms;
  m.t_risk_classification_ms = telemetry.t_risk_classification_ms;
  m.t_low_solve_ms = telemetry.t_low_solve_ms;
  m.t_medium_solve_ms = telemetry.t_medium_solve_ms;
  m.t_strict_solve_ms = telemetry.t_strict_solve_ms;
  m.t_full_svsdf_ms = telemetry.t_full_svsdf_ms;
  m.t_stitch_ms = telemetry.t_stitch_ms;
  m.t_internal_validation_ms = telemetry.t_internal_validation_ms;
  m.t_recovery_ms = telemetry.t_recovery_ms;
  m.t_first_feasible_ms = telemetry.t_first_feasible_ms;
  m.t_risk_ground_truth_ms = telemetry.t_risk_ground_truth_ms;
  m.low_solver_call_count = telemetry.low_solver_call_count;
  m.low_solver_success_count = telemetry.low_solver_success_count;
  m.medium_solver_call_count = telemetry.medium_solver_call_count;
  m.medium_solver_success_count = telemetry.medium_solver_success_count;
  m.strict_solver_call_count = telemetry.strict_solver_call_count;
  m.strict_solver_success_count = telemetry.strict_solver_success_count;
  m.full_svsdf_call_count = telemetry.full_svsdf_call_count;
  m.full_svsdf_success_count = telemetry.full_svsdf_success_count;
  m.full_svsdf_used = telemetry.full_svsdf_call_count > 0;
  m.full_svsdf_initial_assigned_call_count =
      telemetry.full_svsdf_initial_assigned_call_count;
  m.full_svsdf_initial_escalation_call_count =
      telemetry.full_svsdf_initial_escalation_call_count;
  m.full_svsdf_frontend_fallback_call_count =
      telemetry.full_svsdf_frontend_fallback_call_count;
  m.full_svsdf_validation_recovery_call_count =
      telemetry.full_svsdf_validation_recovery_call_count;
  m.full_svsdf_tail_refinement_call_count =
      telemetry.full_svsdf_tail_refinement_call_count;
  m.full_svsdf_other_call_count = telemetry.full_svsdf_other_call_count;
  m.preserve_shape_bypass_count = telemetry.preserve_shape_bypass_count;
  m.initial_level_upgrade_attempt_count =
      telemetry.initial_level_upgrade_attempt_count;
  m.initial_level_upgrade_success_count =
      telemetry.initial_level_upgrade_success_count;
  m.frontend_fallback_attempt_count =
      telemetry.frontend_fallback_attempt_count;
  m.frontend_fallback_success_count =
      telemetry.frontend_fallback_success_count;
  m.validation_recovery_attempt_count =
      telemetry.validation_recovery_attempt_count;
  m.validation_recovery_success_count =
      telemetry.validation_recovery_success_count;
  m.local_upgrade_attempt_count = telemetry.local_upgrade_attempt_count;
  m.local_upgrade_success_count = telemetry.local_upgrade_success_count;
  m.strict_patch_attempt_count = telemetry.strict_patch_attempt_count;
  m.strict_patch_success_count = telemetry.strict_patch_success_count;
  m.swept_astar_attempt_count = telemetry.swept_astar_attempt_count;
  m.swept_astar_success_count = telemetry.swept_astar_success_count;
  m.swept_astar_expanded_nodes = telemetry.swept_astar_expanded_nodes;
  m.hybrid_astar_attempt_count = telemetry.hybrid_astar_attempt_count;
  m.hybrid_astar_success_count = telemetry.hybrid_astar_success_count;
  m.candidate_switch_count = telemetry.candidate_switch_count;
  m.post_incumbent_svsdf_call_count =
      telemetry.post_incumbent_svsdf_call_count;
  m.post_incumbent_svsdf_budget_reject_count =
      telemetry.post_incumbent_svsdf_budget_reject_count;
  m.tail_refinement_svsdf_budget_reject_count =
      telemetry.tail_refinement_svsdf_budget_reject_count;
  m.external_equivalent_gate_attempt_count =
      telemetry.external_equivalent_gate_attempt_count;
  m.external_equivalent_gate_reject_count =
      telemetry.external_equivalent_gate_reject_count;
  m.incumbent_update_count = telemetry.incumbent_update_count;
  m.incumbent_reject_count = telemetry.incumbent_reject_count;
  m.validation_failure_count = telemetry.validation_failure_count;
  m.recovery_success = telemetry.recovery_success;
  m.risk_ground_truth_enabled = request.risk_ground_truth_enabled;
  m.original_trajectory_length_m = telemetry.original_trajectory_length_m;
  m.replanned_length_m = telemetry.replanned_length_m;
  m.patch_length_m = telemetry.patch_length_m;
  m.trajectory_reuse_ratio = telemetry.trajectory_reuse_ratio;

  for (const isweep_planner::RuntimeIncumbentEvent& source :
       telemetry.incumbent_events) {
    IncumbentEventMetric event;
    event.run_id = request.run_id;
    event.update_index = source.update_index;
    event.elapsed_ms = source.elapsed_ms;
    event.candidate_index = source.candidate_index;
    event.minimum_clearance_m = source.minimum_clearance_m;
    event.smoothness_cost = source.smoothness_cost;
    event.trajectory_length_m = source.trajectory_length_m;
    event.trajectory_duration_sec = source.trajectory_duration_sec;
    event.piece_count = source.piece_count;
    event.low_segment_count = source.low_segment_count;
    event.medium_segment_count = source.medium_segment_count;
    event.high_segment_count = source.high_segment_count;
    event.recovery_success = source.recovery_success;
    output.incumbent_events.push_back(event);
  }

  for (const isweep_planner::RuntimeRiskGroundTruthTrial& source :
       raw.risk_ground_truth_trials) {
    RiskGroundTruthMetric metric;
    metric.run_id = request.run_id;
    metric.candidate_index = source.candidate_index;
    metric.segment_index = source.segment_index;
    metric.assigned_risk_label =
        isweep_planner::RiskLevelName(source.assigned_level);
    metric.maximum_risk_score = source.maximum_risk_score;
    metric.mean_risk_score = source.mean_risk_score;
    metric.state_count = source.state_count;
    metric.segment_length_m = source.segment_length_m;
    metric.expected_start = source.expected_start;
    metric.expected_goal = source.expected_goal;
    metric.low_solver_attempted = source.low_solver_attempted;
    metric.low_solver_success = source.low_solver_success;
    metric.low_solve_ms = source.low_solve_ms;
    metric.low_trajectory = source.low_trajectory;
    metric.medium_solver_attempted = source.medium_solver_attempted;
    metric.medium_solver_success = source.medium_solver_success;
    metric.medium_solve_ms = source.medium_solve_ms;
    metric.medium_trajectory = source.medium_trajectory;
    metric.strict_solver_attempted = source.strict_solver_attempted;
    metric.strict_solver_success = source.strict_solver_success;
    metric.full_svsdf_used = source.full_svsdf_used;
    metric.full_svsdf_success = source.full_svsdf_success;
    metric.strict_solve_ms = source.strict_solve_ms;
    metric.full_svsdf_ms = source.full_svsdf_ms;
    metric.strict_trajectory = source.strict_trajectory;
    output.risk_ground_truth_metrics.push_back(std::move(metric));
  }

  if (!output.trajectory.empty()) {
    m.trajectory_duration_sec = output.trajectory.totalDuration();
    m.trajectory_length_m =
        isweep_planner::ApproximateTrajectoryLength(output.trajectory);
    m.smoothness_cost =
        isweep_planner::TrajectorySmoothnessCost(output.trajectory);
  }
  CollectReferenceMetrics(request, raw.risk_aware_global_reference, &output);
  return output;
}

void PlannerAdapter::CollectReferenceMetrics(
    const PlanningRequest& request,
    const isweep_planner::RiskAwareGlobalReferenceData& reference,
    PlanningResult* result) const {
  if (!result) return;
  const auto& map = runtime_.grid_map();
  const auto& checker = runtime_.collision_checker();
  const auto& footprint = runtime_.footprint();
  double arc_length = 0.0;
  std::map<int, std::vector<size_t>> segment_states;
  for (size_t i = 0; i < reference.points.size(); ++i) {
    const auto& point = reference.points[i];
    if (i > 0) {
      const auto& previous = reference.points[i - 1];
      arc_length += std::hypot(point.x - previous.x, point.y - previous.y);
    }
    StateMetric metric;
    metric.run_id = request.run_id;
    metric.candidate_index = result->metrics.selected_candidate_index;
    metric.state_index = static_cast<int>(i);
    metric.arc_length_m = arc_length;
    metric.x = point.x;
    metric.y = point.y;
    metric.selected_yaw = point.yaw;
    metric.center_clearance_m = map.getEsdf(point.x, point.y);
    metric.footprint_clearance_m =
        FootprintClearance(map, footprint, point.x, point.y, point.yaw);
    const std::vector<int> safe = checker.safeYawIndices(point.x, point.y);
    metric.safe_yaw_count = static_cast<int>(safe.size());
    metric.largest_contiguous_safe_yaw_bins =
        LargestContiguousSafeYawBins(safe, checker.numYawBins());
    metric.risk_score = point.risk_score;
    metric.effective_clearance_m = point.effective_clearance;
    metric.feasible_yaw_width = point.feasible_yaw_width;
    metric.feasible_yaw_overlap = point.feasible_yaw_overlap;
    metric.correction_distance_m = point.correction_distance;
    metric.uncertainty_erosion_m = point.uncertainty_erosion;
    metric.clearance_risk = point.clearance_risk;
    metric.orientation_width_risk = point.orientation_width_risk;
    metric.orientation_continuity_risk =
        point.orientation_continuity_risk;
    metric.correction_risk = point.correction_risk;
    metric.uncertainty_risk = point.uncertainty_risk;
    metric.risk_label =
        point.risk_level >= isweep_planner::RiskLevelRank(isweep_planner::RiskLevel::HIGH)
            ? "HIGH"
            : (point.risk_level >=
                       isweep_planner::RiskLevelRank(isweep_planner::RiskLevel::MEDIUM)
                   ? "MEDIUM"
                   : "LOW");
    metric.source_segment_index = point.segment_id;
    result->state_metrics.push_back(metric);
    segment_states[point.segment_id].push_back(i);
  }

  for (const auto& entry : segment_states) {
    SegmentMetric segment;
    segment.run_id = request.run_id;
    segment.candidate_index = result->metrics.selected_candidate_index;
    segment.segment_index = entry.first;
    segment.start_state_index = static_cast<int>(entry.second.front());
    segment.end_state_index = static_cast<int>(entry.second.back());
    segment.state_count = static_cast<int>(entry.second.size());
    segment.risk_label =
        result->state_metrics[entry.second.front()].risk_label;
    double safe_sum = 0.0;
    double length = 0.0;
    double center_min = isweep_planner::kInf;
    double footprint_min = isweep_planner::kInf;
    for (size_t offset = 0; offset < entry.second.size(); ++offset) {
      const StateMetric& state = result->state_metrics[entry.second[offset]];
      safe_sum += state.safe_yaw_count;
      center_min = std::min(center_min, state.center_clearance_m);
      footprint_min = std::min(footprint_min, state.footprint_clearance_m);
      if (offset > 0) {
        const StateMetric& previous =
            result->state_metrics[entry.second[offset - 1]];
        length += std::hypot(state.x - previous.x, state.y - previous.y);
      }
    }
    segment.segment_length_m = length;
    segment.minimum_center_clearance_m = center_min;
    segment.minimum_footprint_clearance_m = footprint_min;
    segment.mean_safe_yaw_count = safe_sum / entry.second.size();
    segment.strict_called = segment.risk_label == "HIGH";
    // HIGH is a semantic contract, not an experiment-only force flag: every
    // successfully returned HIGH segment has executed the full SVSDF backend.
    segment.full_svsdf_used = segment.strict_called;
    segment.full_svsdf_metric_available = true;
    segment.solver_success = result->planner_returned;
    result->segment_metrics.push_back(segment);
  }

  ExperimentMetrics& m = result->metrics;
  m.state_count = static_cast<int>(result->state_metrics.size());
  for (const StateMetric& state : result->state_metrics) {
    if (state.risk_label == "HIGH") ++m.high_state_count;
    else if (state.risk_label == "MEDIUM") ++m.medium_state_count;
    else ++m.low_state_count;
    if (!std::isfinite(m.minimum_center_clearance_m) ||
        state.center_clearance_m < m.minimum_center_clearance_m) {
      m.minimum_center_clearance_m = state.center_clearance_m;
    }
    if (!std::isfinite(m.minimum_internal_footprint_clearance_m) ||
        state.footprint_clearance_m < m.minimum_internal_footprint_clearance_m) {
      m.minimum_internal_footprint_clearance_m = state.footprint_clearance_m;
    }
    if (m.minimum_safe_yaw_count < 0 || state.safe_yaw_count < m.minimum_safe_yaw_count) {
      m.minimum_safe_yaw_count = state.safe_yaw_count;
    }
    if (!std::isfinite(m.mean_safe_yaw_count)) m.mean_safe_yaw_count = 0.0;
    m.mean_safe_yaw_count += state.safe_yaw_count;
  }
  if (m.state_count > 0) {
    m.high_state_ratio = static_cast<double>(m.high_state_count) / m.state_count;
    m.mean_safe_yaw_count /= m.state_count;
  }
  m.segment_count = static_cast<int>(result->segment_metrics.size());
  m.segment_boundary_count = std::max(0, m.segment_count - 1);
  for (const SegmentMetric& segment : result->segment_metrics) {
    if (segment.risk_label == "HIGH") ++m.high_segment_count;
    else if (segment.risk_label == "MEDIUM") ++m.medium_segment_count;
    else ++m.low_segment_count;
  }
}

}  // namespace planner_experiments
