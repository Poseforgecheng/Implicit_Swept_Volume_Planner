#include "isweep_planner/framework/svsdf_runtime.h"
#include "isweep_planner/ros_interface/ros_visualizer.h"
#include "isweep_planner/framework/recovery_manager.h"
#include "isweep_planner/global_search/swept_astar.h"
#include "isweep_planner/core/trajectory_utils.h"
#include "isweep_planner/core/math_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <string>

#include <geometry_msgs/PoseStamped.h>

namespace isweep_planner {

namespace {

void ScaleTrajectoryTimeForGroundTruth(double scale, Trajectory* trajectory) {
  if (trajectory == nullptr || trajectory->empty() || scale <= 1.0) return;
  for (PolyPiece& piece : trajectory->pos_pieces) {
    piece.duration *= scale;
    double divisor = scale;
    for (int order = 1; order < piece.coeffs.cols(); ++order) {
      piece.coeffs.col(order) /= divisor;
      divisor *= scale;
    }
  }
  for (YawPolyPiece& piece : trajectory->yaw_pieces) {
    piece.duration *= scale;
    double divisor = scale;
    for (int order = 1; order < piece.coeffs.cols(); ++order) {
      piece.coeffs.col(order) /= divisor;
      divisor *= scale;
    }
  }
}

void NormalizeGroundTruthDynamics(
    const OptimizerParams& params, double max_yaw_acceleration,
    double velocity_tolerance, double acceleration_tolerance,
    double yaw_rate_tolerance, double yaw_acceleration_tolerance,
    Trajectory* trajectory) {
  if (trajectory == nullptr || trajectory->empty()) return;
  const double duration = trajectory->totalDuration();
  const int sample_count =
      std::max(1, static_cast<int>(std::ceil(duration / 0.02)));
  double max_velocity = 0.0;
  double max_acceleration = 0.0;
  double max_yaw_rate = 0.0;
  double max_yaw_acceleration_observed = 0.0;
  for (int sample = 0; sample <= sample_count; ++sample) {
    const double time = duration * static_cast<double>(sample) /
                        static_cast<double>(sample_count);
    max_velocity =
        std::max(max_velocity, trajectory->sampleVelocity(time).norm());
    max_acceleration =
        std::max(max_acceleration, trajectory->sampleAcceleration(time).norm());
    max_yaw_rate =
        std::max(max_yaw_rate, std::abs(trajectory->sampleYawRate(time)));
    max_yaw_acceleration_observed = std::max(
        max_yaw_acceleration_observed,
        std::abs(trajectory->sampleYawAcceleration(time)));
  }
  double required_scale = 1.0;
  required_scale = std::max(
      required_scale,
      max_velocity / std::max(1e-6, params.max_vel + velocity_tolerance));
  required_scale = std::max(
      required_scale,
      std::sqrt(max_acceleration /
                std::max(1e-6, params.max_acc + acceleration_tolerance)));
  required_scale = std::max(
      required_scale,
      max_yaw_rate /
          std::max(1e-6, params.max_yaw_rate + yaw_rate_tolerance));
  required_scale = std::max(
      required_scale,
      std::sqrt(max_yaw_acceleration_observed /
                std::max(1e-6, max_yaw_acceleration +
                                   yaw_acceleration_tolerance)));
  if (required_scale > 1.0 + 1e-6) {
    ScaleTrajectoryTimeForGroundTruth(1.02 * required_scale, trajectory);
  }
}

}  // namespace

void SvsdfRuntime::PushSegmentWaypointsFromObstacles(
    std::vector<MotionSegment>* segments) const {
  if (segments == nullptr) return;
  const double push_margin = 0.40;
  const double push_step = 0.12;
  const int max_iters = 20;

  for (auto& seg : *segments) {
    for (auto& wp : seg.waypoints) {
      Eigen::Vector2d pos(wp.x, wp.y);
      math::PushPointFromObstacle(grid_map_, pos, push_margin, push_step, max_iters);
      wp.x = pos.x();
      wp.y = pos.y();
    }
  }

  for (size_t i = 0; i + 1 < segments->size(); ++i) {
    if (!(*segments)[i].waypoints.empty() && !(*segments)[i + 1].waypoints.empty()) {
      (*segments)[i + 1].waypoints.front() = (*segments)[i].waypoints.back();
    }
  }
}

void SvsdfRuntime::Initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  const std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();
  LoadParameters(pnh);
  strict_solver_.Initialize(nh, pnh, footprint_);
  last_initialization_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - begin).count();
  initialized_ = true;
}

void SvsdfRuntime::LoadParameters(ros::NodeHandle& pnh) {
  footprint_.setPolygon(LoadFootprint(pnh));

  double inscribed_radius = 0.08;
  pnh.param("inscribed_radius", inscribed_radius, 0.08);
  footprint_.setInscribedRadius(inscribed_radius);

  pnh.param("topology/num_samples", topology_num_samples_, 520);
  pnh.param("topology/knn", topology_knn_, 18);
  pnh.param("topology/max_paths", topology_max_paths_, 14);

  pnh.param("se2/discretization_step", se2_disc_step_, 0.15);
  pnh.param("kernel_yaw_num", yaw_bins_, 18);
  pnh.param("se2/max_push_attempts", max_push_attempts_, 5);
  pnh.param("se2/corner_smoothing_iterations",
            se2_corner_smoothing_iterations_, 0);
  pnh.param("se2/corner_smoothing_alpha",
            se2_corner_smoothing_alpha_, 0.45);
  se2_corner_smoothing_iterations_ =
      std::max(0, se2_corner_smoothing_iterations_);
  se2_corner_smoothing_alpha_ =
      std::max(0.0, std::min(0.75, se2_corner_smoothing_alpha_));
  pnh.param("reference/sample_dt", reference_sample_dt_, 0.15);
  pnh.param("reference/min_preferred_speed", reference_min_preferred_speed_, 0.35);
  pnh.param("reference/max_preferred_speed", reference_max_preferred_speed_, 0.85);
  pnh.param("anytime/continue_after_first_solution",
            experiment_options_.candidate_switch, false);
  pnh.param("anytime/refine_after_first_solution",
            experiment_options_.tail_refinement, false);
  pnh.param("anytime/total_time_budget_sec",
            experiment_options_.total_time_budget_sec, 20.0);
  pnh.param("anytime/post_incumbent_svsdf_budget",
            experiment_options_.post_incumbent_svsdf_budget, 4);
  pnh.param("anytime/tail_refinement_svsdf_budget",
            experiment_options_.tail_refinement_svsdf_budget, 1);

  experiment_options_.total_time_budget_sec =
      std::max(0.1, experiment_options_.total_time_budget_sec);

  pnh.param("risk/enable_three_level",
            continuous_risk_params_.enable_three_level, true);
  pnh.param("risk/low_threshold", continuous_risk_params_.low_threshold, 0.35);
  pnh.param("risk/high_threshold", continuous_risk_params_.high_threshold, 0.65);
  pnh.param("risk/hysteresis", continuous_risk_params_.hysteresis, 0.05);
  pnh.param("risk/guard_states", continuous_risk_params_.guard_states, 1);
  pnh.param("risk/clearance_scale", continuous_risk_params_.clearance_scale, 0.35);
  pnh.param("risk/correction_distance_scale",
            continuous_risk_params_.correction_distance_scale, 0.30);
  pnh.param("risk/localization_sigma", continuous_risk_params_.localization_sigma, 0.02);
  pnh.param("risk/localization_sigma_scale",
            continuous_risk_params_.localization_sigma_scale, 3.0);
  pnh.param("risk/control_error", continuous_risk_params_.control_error, 0.03);
  pnh.param("risk/map_error", continuous_risk_params_.map_error, 0.02);
  pnh.param("risk/shape_error", continuous_risk_params_.shape_error, 0.01);
  pnh.param("risk/weight_clearance", continuous_risk_params_.weight_clearance, 0.90);
  pnh.param("risk/weight_orientation_width",
            continuous_risk_params_.weight_orientation_width, 0.70);
  pnh.param("risk/weight_orientation_continuity",
            continuous_risk_params_.weight_orientation_continuity, 0.80);
  pnh.param("risk/weight_correction", continuous_risk_params_.weight_correction, 0.60);
  pnh.param("risk/weight_uncertainty", continuous_risk_params_.weight_uncertainty, 0.75);

  pnh.param("validation/min_clearance", validation_min_clearance_, 0.0);
  pnh.param("validation/velocity_tolerance", validation_velocity_tolerance_, 1e-3);
  pnh.param("validation/acceleration_tolerance", validation_acceleration_tolerance_, 1e-2);
  pnh.param("validation/yaw_rate_tolerance", validation_yaw_rate_tolerance_, 1e-3);
  pnh.param("validation/max_yaw_acceleration",
            validation_max_yaw_acceleration_, 6.0);
  pnh.param("validation/yaw_acceleration_tolerance",
            validation_yaw_acceleration_tolerance_, 1e-2);
  pnh.param("validation/position_jump_tolerance",
            validation_position_jump_tolerance_, 1e-4);
  pnh.param("validation/velocity_jump_tolerance",
            validation_velocity_jump_tolerance_, 1e-3);
  pnh.param("validation/acceleration_jump_tolerance",
            validation_acceleration_jump_tolerance_, 1e-2);
  pnh.param("validation/yaw_jump_tolerance", validation_yaw_jump_tolerance_, 1e-3);
  pnh.param("validation/yaw_rate_jump_tolerance",
            validation_yaw_rate_jump_tolerance_, 1e-2);
  pnh.param("validation/time_step_sec",
            footprint_validation_config_.time_step_sec, 0.01);
  pnh.param("validation/max_translation_step_m",
            footprint_validation_config_.max_translation_step_m, 0.01);
  pnh.param("validation/max_yaw_step_rad",
            footprint_validation_config_.max_yaw_step_rad, 0.01);
  pnh.param("validation/footprint_boundary_step_m",
            footprint_validation_config_.footprint_boundary_step_m, 0.01);
  pnh.param("validation/footprint_interior_step_m",
            footprint_validation_config_.footprint_interior_step_m, 0.02);
  pnh.param("validation/collision_clearance_m",
            footprint_validation_config_.collision_clearance_m, 0.0);
  pnh.param("validation/max_subdivision_depth",
            footprint_validation_config_.max_subdivision_depth, 24);

  continuous_risk_params_.low_threshold =
      std::max(0.0, std::min(0.999, continuous_risk_params_.low_threshold));
  continuous_risk_params_.high_threshold =
      std::max(continuous_risk_params_.low_threshold + 1e-3,
               std::min(1.0, continuous_risk_params_.high_threshold));
  continuous_risk_params_.hysteresis =
      std::max(0.0, continuous_risk_params_.hysteresis);
  continuous_risk_params_.guard_states =
      std::max(0, continuous_risk_params_.guard_states);
  continuous_risk_params_.clearance_scale =
      std::max(1e-3, continuous_risk_params_.clearance_scale);
  continuous_risk_params_.correction_distance_scale =
      std::max(1e-3, continuous_risk_params_.correction_distance_scale);
  continuous_risk_params_.localization_sigma =
      std::max(0.0, continuous_risk_params_.localization_sigma);
  continuous_risk_params_.localization_sigma_scale =
      std::max(0.0, continuous_risk_params_.localization_sigma_scale);
  continuous_risk_params_.control_error =
      std::max(0.0, continuous_risk_params_.control_error);
  continuous_risk_params_.map_error =
      std::max(0.0, continuous_risk_params_.map_error);
  continuous_risk_params_.shape_error =
      std::max(0.0, continuous_risk_params_.shape_error);
  validation_min_clearance_ = std::max(0.0, validation_min_clearance_);
  validation_max_yaw_acceleration_ =
      std::max(0.0, validation_max_yaw_acceleration_);

  pnh.param("optimizer/max_iterations", optimizer_params_.max_iterations, 100);
  pnh.param("optimizer/lambda_smooth", optimizer_params_.lambda_smooth, 1.0);
  pnh.param("optimizer/lambda_time", optimizer_params_.lambda_time, 1.0);
  pnh.param("optimizer/lambda_safety", optimizer_params_.lambda_safety, 10.0);
  pnh.param("optimizer/lambda_dynamics", optimizer_params_.lambda_dynamics, 1.0);
  pnh.param("optimizer/lambda_pos_residual",
            optimizer_params_.lambda_pos_residual, 5.0);
  pnh.param("optimizer/lambda_yaw_residual",
            optimizer_params_.lambda_yaw_residual, 2.0);
  pnh.param("optimizer/max_vel", optimizer_params_.max_vel, 1.0);
  pnh.param("optimizer/max_acc", optimizer_params_.max_acc, 2.0);
  pnh.param("optimizer/max_yaw_rate", optimizer_params_.max_yaw_rate, 1.5);
  pnh.param("optimizer/safety_margin", optimizer_params_.safety_margin, 0.05);
  pnh.param("optimizer/step_size", optimizer_params_.step_size, 0.005);
  pnh.param("optimizer/fit_max_piece_length_r2",
            optimizer_params_.fit_max_piece_length_r2, 1.0);
  pnh.param("optimizer/fit_max_piece_length_se2",
            optimizer_params_.fit_max_piece_length_se2, 1.5);
  pnh.param("optimizer/fit_max_heading_change",
            optimizer_params_.fit_max_heading_change, 4.0);
  pnh.param("optimizer/stitch_tangent_speed_scale",
            optimizer_params_.stitch_tangent_speed_scale, 0.35);
  pnh.param("optimizer/stitch_max_tangent_speed_ratio",
            optimizer_params_.stitch_max_tangent_speed_ratio, 0.60);
  pnh.param("optimizer/stitch_guard_pieces",
            optimizer_params_.stitch_guard_pieces, 0);
  pnh.param("optimizer/post_stitch_smoothing_iterations",
            optimizer_params_.post_stitch_smoothing_iterations, 0);
  pnh.param("optimizer/post_stitch_smoothing_alpha",
            optimizer_params_.post_stitch_smoothing_alpha, 0.40);
  pnh.param("optimizer/post_stitch_sample_spacing",
            optimizer_params_.post_stitch_sample_spacing, 0.12);
  optimizer_params_.stitch_tangent_speed_scale =
      std::max(0.0, std::min(1.0,
                            optimizer_params_.stitch_tangent_speed_scale));
  optimizer_params_.stitch_max_tangent_speed_ratio =
      std::max(0.0, std::min(1.0,
                            optimizer_params_.stitch_max_tangent_speed_ratio));
  optimizer_params_.stitch_guard_pieces =
      std::max(0, std::min(1, optimizer_params_.stitch_guard_pieces));
  optimizer_params_.post_stitch_smoothing_iterations =
      std::max(0, optimizer_params_.post_stitch_smoothing_iterations);
  optimizer_params_.post_stitch_smoothing_alpha =
      std::max(0.0, std::min(0.75,
                            optimizer_params_.post_stitch_smoothing_alpha));
  optimizer_params_.post_stitch_sample_spacing =
      std::max(0.05, optimizer_params_.post_stitch_sample_spacing);
  optimizer_params_.fit_max_piece_length_r2 =
      std::max(0.05, optimizer_params_.fit_max_piece_length_r2);
  optimizer_params_.fit_max_piece_length_se2 =
      std::max(0.05, optimizer_params_.fit_max_piece_length_se2);
  optimizer_params_.fit_max_heading_change =
      std::max(0.01, optimizer_params_.fit_max_heading_change);
  pnh.param("optimizer/coalesce_escalated_equal_level_segments",
            coalesce_escalated_equal_level_segments_, false);

  pnh.param("hybrid_astar/step_size", hybrid_astar_params_.step_size, 0.35);
  pnh.param("hybrid_astar/wheel_base", hybrid_astar_params_.wheel_base, 0.8);
  pnh.param("hybrid_astar/max_steer", hybrid_astar_params_.max_steer, 0.6);
  pnh.param("hybrid_astar/steer_samples", hybrid_astar_params_.steer_samples, 5);
  pnh.param("hybrid_astar/goal_tolerance_pos",
            hybrid_astar_params_.goal_tolerance_pos, 0.35);
  pnh.param("hybrid_astar/goal_tolerance_yaw",
            hybrid_astar_params_.goal_tolerance_yaw, 0.4);
  pnh.param("hybrid_astar/reverse_penalty",
            hybrid_astar_params_.reverse_penalty, 2.0);
  pnh.param("hybrid_astar/steer_penalty",
            hybrid_astar_params_.steer_penalty, 0.2);
  pnh.param("hybrid_astar/steer_change_penalty",
            hybrid_astar_params_.steer_change_penalty, 0.2);
  pnh.param("hybrid_astar/switch_penalty",
            hybrid_astar_params_.switch_penalty, 2.0);
  pnh.param("hybrid_astar/max_expansions",
            hybrid_astar_params_.max_expansions, 50000);
  pnh.param("hybrid_astar/fast_pass_max_expansions",
            hybrid_astar_params_.fast_pass_max_expansions, 8000);
  pnh.param("hybrid_astar/fast_pass_steer_samples",
            hybrid_astar_params_.fast_pass_steer_samples, 3);
  pnh.param("hybrid_astar/heuristic_weight",
            hybrid_astar_params_.heuristic_weight, 1.0);
  hybrid_astar_params_.heuristic_weight =
      std::max(1.0, hybrid_astar_params_.heuristic_weight);
  pnh.param("hybrid_astar/pose_recovery_max_radius",
            hybrid_astar_params_.pose_recovery_max_radius, 0.6);
  pnh.param("hybrid_astar/pose_recovery_radial_step",
            hybrid_astar_params_.pose_recovery_radial_step, 0.05);
  pnh.param("hybrid_astar/pose_recovery_angular_samples",
            hybrid_astar_params_.pose_recovery_angular_samples, 24);
}

bool SvsdfRuntime::UpdateMap(const nav_msgs::OccupancyGrid& map) {
  if (!initialized_) {
    return false;
  }

  latest_map_ = map;
  nav_msgs::OccupancyGrid::Ptr map_ptr(new nav_msgs::OccupancyGrid(map));
  const std::chrono::steady_clock::time_point esdf_begin =
      std::chrono::steady_clock::now();
  grid_map_.fromOccupancyGrid(map_ptr);
  last_esdf_build_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - esdf_begin).count();

  const std::chrono::steady_clock::time_point precompute_begin =
      std::chrono::steady_clock::now();
  collision_checker_.init(grid_map_, footprint_, yaw_bins_);
  se2_generator_.init(grid_map_, collision_checker_, se2_disc_step_,
                      max_push_attempts_, continuous_risk_params_,
                      se2_corner_smoothing_iterations_,
                      se2_corner_smoothing_alpha_);
  hybrid_astar_.init(grid_map_, collision_checker_, hybrid_astar_params_);
  const double topo_safe_dist = 0.25;
  topology_planner_.init(grid_map_, collision_checker_, topology_num_samples_,
                         topology_knn_, topology_max_paths_, topo_safe_dist);
  svsdf_evaluator_.initGridEsdf(grid_map_, footprint_);
  optimizer_.init(grid_map_, svsdf_evaluator_, optimizer_params_);
  if (!strict_solver_.UpdateMap(map)) {
    last_map_precompute_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - precompute_begin).count();
    map_ready_ = false;
    return false;
  }
  strict_solver_.setGridMapForMidend(&grid_map_);
  last_map_precompute_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - precompute_begin).count();

  map_ready_ = true;
  return true;
}





bool SvsdfRuntime::SolveStrictSegment(const std::vector<SE2State>& waypoints,
                                      Trajectory* trajectory,
                                      bool require_full_svsdf,
                                      bool allow_shape_preserving_rollback,
                                      FullSvsdfCallSource source) {
  if (trajectory == nullptr) {
    return false;
  }
  const bool tail_call = source == FullSvsdfCallSource::TAIL_REFINEMENT;
  const bool measurement_call = source == FullSvsdfCallSource::MEASUREMENT;
  if (tail_call && experiment_options_.tail_refinement_svsdf_budget >= 0 &&
      experiment_telemetry_.full_svsdf_tail_refinement_call_count >=
          experiment_options_.tail_refinement_svsdf_budget) {
    ++experiment_telemetry_.tail_refinement_svsdf_budget_reject_count;
    ROS_INFO("Tail full-SVSDF call rejected by per-run quota %d",
             experiment_options_.tail_refinement_svsdf_budget);
    return false;
  }
  if (!tail_call && !measurement_call && post_incumbent_budget_active_ &&
      experiment_options_.post_incumbent_svsdf_budget >= 0 &&
      experiment_telemetry_.post_incumbent_svsdf_call_count >=
          experiment_options_.post_incumbent_svsdf_budget) {
    ++experiment_telemetry_.post_incumbent_svsdf_budget_reject_count;
    ROS_INFO("Post-incumbent full-SVSDF call rejected by per-run quota %d",
             experiment_options_.post_incumbent_svsdf_budget);
    return false;
  }
  const double raw_clearance = svsdf_evaluator_.segmentClearance(waypoints, 0.10);
  const bool raw_chain_feasible = std::isfinite(raw_clearance) &&
                                  raw_clearance >= validation_min_clearance_;
  // Keep the dense corner geometry.  If footprint sampling detects a
  // map-cell penetration, move only the affected interior states along the
  // footprint SVSDF gradient.  The former sparsify-then-center-push path cut
  // across L-corners and removed the anchors the strict solver needed.
  std::vector<SE2State> strict_waypoints = waypoints;
  if (!raw_chain_feasible) {
    const double target_clearance =
        validation_min_clearance_ + std::max(0.02, 0.5 * grid_map_.resolution());
    const double max_step = std::max(0.02, grid_map_.resolution());
    for (size_t i = 1; i + 1 < strict_waypoints.size(); ++i) {
      for (int iter = 0; iter < 12; ++iter) {
        const double clearance = svsdf_evaluator_.evaluate(strict_waypoints[i]);
        if (std::isfinite(clearance) && clearance >= target_clearance) break;
        Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();
        double grad_yaw = 0.0;
        svsdf_evaluator_.gradient(strict_waypoints[i], grad_pos, grad_yaw);
        if (!grad_pos.allFinite() || grad_pos.norm() < 1e-8) break;
        const double deficit = std::isfinite(clearance)
                                   ? target_clearance - clearance
                                   : max_step;
        const double step =
            std::max(0.25 * grid_map_.resolution(), std::min(max_step, deficit));
        grad_pos.normalize();
        strict_waypoints[i].x += step * grad_pos.x();
        strict_waypoints[i].y += step * grad_pos.y();
      }
    }
  }
  const double prepared_clearance =
      svsdf_evaluator_.segmentClearance(strict_waypoints, 0.05);
  const bool prepared_chain_feasible = std::isfinite(prepared_clearance) &&
                                       prepared_clearance >=
                                           validation_min_clearance_;
  ROS_INFO(
      "Runtime stage: strict SE(2) input kept dense at %zu points "
      "(raw_clearance=%.3f prepared_clearance=%.3f)",
      strict_waypoints.size(), raw_clearance, prepared_clearance);

  const std::chrono::steady_clock::time_point solve_start =
      std::chrono::steady_clock::now();
  ++experiment_telemetry_.strict_solver_call_count;
  const bool solved = strict_solver_.ready() &&
      strict_solver_.Solve(strict_waypoints, trajectory, prepared_chain_feasible,
                           require_full_svsdf ||
                               experiment_options_.force_full_svsdf);
  const double solve_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - solve_start).count();
  experiment_telemetry_.t_strict_solve_ms += solve_ms;
  const Se2SvsdfSolver::SolveDiagnostics& diagnostics =
      strict_solver_.lastSolveDiagnostics();
  if (diagnostics.full_svsdf_used) {
    ++experiment_telemetry_.full_svsdf_call_count;
    if (post_incumbent_budget_active_ && !tail_call && !measurement_call) {
      ++experiment_telemetry_.post_incumbent_svsdf_call_count;
    }
    switch (source) {
      case FullSvsdfCallSource::INITIAL_ASSIGNED:
        ++experiment_telemetry_.full_svsdf_initial_assigned_call_count;
        break;
      case FullSvsdfCallSource::INITIAL_ESCALATION:
        ++experiment_telemetry_.full_svsdf_initial_escalation_call_count;
        break;
      case FullSvsdfCallSource::FRONTEND_FALLBACK:
        ++experiment_telemetry_.full_svsdf_frontend_fallback_call_count;
        break;
      case FullSvsdfCallSource::VALIDATION_RECOVERY:
        ++experiment_telemetry_.full_svsdf_validation_recovery_call_count;
        break;
      case FullSvsdfCallSource::TAIL_REFINEMENT:
        ++experiment_telemetry_.full_svsdf_tail_refinement_call_count;
        break;
      case FullSvsdfCallSource::MEASUREMENT:
        // Measurement telemetry is restored after risk-ground-truth
        // collection and must never contaminate the main planning counters.
        break;
      case FullSvsdfCallSource::OTHER:
        ++experiment_telemetry_.full_svsdf_other_call_count;
        break;
    }
    experiment_telemetry_.t_full_svsdf_ms += diagnostics.full_svsdf_elapsed_ms;
  }
  if (diagnostics.full_svsdf_success) {
    ++experiment_telemetry_.full_svsdf_success_count;
  }
  if (diagnostics.preserve_shape_bypass) {
    ++experiment_telemetry_.preserve_shape_bypass_count;
  }
  if (!solved ||
      trajectory->empty()) {
    return false;
  }
  ++experiment_telemetry_.strict_solver_success_count;

  const double solved_clearance = svsdf_evaluator_.evaluateTrajectory(*trajectory);
  if (allow_shape_preserving_rollback && prepared_chain_feasible &&
      solved_clearance + 1e-9 < validation_min_clearance_) {
    ROS_WARN(
        "Full SVSDF degraded a feasible dense chain: prepared_clearance=%.3f "
        "solved_clearance=%.3f; trying validated shape-preserving rollback",
        prepared_clearance, solved_clearance);
    Trajectory rollback;
    if (strict_solver_.Solve(strict_waypoints, &rollback, true, false) &&
        !rollback.empty()) {
      const double rollback_clearance =
          svsdf_evaluator_.evaluateTrajectory(rollback);
      if (rollback_clearance + 1e-9 >= validation_min_clearance_) {
        *trajectory = rollback;
        ROS_INFO("Accepted strict shape-preserving rollback: clearance=%.3f",
                 rollback_clearance);
      }
    }
  }

  return true;
}

bool SvsdfRuntime::OptimizeLowRiskSegment(const std::vector<SE2State>& waypoints,
                                          double seg_time,
                                          Trajectory* trajectory) {
  if (trajectory == nullptr) {
    return false;
  }

  // Use a balanced sparsification for low-risk segments to maintain
  // continuity at stitching points while keeping open space smooth.
  std::vector<SE2State> sparse_waypoints;
  if (waypoints.size() > 4) {
    sparse_waypoints.push_back(waypoints.front());
    sparse_waypoints.push_back(waypoints[waypoints.size() / 2]);
    sparse_waypoints.push_back(waypoints.back());
  } else {
    sparse_waypoints = waypoints;
  }

  const std::chrono::steady_clock::time_point solve_start =
      std::chrono::steady_clock::now();
  ++experiment_telemetry_.low_solver_call_count;
  *trajectory = optimizer_.optimizeR2(sparse_waypoints, seg_time);
  experiment_telemetry_.t_low_solve_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - solve_start).count();
  if (!trajectory->empty()) {
    ++experiment_telemetry_.low_solver_success_count;
    return true;
  }
  return false;
}

bool SvsdfRuntime::OptimizeMediumRiskSegment(
    const std::vector<SE2State>& waypoints, double seg_time,
    Trajectory* trajectory, bool preserve_failed_trajectory) {
  if (trajectory == nullptr || waypoints.size() < 2) return false;
  const std::chrono::steady_clock::time_point solve_start =
      std::chrono::steady_clock::now();
  ++experiment_telemetry_.medium_solver_call_count;
  std::string failure_reason;
  Trajectory last_nonempty;
  for (double time_scale : {1.0, 1.5, 2.25}) {
    *trajectory = optimizer_.optimizeSE2(
        waypoints, std::max(0.5, seg_time * time_scale));
    if (trajectory->empty()) continue;
    last_nonempty = *trajectory;
    double clearance = -kInf;
    if (IsTrajectoryFeasible(*trajectory, &clearance, nullptr, nullptr,
                             nullptr, &failure_reason)) {
      experiment_telemetry_.t_medium_solve_ms +=
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - solve_start)
              .count();
      ++experiment_telemetry_.medium_solver_success_count;
      return true;
    }
    // Extra time only repairs dynamics.  A geometric failure should escalate
    // immediately to HIGH rather than repeating the same SE(2) geometry.
    if (failure_reason.find("clearance") != std::string::npos) break;
  }
  experiment_telemetry_.t_medium_solve_ms +=
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - solve_start)
          .count();
  *trajectory = preserve_failed_trajectory ? last_nonempty : Trajectory();
  return false;
}

bool SvsdfRuntime::IsTrajectoryFeasible(const Trajectory& traj, double* min_clearance,
                                        double* max_vel, double* max_acc,
                                        double* max_yaw_rate,
                                        std::string* failure_reason,
                                        double* failure_time) const {
  if (failure_time != nullptr) {
    *failure_time = std::numeric_limits<double>::quiet_NaN();
  }
  const auto fail = [&](const std::string& reason) {
    if (failure_reason != nullptr) *failure_reason = reason;
    return false;
  };
  const auto fail_at = [&](const std::string& reason, double time) {
    if (failure_reason != nullptr) *failure_reason = reason;
    if (failure_time != nullptr) *failure_time = time;
    return false;
  };
  if (traj.empty()) {
    return fail("empty trajectory");
  }
  if (traj.pos_pieces.size() != traj.yaw_pieces.size()) {
    return fail("position/yaw piece count mismatch");
  }

  double clearance = svsdf_evaluator_.evaluateTrajectory(traj);
  double vmax = 0.0;
  double amax = 0.0;
  double yaw_rate_max = 0.0;
  double yaw_acceleration_max = 0.0;
  double vmax_time = 0.0;
  double amax_time = 0.0;
  double yaw_rate_max_time = 0.0;
  double yaw_acceleration_max_time = 0.0;
  const double total = traj.totalDuration();
  if (!std::isfinite(total) || total <= 0.0) {
    return fail("non-positive or non-finite trajectory duration");
  }
  const int sample_count = std::max(1, static_cast<int>(std::ceil(total / 0.02)));
  for (int sample_index = 0; sample_index <= sample_count; ++sample_index) {
    const double t = total * static_cast<double>(sample_index) /
                     static_cast<double>(sample_count);
    const double velocity = traj.sampleVelocity(t).norm();
    const double acceleration = traj.sampleAcceleration(t).norm();
    const double yaw_rate = std::abs(traj.sampleYawRate(t));
    const double yaw_acceleration =
        std::abs(traj.sampleYawAcceleration(t));
    if (velocity > vmax) {
      vmax = velocity;
      vmax_time = t;
    }
    if (acceleration > amax) {
      amax = acceleration;
      amax_time = t;
    }
    if (yaw_rate > yaw_rate_max) {
      yaw_rate_max = yaw_rate;
      yaw_rate_max_time = t;
    }
    if (yaw_acceleration > yaw_acceleration_max) {
      yaw_acceleration_max = yaw_acceleration;
      yaw_acceleration_max_time = t;
    }
  }

  if (min_clearance) {
    *min_clearance = clearance;
  }
  if (max_vel) {
    *max_vel = vmax;
  }
  if (max_acc) {
    *max_acc = amax;
  }
  if (max_yaw_rate) {
    *max_yaw_rate = yaw_rate_max;
  }

  if (!std::isfinite(clearance) || clearance <= validation_min_clearance_) {
    return fail("footprint clearance below validation margin");
  }
  if (!std::isfinite(vmax) ||
      vmax > optimizer_params_.max_vel + validation_velocity_tolerance_) {
    return fail_at("linear velocity limit exceeded", vmax_time);
  }
  if (!std::isfinite(amax) ||
      amax > optimizer_params_.max_acc + validation_acceleration_tolerance_) {
    return fail_at("linear acceleration limit exceeded", amax_time);
  }
  if (!std::isfinite(yaw_rate_max) ||
      yaw_rate_max >
          optimizer_params_.max_yaw_rate + validation_yaw_rate_tolerance_) {
    return fail_at("yaw-rate limit exceeded", yaw_rate_max_time);
  }
  if (!std::isfinite(yaw_acceleration_max) ||
      yaw_acceleration_max > validation_max_yaw_acceleration_ +
                                     validation_yaw_acceleration_tolerance_) {
    return fail_at("yaw-acceleration limit exceeded",
                   yaw_acceleration_max_time);
  }

  double boundary_time = 0.0;
  for (size_t i = 0; i + 1 < traj.pos_pieces.size(); ++i) {
    const PolyPiece& left_pos = traj.pos_pieces[i];
    const PolyPiece& right_pos = traj.pos_pieces[i + 1];
    const YawPolyPiece& left_yaw = traj.yaw_pieces[i];
    const YawPolyPiece& right_yaw = traj.yaw_pieces[i + 1];
    if (!std::isfinite(left_pos.duration) || left_pos.duration <= 0.0 ||
        !std::isfinite(left_yaw.duration) || left_yaw.duration <= 0.0) {
      return fail_at("invalid piece duration", boundary_time);
    }
    boundary_time += left_pos.duration;
    if ((left_pos.evaluate(left_pos.duration) - right_pos.evaluate(0.0)).norm() >
        validation_position_jump_tolerance_) {
      return fail_at("position discontinuity at segment boundary", boundary_time);
    }
    if ((left_pos.velocity(left_pos.duration) - right_pos.velocity(0.0)).norm() >
        validation_velocity_jump_tolerance_) {
      return fail_at("velocity discontinuity at segment boundary", boundary_time);
    }
    if ((left_pos.acceleration(left_pos.duration) -
         right_pos.acceleration(0.0)).norm() >
        validation_acceleration_jump_tolerance_) {
      return fail_at("acceleration discontinuity at segment boundary",
                     boundary_time);
    }
    if (std::abs(normalizeAngle(left_yaw.evaluate(left_yaw.duration) -
                                right_yaw.evaluate(0.0))) >
        validation_yaw_jump_tolerance_) {
      return fail_at("yaw discontinuity at segment boundary", boundary_time);
    }
    if (std::abs(left_yaw.velocity(left_yaw.duration) -
                 right_yaw.velocity(0.0)) >
        validation_yaw_rate_jump_tolerance_) {
      return fail_at("yaw-rate discontinuity at segment boundary", boundary_time);
    }
  }

  if (failure_reason != nullptr) failure_reason->clear();
  return true;
}

bool SvsdfRuntime::PassesExternalEquivalentFootprintGate(
    const Trajectory& traj, double* min_clearance,
    std::string* failure_reason, double* failure_time) const {
  const TrajectoryFootprintValidationResult validation =
      ValidateTrajectoryFootprint(
          grid_map_, footprint_, traj, footprint_validation_config_);
  if (min_clearance != nullptr) {
    *min_clearance = validation.minimum_clearance_m;
  }
  if (failure_time != nullptr) {
    *failure_time = validation.collision_time_sec;
  }
  if (!validation.valid) {
    if (failure_reason != nullptr) {
      *failure_reason = std::string("external-equivalent footprint gate: ") +
                        (validation.collision_type.empty()
                             ? "unknown collision"
                             : validation.collision_type);
    }
    return false;
  }
  if (failure_reason != nullptr) failure_reason->clear();
  return true;
}

bool SvsdfRuntime::PassesExternalEquivalentValidationGate(
    const Trajectory& traj, const SE2State& requested_start,
    const SE2State& requested_goal, double* min_clearance, double* max_vel,
    double* max_acc, std::string* failure_reason,
    double* failure_time) const {
  if (external_validation_gate_) {
    return external_validation_gate_(
        traj, requested_start, requested_goal, min_clearance, max_vel,
        max_acc, failure_reason, failure_time);
  }

  // Standalone ROS fallback. Experiment runs install the exact independent
  // validator callback above; this path preserves the same contract shape for
  // interactive use without introducing a core -> experiments dependency.
  double internal_clearance = -kInf;
  if (!IsTrajectoryFeasible(traj, &internal_clearance, max_vel, max_acc,
                            nullptr, failure_reason, failure_time)) {
    return false;
  }
  double footprint_clearance = -kInf;
  if (!PassesExternalEquivalentFootprintGate(
          traj, &footprint_clearance, failure_reason, failure_time)) {
    return false;
  }
  const SE2State actual_start = traj.sample(0.0);
  const SE2State actual_goal = traj.sample(traj.totalDuration());
  const bool endpoints_match =
      (actual_start.position() - requested_start.position()).norm() <= 1e-3 &&
      std::abs(normalizeAngle(actual_start.yaw - requested_start.yaw)) <= 1e-2 &&
      (actual_goal.position() - requested_goal.position()).norm() <= 1e-3 &&
      std::abs(normalizeAngle(actual_goal.yaw - requested_goal.yaw)) <= 1e-2;
  if (!endpoints_match) {
    if (failure_reason != nullptr) {
      *failure_reason = "external-equivalent gate: requested endpoint mismatch";
    }
    if (failure_time != nullptr) {
      *failure_time = traj.totalDuration();
    }
    return false;
  }
  if (min_clearance != nullptr) {
    *min_clearance = std::min(internal_clearance, footprint_clearance);
  }
  if (failure_reason != nullptr) failure_reason->clear();
  return true;
}

bool SvsdfRuntime::SolveStrictCached(size_t idx, const std::vector<SE2State>& wp,
                                     Trajectory* traj, SegmentCache& cache,
                                     bool require_full_svsdf,
                                     FullSvsdfCallSource source) {
  SegmentCacheKey key(idx, RiskLevelRank(RiskLevel::HIGH));
  auto it = cache.find(key);
  if (it != cache.end()) {
    *traj = it->second;
    return !traj->empty();
  }
  bool ok = SolveStrictSegment(wp, traj, require_full_svsdf, true, source);
  if (ok && !traj->empty()) {
    cache[key] = *traj;
  }
  return ok;
}

bool SvsdfRuntime::OptimizeMediumRiskCached(
    size_t idx, const std::vector<SE2State>& wp, double t, Trajectory* traj,
    SegmentCache& cache) {
  SegmentCacheKey key(idx, RiskLevelRank(RiskLevel::MEDIUM));
  const auto it = cache.find(key);
  if (it != cache.end()) {
    *traj = it->second;
    return !traj->empty();
  }
  const bool ok = OptimizeMediumRiskSegment(wp, t, traj);
  if (ok && !traj->empty()) cache[key] = *traj;
  return ok;
}

bool SvsdfRuntime::OptimizeLowRiskCached(size_t idx, const std::vector<SE2State>& wp,
                                         double t, Trajectory* traj, SegmentCache& cache) {
  SegmentCacheKey key(idx, RiskLevelRank(RiskLevel::LOW));
  auto it = cache.find(key);
  if (it != cache.end()) {
    *traj = it->second;
    return !traj->empty();
  }
  bool ok = OptimizeLowRiskSegment(wp, t, traj);
  if (ok && !traj->empty()) {
    cache[key] = *traj;
  }
  return ok;
}

double SvsdfRuntime::EstimateSegmentTime(
    const std::vector<SE2State>& waypoints) const {
  if (waypoints.size() < 2) {
    return 0.5;
  }
  double minimum_motion_time = 0.0;
  const double max_vel = std::max(0.10, optimizer_params_.max_vel);
  const double max_yaw_rate =
      std::max(0.10, optimizer_params_.max_yaw_rate);
  for (size_t i = 1; i < waypoints.size(); ++i) {
    const double dx = waypoints[i].x - waypoints[i - 1].x;
    const double dy = waypoints[i].y - waypoints[i - 1].y;
    const double translation_time = std::hypot(dx, dy) / max_vel;
    const double rotation_time =
        std::abs(normalizeAngle(waypoints[i].yaw - waypoints[i - 1].yaw)) /
        max_yaw_rate;
    minimum_motion_time += std::max(translation_time, rotation_time);
  }
  // A stop-to-stop quintic peaks above its average speed/rate.  The factor of
  // two keeps both translation and rotation inside the dynamics validator.
  return std::max(0.5, 2.0 * minimum_motion_time);
}

double SvsdfRuntime::PreferredSpeedForRisk(RiskLevel risk) const {
  if (risk == RiskLevel::HIGH) return reference_min_preferred_speed_;
  if (risk == RiskLevel::MEDIUM) {
    return 0.5 * (reference_min_preferred_speed_ +
                  reference_max_preferred_speed_);
  }
  return reference_max_preferred_speed_;
}

std::vector<RuntimeRiskGroundTruthTrial>
SvsdfRuntime::CollectRiskGroundTruth(const CandidateResult& candidate) {
  std::vector<RuntimeRiskGroundTruthTrial> trials;
  if (!experiment_options_.collect_risk_ground_truth ||
      candidate.segments.empty()) {
    return trials;
  }

  // Three-level ground-truth solves are measurement work performed after the
  // planner has selected its output.  Preserve the planning telemetry so these
  // deliberately duplicated calls cannot inflate method-level query counts or
  // alter the reported first-solution timing.
  const RuntimeExperimentTelemetry planning_telemetry = experiment_telemetry_;
  const std::chrono::steady_clock::time_point ground_truth_start =
      std::chrono::steady_clock::now();
  trials.reserve(candidate.segments.size());

  for (size_t index = 0; index < candidate.segments.size(); ++index) {
    const MotionSegment& segment = candidate.segments[index];
    if (segment.waypoints.size() < 2) continue;

    RuntimeRiskGroundTruthTrial trial;
    trial.candidate_index = candidate.candidate_index;
    trial.segment_index = static_cast<int>(index);
    trial.assigned_level = segment.risk;
    trial.maximum_risk_score = segment.max_risk_score;
    trial.mean_risk_score = segment.mean_risk_score;
    trial.state_count = static_cast<int>(segment.waypoints.size());
    trial.segment_length_m = WaypointPathLength(segment.waypoints);
    trial.expected_start = segment.waypoints.front();
    trial.expected_goal = segment.waypoints.back();
    const double segment_time = EstimateSegmentTime(segment.waypoints);

    if (experiment_options_.risk_ground_truth_run_low) {
      trial.low_solver_attempted = true;
      const std::chrono::steady_clock::time_point low_start =
          std::chrono::steady_clock::now();
      trial.low_solver_success = OptimizeLowRiskSegment(
          segment.waypoints, segment_time, &trial.low_trajectory);
      trial.low_solve_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - low_start)
                               .count();
      trial.low_solver_success =
          trial.low_solver_success && !trial.low_trajectory.empty();
      if (!trial.low_trajectory.empty()) {
        NormalizeGroundTruthDynamics(
            optimizer_params_, validation_max_yaw_acceleration_,
            validation_velocity_tolerance_, validation_acceleration_tolerance_,
            validation_yaw_rate_tolerance_,
            validation_yaw_acceleration_tolerance_, &trial.low_trajectory);
      }
    }

    if (experiment_options_.risk_ground_truth_run_medium) {
      trial.medium_solver_attempted = true;
      const std::chrono::steady_clock::time_point medium_start =
          std::chrono::steady_clock::now();
      trial.medium_solver_success = OptimizeMediumRiskSegment(
          segment.waypoints, segment_time, &trial.medium_trajectory, true);
      trial.medium_solve_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() -
                                  medium_start)
                                  .count();
      trial.medium_solver_success =
          trial.medium_solver_success && !trial.medium_trajectory.empty();
      if (!trial.medium_trajectory.empty()) {
        NormalizeGroundTruthDynamics(
            optimizer_params_, validation_max_yaw_acceleration_,
            validation_velocity_tolerance_, validation_acceleration_tolerance_,
            validation_yaw_rate_tolerance_,
            validation_yaw_acceleration_tolerance_,
            &trial.medium_trajectory);
      }
    }

    if (experiment_options_.risk_ground_truth_run_strict) {
      trial.strict_solver_attempted = true;
      const int full_calls_before = experiment_telemetry_.full_svsdf_call_count;
      const int full_success_before =
          experiment_telemetry_.full_svsdf_success_count;
      const double full_time_before = experiment_telemetry_.t_full_svsdf_ms;
      const std::chrono::steady_clock::time_point strict_start =
          std::chrono::steady_clock::now();
      trial.strict_solver_success = SolveStrictSegment(
          segment.waypoints, &trial.strict_trajectory, true, false,
          FullSvsdfCallSource::MEASUREMENT);
      trial.strict_solve_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - strict_start)
                                  .count();
      trial.strict_solver_success =
          trial.strict_solver_success && !trial.strict_trajectory.empty();
      if (!trial.strict_trajectory.empty()) {
        NormalizeGroundTruthDynamics(
            optimizer_params_, validation_max_yaw_acceleration_,
            validation_velocity_tolerance_, validation_acceleration_tolerance_,
            validation_yaw_rate_tolerance_,
            validation_yaw_acceleration_tolerance_,
            &trial.strict_trajectory);
      }
      trial.full_svsdf_used =
          experiment_telemetry_.full_svsdf_call_count > full_calls_before;
      trial.full_svsdf_success =
          experiment_telemetry_.full_svsdf_success_count > full_success_before;
      trial.full_svsdf_ms = std::max(
          0.0, experiment_telemetry_.t_full_svsdf_ms - full_time_before);
    }
    trials.push_back(std::move(trial));
  }

  const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() -
                                ground_truth_start)
                                .count();
  experiment_telemetry_ = planning_telemetry;
  experiment_telemetry_.t_risk_ground_truth_ms = elapsed_ms;
  return trials;
}

}  // namespace isweep_planner
