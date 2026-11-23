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
SvsdfPlanResult SvsdfRuntime::Plan(const Eigen::Vector3d& start,
                                   const Eigen::Vector3d& goal) {
  SvsdfPlanResult result;
  experiment_telemetry_ = RuntimeExperimentTelemetry();
  post_incumbent_budget_active_ = false;
  if (!map_ready_) {
    return result;
  }

  const SE2State start_state = ToSe2State(start);
  const SE2State goal_state = ToSe2State(goal);

  const ros::WallTime total_start = ros::WallTime::now();
  const std::chrono::steady_clock::time_point total_steady_start =
      std::chrono::steady_clock::now();
  const double configured_budget =
      std::max(0.01, experiment_options_.total_time_budget_sec);
  const ros::WallTime planning_deadline =
      total_start + ros::WallDuration(configured_budget);
  const ros::WallTime search_start = ros::WallTime::now();
  const Eigen::Vector2d start_2d(start_state.x, start_state.y);
  const Eigen::Vector2d goal_2d(goal_state.x, goal_state.y);

  const double topo_safe_dist = 0.25; // Ensure connectivity for topological roadmap
  std::vector<TopoPath> topo_paths;
  if (!experiment_options_.hybrid_astar_only) {
    topology_planner_.init(grid_map_, collision_checker_, topology_num_samples_,
                           topology_knn_, topology_max_paths_, topo_safe_dist);
    const std::chrono::steady_clock::time_point roadmap_start =
        std::chrono::steady_clock::now();
    topology_planner_.buildRoadmap(start_2d, goal_2d);
    experiment_telemetry_.t_roadmap_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - roadmap_start).count();
    const std::chrono::steady_clock::time_point dijkstra_start =
        std::chrono::steady_clock::now();
    topo_paths = topology_planner_.searchPaths();
    experiment_telemetry_.t_dijkstra_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - dijkstra_start).count();
  }

  if (!topo_paths.empty()) {
    const size_t raw_topology_count = topo_paths.size();
    const size_t max_eval_paths =
        raw_topology_count > 3 ? 2 : std::min<size_t>(raw_topology_count, 3);
    if (topo_paths.size() > max_eval_paths) {
      topo_paths.resize(max_eval_paths);
    }
    ROS_INFO(
        "Runtime stage 1: Found %zu topological paths, shortening %zu selected candidates (Algorithm 1)",
        raw_topology_count, topo_paths.size());
    const std::chrono::steady_clock::time_point shortcut_start =
        std::chrono::steady_clock::now();
    topology_planner_.shortenPaths(topo_paths);
    experiment_telemetry_.t_shortcut_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - shortcut_start).count();
    ROS_INFO("Runtime stage 1: evaluating %zu shortened paths (from %zu raw)",
             topo_paths.size(), raw_topology_count);
  }

  std::vector<SE2State> hybrid_path;
  std::vector<SE2State> hybrid_diagnostic_partial;
  if (!DeadlineExpired(planning_deadline) &&
      (experiment_options_.hybrid_astar_only ||
       experiment_options_.hybrid_frontend_fallback)) {
    ROS_INFO("Preparing Hybrid A* fallback under the shared planning deadline");
    ++experiment_telemetry_.hybrid_astar_attempt_count;
    if (!experiment_options_.hybrid_astar_only) {
      ++experiment_telemetry_.frontend_fallback_attempt_count;
    }
    const double remaining = RemainingBudgetSec(planning_deadline);
    const double fallback_prepare_budget =
        experiment_options_.hybrid_astar_only
            ? remaining
            : std::min(remaining, topo_paths.empty() ? 2.0 : 1.0);
    if (hybrid_astar_.plan(start_state, goal_state, hybrid_path,
                           fallback_prepare_budget,
                           &hybrid_diagnostic_partial)) {
      ++experiment_telemetry_.hybrid_astar_success_count;
      if (!experiment_options_.hybrid_astar_only) {
        ++experiment_telemetry_.frontend_fallback_success_count;
      }
      result.coarse_path = RosVisualizer::MakePath(hybrid_path);
      result.stats.coarse_path_points = static_cast<int>(hybrid_path.size());
      if (experiment_options_.hybrid_astar_only) {
        TopoPath hybrid_topo;
        hybrid_topo.waypoints.reserve(hybrid_path.size());
        for (const SE2State& state : hybrid_path) {
          hybrid_topo.waypoints.emplace_back(state.position(), state.yaw);
        }
        hybrid_topo.computeLength();
        topo_paths.push_back(hybrid_topo);
      }
    } else if (experiment_options_.hybrid_astar_only &&
               hybrid_diagnostic_partial.size() >= 2) {
      // Publish the nearest explored chain for failed-run diagnostics only.
      // It is deliberately kept out of hybrid_path/topo_paths so it cannot be
      // optimized, validated, or counted as a successful fallback.
      result.coarse_path =
          RosVisualizer::MakePath(hybrid_diagnostic_partial);
      result.stats.coarse_path_points =
          static_cast<int>(hybrid_diagnostic_partial.size());
    }
  }

  if (result.coarse_path.poses.empty() && !topo_paths.empty()) {
    result.coarse_path = RosVisualizer::MakePathFromTopo(topo_paths.front());
    result.stats.coarse_path_points =
        static_cast<int>(topo_paths.front().waypoints.size());
  }
  experiment_telemetry_.candidate_count = static_cast<int>(topo_paths.size());
  result.stats.search_time = (ros::WallTime::now() - search_start).toSec();

  const ros::WallTime optimize_start = ros::WallTime::now();
  const double optimize_timeout_sec = RemainingBudgetSec(planning_deadline);
  const ros::WallTime optimize_deadline = planning_deadline;
  double total_segment_solve_time = 0.0;
  double total_full_feasibility_time = 0.0;
  double total_tail_refine_time = 0.0;
  double total_recovery_time = 0.0;
  std::vector<CandidateResult> candidate_results;
  candidate_results.reserve(topo_paths.size());
  CandidateResult incumbent;
  bool has_incumbent = false;

  const auto candidate_better = [&](const CandidateResult& candidate,
                                    const CandidateResult& current) {
    const double clearance_band = std::max(
        0.005, 0.10 * std::max(0.05, std::max(candidate.min_clearance,
                                               current.min_clearance)));
    if (candidate.min_clearance > current.min_clearance + clearance_band) return true;
    if (current.min_clearance > candidate.min_clearance + clearance_band) return false;
    const double candidate_smoothness =
        TrajectorySmoothnessCost(candidate.trajectory);
    const double current_smoothness = TrajectorySmoothnessCost(current.trajectory);
    if (candidate_smoothness + 1e-6 < current_smoothness) return true;
    if (current_smoothness + 1e-6 < candidate_smoothness) return false;
    if (candidate.trajectory.pos_pieces.size() !=
        current.trajectory.pos_pieces.size()) {
      return candidate.trajectory.pos_pieces.size() <
             current.trajectory.pos_pieces.size();
    }
    if (std::abs(candidate.trajectory.totalDuration() -
                 current.trajectory.totalDuration()) > 1e-6) {
      return candidate.trajectory.totalDuration() <
             current.trajectory.totalDuration();
    }
    return candidate.min_clearance > current.min_clearance;
  };

  const auto consider_incumbent = [&](CandidateResult candidate) {
    if (!candidate.validated || candidate.trajectory.empty() || candidate.degraded) {
      ++experiment_telemetry_.incumbent_reject_count;
      return false;
    }
    if (!candidate.external_equivalent_gate_passed) {
      ++experiment_telemetry_.external_equivalent_gate_attempt_count;
      double footprint_clearance = -kInf;
      double footprint_failure_time =
          std::numeric_limits<double>::quiet_NaN();
      std::string footprint_failure;
      double external_max_velocity = 0.0;
      double external_max_acceleration = 0.0;
      if (!PassesExternalEquivalentValidationGate(
              candidate.trajectory, start_state, goal_state,
              &footprint_clearance, &external_max_velocity,
              &external_max_acceleration, &footprint_failure,
              &footprint_failure_time)) {
        ++experiment_telemetry_.external_equivalent_gate_reject_count;
        ++experiment_telemetry_.incumbent_reject_count;
        ROS_WARN(
            "Candidate %d rejected before incumbent replacement: %s at t=%.3f",
            candidate.candidate_index, footprint_failure.c_str(),
            footprint_failure_time);
        return false;
      }
      candidate.external_equivalent_gate_passed = true;
      candidate.min_clearance =
          std::min(candidate.min_clearance, footprint_clearance);
      candidate.max_vel = external_max_velocity;
      candidate.max_acc = external_max_acceleration;
    }
    if (!has_incumbent || candidate_better(candidate, incumbent)) {
      const bool first_incumbent = !has_incumbent;
      incumbent = candidate;
      has_incumbent = true;
      if (first_incumbent) {
        post_incumbent_budget_active_ = true;
      }
      ++experiment_telemetry_.incumbent_update_count;
      RuntimeIncumbentEvent event;
      event.update_index = experiment_telemetry_.incumbent_update_count - 1;
      event.elapsed_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() -
                             total_steady_start)
                             .count();
      event.candidate_index = candidate.candidate_index;
      event.minimum_clearance_m = candidate.min_clearance;
      event.smoothness_cost = TrajectorySmoothnessCost(candidate.trajectory);
      event.trajectory_length_m =
          ApproximateTrajectoryLength(candidate.trajectory);
      event.trajectory_duration_sec = candidate.trajectory.totalDuration();
      event.piece_count =
          static_cast<int>(candidate.trajectory.pos_pieces.size());
      for (const MotionSegment& segment : candidate.segments) {
        if (segment.risk == RiskLevel::HIGH) {
          ++event.high_segment_count;
        } else if (segment.risk == RiskLevel::MEDIUM) {
          ++event.medium_segment_count;
        } else {
          ++event.low_segment_count;
        }
      }
      event.recovery_success = candidate.recovery_time > 0.0 &&
                               candidate.validated;
      experiment_telemetry_.incumbent_events.push_back(event);
      if (!std::isfinite(experiment_telemetry_.t_first_feasible_ms)) {
        experiment_telemetry_.t_first_feasible_ms = event.elapsed_ms;
      }
      ROS_INFO(
          "Validated incumbent updated: candidate=%d clearance=%.3f smoothness=%.3f pieces=%zu duration=%.3f",
          candidate.candidate_index, candidate.min_clearance,
          TrajectorySmoothnessCost(candidate.trajectory),
          candidate.trajectory.pos_pieces.size(),
          candidate.trajectory.totalDuration());
      return true;
    }
    ++experiment_telemetry_.incumbent_reject_count;
    return false;
  };

  // Stage A: parallel SE2 sequence generation + waypoint push
  struct PreprocessedCandidate {
    std::vector<MotionSegment> segments;
    SE2GenerationDiagnostics generation_diagnostics;
    double push_ms = 0.0;
    bool valid = false;
  };
  std::vector<PreprocessedCandidate> preprocessed(topo_paths.size());
  const ros::WallTime preprocess_start = ros::WallTime::now();

  #pragma omp parallel for schedule(dynamic)
  for (size_t i = 0; i < topo_paths.size(); ++i) {
    std::vector<MotionSegment> raw_segments =
        se2_generator_.generate(topo_paths[i], start_state, goal_state,
                                &preprocessed[i].generation_diagnostics);
    if (raw_segments.empty()) {
      continue;
    }
    // The generator has already performed footprint-aware ESDF correction and
    // measured that displacement as part of the risk score.  Mutating the
    // states again here would stale the risk profile and could move q_s/q_g.
    preprocessed[i].push_ms = 0.0;
    if (experiment_options_.hybrid_astar_only) {
      MotionSegment hybrid_segment;
      hybrid_segment.risk = RiskLevel::LOW;
      hybrid_segment.waypoints = hybrid_path;
      raw_segments.assign(1, hybrid_segment);
    }
    // Baseline solver forcing is applied inside EvaluateCandidate(). Keeping
    // these segments untouched preserves the assessed risk labels for the
    // published reference and the local controller.
    preprocessed[i].segments = std::move(raw_segments);
    preprocessed[i].valid = true;
  }
  const double preprocess_time =
      (ros::WallTime::now() - preprocess_start).toSec();
  result.stats.preprocess_time = preprocess_time;
  experiment_telemetry_.t_path_adjust_ms = 0.0;
  experiment_telemetry_.t_risk_classification_ms = 0.0;
  size_t preprocessed_valid = 0;
  for (const PreprocessedCandidate& candidate : preprocessed) {
    experiment_telemetry_.t_path_adjust_ms +=
        candidate.generation_diagnostics.path_adjust_ms + candidate.push_ms;
    experiment_telemetry_.t_risk_classification_ms +=
        candidate.generation_diagnostics.risk_classification_ms;
    if (candidate.valid) {
      ++preprocessed_valid;
    }
  }
  experiment_telemetry_.t_se2_sequence_ms = std::max(
      0.0, preprocess_time * 1000.0 - experiment_telemetry_.t_path_adjust_ms -
               experiment_telemetry_.t_risk_classification_ms);
  ROS_INFO(
      "Plan() stage 2: preprocessed %zu/%zu candidates into motion segments in %.3fs",
      preprocessed_valid, preprocessed.size(), preprocess_time);

  // Stage B: serial optimization + early pruning
  for (size_t i = 0; i < preprocessed.size(); ++i) {
    if (!preprocessed[i].valid) {
      ROS_WARN("Path %zu discarded: no motion segments", i);
      continue;
    }
    const double elapsed = (ros::WallTime::now() - optimize_start).toSec();
    if (DeadlineExpired(optimize_deadline)) {
      ROS_WARN("Plan() optimization timeout after %.1fs, evaluated %zu/%zu paths",
               elapsed, i, topo_paths.size());
      break;
    }
    // Early pruning: skip candidates with many more segments than best so far
    if (has_incumbent &&
        preprocessed[i].segments.size() >
            2 * incumbent.trajectory.pos_pieces.size()) {
      ROS_INFO("Path %zu pruned: too many segments (%zu)", i, preprocessed[i].segments.size());
      continue;
    }

    CandidateResult candidate = EvaluateCandidate(
        preprocessed[i].segments, start_state, goal_state, optimize_deadline);
    candidate.candidate_index = static_cast<int>(i);
    total_segment_solve_time += candidate.segment_solve_time;
    total_full_feasibility_time += candidate.full_feasibility_time;
    total_tail_refine_time += candidate.tail_refine_time;
    total_recovery_time += candidate.recovery_time;
    if (candidate.budget_exhausted) {
      ROS_WARN(
          "Path %zu reached the optimization deadline during evaluation: stages seg=%.3fs full=%.3fs tail=%.3fs recovery=%.3fs tail_attempts=%d pruned=%d",
          i, candidate.segment_solve_time, candidate.full_feasibility_time,
          candidate.tail_refine_time, candidate.recovery_time,
          candidate.tail_attempts, candidate.tail_pruned);
    }
    if (candidate.trajectory.empty() || !candidate.validated) {
      if (candidate.budget_exhausted && DeadlineExpired(optimize_deadline)) {
        ROS_WARN(
            "Path %zu stopped by the optimization deadline before producing a usable trajectory",
            i);
        break;
      }
      ROS_WARN("Path %zu discarded: no independently validated trajectory (%s)", i,
               candidate.validation_failure_reason.empty()
                   ? "unspecified validation failure"
                   : candidate.validation_failure_reason.c_str());
      // candidate_switch=false is a hard single-candidate contract.  This is
      // especially important for full_r2: a failed forced-LOW solve must end
      // the run instead of silently trying a second topological candidate.
      if (!experiment_options_.candidate_switch) {
        break;
      }
      continue;
    }

    if (candidate.degraded) {
      ROS_WARN(
          "Path %zu degraded: clearance=%.3f (diagnostic only; never enters incumbent), stages seg=%.3fs full=%.3fs tail=%.3fs recovery=%.3fs tail_attempts=%d pruned=%d",
          i, candidate.min_clearance, candidate.segment_solve_time,
          candidate.full_feasibility_time, candidate.tail_refine_time,
          candidate.recovery_time, candidate.tail_attempts, candidate.tail_pruned);
      ++experiment_telemetry_.incumbent_reject_count;
      if (!experiment_options_.candidate_switch) {
        break;
      }
      if (DeadlineExpired(optimize_deadline)) {
        break;
      }
      continue;
    }

    ROS_INFO(
        "Path %zu accepted: min_svsdf=%.3f, vmax=%.3f, amax=%.3f, escalated=%d, stages seg=%.3fs full=%.3fs tail=%.3fs recovery=%.3fs tail_attempts=%d pruned=%d",
        i, candidate.min_clearance, candidate.max_vel, candidate.max_acc,
        candidate.escalated_segments, candidate.segment_solve_time,
        candidate.full_feasibility_time, candidate.tail_refine_time,
        candidate.recovery_time, candidate.tail_attempts, candidate.tail_pruned);
    candidate_results.push_back(candidate);
    consider_incumbent(candidate);
    if (!experiment_options_.candidate_switch) {
      break;
    }
    if (DeadlineExpired(optimize_deadline)) {
      break;
    }
  }

  if (!has_incumbent && hybrid_path.empty() &&
      experiment_options_.hybrid_frontend_fallback &&
      !DeadlineExpired(optimize_deadline)) {
    ROS_WARN("Prepared Hybrid A* seed was unavailable; using remaining budget for one fallback search");
    ++experiment_telemetry_.hybrid_astar_attempt_count;
    ++experiment_telemetry_.frontend_fallback_attempt_count;
    if (hybrid_astar_.plan(start_state, goal_state, hybrid_path,
                           RemainingBudgetSec(optimize_deadline))) {
      ++experiment_telemetry_.hybrid_astar_success_count;
      ++experiment_telemetry_.frontend_fallback_success_count;
    }
  }

  if (!experiment_options_.hybrid_astar_only && !has_incumbent &&
      !DeadlineExpired(optimize_deadline) && hybrid_path.size() >= 2) {
    ROS_WARN("All topological candidates failed, retrying with Hybrid A* corridor.");
    TopoPath hybrid_topo;
    hybrid_topo.waypoints.reserve(hybrid_path.size());
    for (size_t i = 0; i < hybrid_path.size(); ++i) {
      hybrid_topo.waypoints.push_back(
          TopoWaypoint(Eigen::Vector2d(hybrid_path[i].x, hybrid_path[i].y),
                       hybrid_path[i].yaw));
    }
    hybrid_topo.computeLength();

    const std::vector<MotionSegment> segments =
        se2_generator_.generate(hybrid_topo, start_state, goal_state);
    if (!segments.empty()) {
      CandidateResult candidate = EvaluateCandidate(
          segments, start_state, goal_state, optimize_deadline, true);
      candidate.candidate_index = static_cast<int>(topo_paths.size());
      total_segment_solve_time += candidate.segment_solve_time;
      total_full_feasibility_time += candidate.full_feasibility_time;
      total_tail_refine_time += candidate.tail_refine_time;
      total_recovery_time += candidate.recovery_time;
      if (!candidate.trajectory.empty() && candidate.validated) {
        ROS_INFO("Hybrid fallback accepted: min_svsdf=%.3f, vmax=%.3f, amax=%.3f",
                 candidate.min_clearance, candidate.max_vel, candidate.max_acc);
        candidate_results.push_back(candidate);
        consider_incumbent(candidate);
      }
    }
  }

  if (!experiment_options_.hybrid_astar_only && !has_incumbent &&
      !DeadlineExpired(optimize_deadline) && hybrid_path.size() >= 2) {
    const double total_time = 1.5 * EstimateSegmentTime(hybrid_path);
    const Trajectory hybrid_traj = optimizer_.optimizeSE2(hybrid_path, total_time);
    double min_clearance = -kInf;
    double max_vel = 0.0;
    double max_acc = 0.0;
    const SE2State hybrid_start = hybrid_traj.empty()
                                      ? SE2State()
                                      : hybrid_traj.sample(0.0);
    const SE2State hybrid_goal =
        hybrid_traj.empty()
            ? SE2State()
            : hybrid_traj.sample(hybrid_traj.totalDuration());
    const bool endpoints_match =
        !hybrid_traj.empty() &&
        (hybrid_start.position() - start_state.position()).norm() <= 1e-3 &&
        std::abs(normalizeAngle(hybrid_start.yaw - start_state.yaw)) <= 1e-2 &&
        (hybrid_goal.position() - goal_state.position()).norm() <= 1e-3 &&
        std::abs(normalizeAngle(hybrid_goal.yaw - goal_state.yaw)) <= 1e-2;
    if (endpoints_match &&
        IsTrajectoryFeasible(hybrid_traj, &min_clearance, &max_vel, &max_acc)) {
      ROS_INFO("Monolithic Hybrid fallback accepted: min_svsdf=%.3f, vmax=%.3f, amax=%.3f",
               min_clearance, max_vel, max_acc);
      CandidateResult candidate;
      candidate.trajectory = hybrid_traj;
      candidate.min_clearance = min_clearance;
      candidate.max_vel = max_vel;
      candidate.max_acc = max_acc;
      candidate.support_points = static_cast<int>(hybrid_path.size());
      candidate.validated = true;
      candidate.candidate_index = static_cast<int>(topo_paths.size());
      candidate_results.push_back(candidate);
      consider_incumbent(candidate);
    }
  }

  result.stats.optimization_time = (ros::WallTime::now() - optimize_start).toSec();
  result.stats.segment_solve_time = total_segment_solve_time;
  result.stats.full_feasibility_time = total_full_feasibility_time;
  result.stats.tail_refine_time = total_tail_refine_time;
  result.stats.recovery_time = total_recovery_time;

  const auto log_final_stage_totals = [&](const char* outcome) {
    ROS_INFO(
        "Plan() final stage totals [%s]: topology=%.3fs preprocess=%.3fs segment_solve=%.3fs full_feasibility=%.3fs tail_refinement=%.3fs recovery=%.3fs optimize_total=%.3fs total=%.3fs",
        outcome, result.stats.search_time, result.stats.preprocess_time,
        result.stats.segment_solve_time, result.stats.full_feasibility_time,
        result.stats.tail_refine_time, result.stats.recovery_time,
        result.stats.optimization_time, result.stats.total_solve_time);
  };

  if (!has_incumbent) {
    if (hybrid_path.size() >= 2) {
      ROS_WARN(
          "Discrete Hybrid A* fallback exists but did not produce a validated continuous trajectory; returning explicit failure");
    }
    result.stats.optimization_time = (ros::WallTime::now() - optimize_start).toSec();
    result.stats.total_solve_time = (ros::WallTime::now() - total_start).toSec();
    result.timeout = DeadlineExpired(optimize_deadline);
    experiment_telemetry_.timeout = result.timeout;
    log_final_stage_totals("failed");
    return result;
  }

  const ros::WallTime selection_start = ros::WallTime::now();
  CandidateResult final_candidate = incumbent;
  Trajectory best = final_candidate.trajectory;
  const double best_smoothness = TrajectorySmoothnessCost(best);
  const double selection_time = (ros::WallTime::now() - selection_start).toSec();
  ROS_INFO(
      "Plan() incumbent selection: validated=%zu selected_candidate=%d clearance=%.3f smooth=%.3f pieces=%zu duration=%.3f selection=%.3fs",
      candidate_results.size(), final_candidate.candidate_index,
      final_candidate.min_clearance,
      best_smoothness, best.pos_pieces.size(), best.totalDuration(), selection_time);
  if (experiment_options_.recovery_enabled &&
      experiment_options_.tail_refinement &&
      !final_candidate.segments.empty() && !final_candidate.segment_trajectories.empty()) {
    const ros::WallTime tail_start = ros::WallTime::now();
    const CandidateResult before_tail = final_candidate;
    final_candidate.trajectory = best;
    RecoveryManager recovery(this);
    recovery.MaybeImproveTailTrajectory(final_candidate.segments, final_candidate.segment_trajectories,
                                        optimize_deadline, &final_candidate);
    const double tail_elapsed = (ros::WallTime::now() - tail_start).toSec();
    final_candidate.tail_refine_time += tail_elapsed;
    total_tail_refine_time += tail_elapsed;
    std::string tail_validation_failure;
    bool tail_feasible =
        final_candidate.validated &&
        IsTrajectoryFeasible(final_candidate.trajectory,
                             &final_candidate.min_clearance,
                             &final_candidate.max_vel,
                             &final_candidate.max_acc, nullptr,
                             &tail_validation_failure);
    if (tail_feasible && !final_candidate.external_equivalent_gate_passed) {
      ++experiment_telemetry_.external_equivalent_gate_attempt_count;
      double footprint_clearance = -kInf;
      double footprint_failure_time =
          std::numeric_limits<double>::quiet_NaN();
      double external_max_velocity = 0.0;
      double external_max_acceleration = 0.0;
      tail_feasible = PassesExternalEquivalentValidationGate(
          final_candidate.trajectory, start_state, goal_state,
          &footprint_clearance, &external_max_velocity,
          &external_max_acceleration, &tail_validation_failure,
          &footprint_failure_time);
      if (!tail_feasible) {
        ++experiment_telemetry_.external_equivalent_gate_reject_count;
      } else {
        final_candidate.external_equivalent_gate_passed = true;
        final_candidate.min_clearance =
            std::min(final_candidate.min_clearance, footprint_clearance);
        final_candidate.max_vel = external_max_velocity;
        final_candidate.max_acc = external_max_acceleration;
      }
    }
    const bool tail_changed =
        final_candidate.trajectory.pos_pieces.size() !=
            before_tail.trajectory.pos_pieces.size() ||
        std::abs(ApproximateTrajectoryLength(final_candidate.trajectory) -
                 ApproximateTrajectoryLength(before_tail.trajectory)) > 1e-6 ||
        std::abs(TrajectorySmoothnessCost(final_candidate.trajectory) -
                 TrajectorySmoothnessCost(before_tail.trajectory)) > 1e-6 ||
        std::abs(final_candidate.min_clearance - before_tail.min_clearance) >
            1e-6;
    if (tail_feasible) {
      best = final_candidate.trajectory;
      if (tail_changed) {
        ++experiment_telemetry_.incumbent_update_count;
        RuntimeIncumbentEvent event;
        event.update_index = experiment_telemetry_.incumbent_update_count - 1;
        event.elapsed_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() -
                               total_steady_start)
                               .count();
        event.candidate_index = final_candidate.candidate_index;
        event.minimum_clearance_m = final_candidate.min_clearance;
        event.smoothness_cost = TrajectorySmoothnessCost(best);
        event.trajectory_length_m = ApproximateTrajectoryLength(best);
        event.trajectory_duration_sec = best.totalDuration();
        event.piece_count = static_cast<int>(best.pos_pieces.size());
        for (const MotionSegment& segment : final_candidate.segments) {
          if (segment.risk == RiskLevel::HIGH) {
            ++event.high_segment_count;
          } else if (segment.risk == RiskLevel::MEDIUM) {
            ++event.medium_segment_count;
          } else {
            ++event.low_segment_count;
          }
        }
        event.recovery_success = false;
        experiment_telemetry_.incumbent_events.push_back(event);
      }
    } else {
      ROS_WARN("Tail refinement rejected by final validation: %s",
               tail_validation_failure.c_str());
      final_candidate = before_tail;
      best = final_candidate.trajectory;
    }
  }

  // The interactive demo always published the optimizer's fully stitched
  // trajectory.  In current_full, however, a locally recovered incumbent or
  // an accepted tail replacement can be assembled from an untouched prefix
  // and a new patch after the ordinary stitch pass.  Re-run the same global
  // boundary conditioning and collision-aware post-stitch smoothing before
  // publication so LOW/MEDIUM/HIGH transitions cannot bypass the Ackermann
  // smoothness profile.  Keep the already validated incumbent if the
  // synchronized result fails the independent gate.
  if (experiment_options_.method_name == "current_full" &&
      !final_candidate.segments.empty() &&
      final_candidate.segments.size() ==
          final_candidate.segment_trajectories.size()) {
    const std::chrono::steady_clock::time_point restitch_begin =
        std::chrono::steady_clock::now();
    const Trajectory synchronized = optimizer_.stitch(
        final_candidate.segments, final_candidate.segment_trajectories);
    experiment_telemetry_.t_stitch_ms +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - restitch_begin)
            .count();

    if (!synchronized.empty()) {
      std::string synchronization_failure;
      double synchronized_clearance = -kInf;
      double synchronized_max_velocity = 0.0;
      double synchronized_max_acceleration = 0.0;
      double synchronization_failure_time =
          std::numeric_limits<double>::quiet_NaN();
      ++experiment_telemetry_.external_equivalent_gate_attempt_count;
      if (PassesExternalEquivalentValidationGate(
              synchronized, start_state, goal_state,
              &synchronized_clearance, &synchronized_max_velocity,
              &synchronized_max_acceleration, &synchronization_failure,
              &synchronization_failure_time)) {
        const double before_smoothness = TrajectorySmoothnessCost(best);
        const size_t before_piece_count = best.pos_pieces.size();
        const double after_smoothness =
            TrajectorySmoothnessCost(synchronized);
        best = synchronized;
        final_candidate.trajectory = synchronized;
        final_candidate.min_clearance = synchronized_clearance;
        final_candidate.max_vel = synchronized_max_velocity;
        final_candidate.max_acc = synchronized_max_acceleration;
        final_candidate.external_equivalent_gate_passed = true;
        ROS_INFO(
            "current_full final stitch synchronization accepted: smoothness %.6f -> %.6f pieces %zu -> %zu",
            before_smoothness, after_smoothness,
            before_piece_count, best.pos_pieces.size());
      } else {
        ++experiment_telemetry_.external_equivalent_gate_reject_count;
        ROS_WARN(
            "current_full final stitch synchronization rejected; retaining validated incumbent: %s",
            synchronization_failure.c_str());
      }
    }
  }

  // Publication gate: repeat the independent contract after all optional
  // refinements.  No centerline, partial search path, or stale segment result
  // may cross this point.
  std::string publication_failure;
  double publication_clearance = -kInf;
  double publication_max_vel = 0.0;
  double publication_max_acc = 0.0;
  ++experiment_telemetry_.external_equivalent_gate_attempt_count;
  double publication_failure_time =
      std::numeric_limits<double>::quiet_NaN();
  const bool publication_valid = PassesExternalEquivalentValidationGate(
      best, start_state, goal_state, &publication_clearance,
      &publication_max_vel, &publication_max_acc, &publication_failure,
      &publication_failure_time);
  if (!publication_valid) {
    ++experiment_telemetry_.external_equivalent_gate_reject_count;
    ROS_ERROR("Validated incumbent rejected at publication gate: %s",
              publication_failure.c_str());
    result.stats.total_solve_time = (ros::WallTime::now() - total_start).toSec();
    result.timeout = DeadlineExpired(optimize_deadline);
    experiment_telemetry_.timeout = result.timeout;
    ++experiment_telemetry_.validation_failure_count;
    log_final_stage_totals("publication_failed");
    return result;
  }
  final_candidate.min_clearance = publication_clearance;
  final_candidate.max_vel = publication_max_vel;
  final_candidate.max_acc = publication_max_acc;

  result.stats.optimization_time = (ros::WallTime::now() - optimize_start).toSec();
  result.success = true;
  result.validated = true;
  result.degraded = false;
  result.timeout = DeadlineExpired(optimize_deadline);
  result.selected_candidate_index = final_candidate.candidate_index;
  experiment_telemetry_.selected_candidate_index = final_candidate.candidate_index;
  experiment_telemetry_.degraded = false;
  experiment_telemetry_.timeout = result.timeout;
  experiment_telemetry_.candidate_switch_count =
      final_candidate.candidate_index > 0 ? final_candidate.candidate_index : 0;

  ROS_INFO("Plan(): best traj pieces=%zu duration=%.3f, sampling first piece c0=(%.4f,%.4f) c1=(%.4f,%.4f)",
           best.pos_pieces.size(), best.totalDuration(),
           best.pos_pieces.empty() ? 0.0 : best.pos_pieces[0].coeffs(0,0),
           best.pos_pieces.empty() ? 0.0 : best.pos_pieces[0].coeffs(1,0),
           best.pos_pieces.empty() ? 0.0 : best.pos_pieces[0].coeffs(0,1),
           best.pos_pieces.empty() ? 0.0 : best.pos_pieces[0].coeffs(1,1));
  result.trajectory = RosVisualizer::MakeTrajectoryPath(best);
  result.raw_trajectory = best;
  result.risk_aware_global_reference = BuildRiskAwareGlobalReference(final_candidate);
  experiment_telemetry_.original_trajectory_length_m =
      final_candidate.original_trajectory_length_m;
  experiment_telemetry_.replanned_length_m =
      final_candidate.replanned_length_m;
  experiment_telemetry_.patch_length_m = final_candidate.patch_length_m;
  experiment_telemetry_.trajectory_reuse_ratio =
      final_candidate.trajectory_reuse_ratio;
  result.risk_ground_truth_trials = CollectRiskGroundTruth(final_candidate);
  ROS_INFO("Plan(): published trajectory poses=%zu", result.trajectory.poses.size());
  ROS_INFO("Plan(): built risk-aware reference points=%zu segments=%d medium_risk=%d high_risk=%d length=%.3f",
           result.risk_aware_global_reference.points.size(),
           result.risk_aware_global_reference.num_segments,
           result.risk_aware_global_reference.num_medium_risk_segments,
           result.risk_aware_global_reference.num_high_risk_segments,
           result.risk_aware_global_reference.total_length);
  result.stats.min_clearance = final_candidate.min_clearance;
  result.stats.support_points = final_candidate.support_points;
  result.stats.local_obstacle_points = final_candidate.high_risk_segments;
  result.stats.optimizer_iterations = 0;
  result.stats.segment_solve_time = total_segment_solve_time;
  result.stats.full_feasibility_time = total_full_feasibility_time;
  result.stats.tail_refine_time = total_tail_refine_time;
  result.stats.recovery_time = total_recovery_time;
  result.stats.total_solve_time = (ros::WallTime::now() - total_start).toSec();
  const double optimization_other_time = std::max(
      0.0, result.stats.optimization_time - preprocess_time -
               total_segment_solve_time - total_full_feasibility_time -
               total_tail_refine_time - total_recovery_time - selection_time);
  ROS_INFO(
      "Plan() timing breakdown: topology=%.3fs preprocess=%.3fs segment_solve=%.3fs full_feasibility=%.3fs tail_refinement=%.3fs recovery=%.3fs selection=%.3fs optimize_other=%.3fs optimize_total=%.3fs",
      result.stats.search_time, preprocess_time, total_segment_solve_time,
      total_full_feasibility_time, total_tail_refine_time,
      total_recovery_time, selection_time, optimization_other_time,
      result.stats.optimization_time);
  log_final_stage_totals("success");
  return result;
}
} // namespace isweep_planner
