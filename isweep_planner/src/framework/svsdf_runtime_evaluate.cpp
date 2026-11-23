#include "isweep_planner/framework/svsdf_runtime.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "isweep_planner/core/trajectory_utils.h"
#include "isweep_planner/framework/recovery_manager.h"

namespace isweep_planner {

namespace {

void ScaleTrajectoryTime(double scale, Trajectory* trajectory) {
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

bool IsTemporalValidationFailure(const std::string& reason) {
  return reason.find("velocity") != std::string::npos ||
         reason.find("acceleration") != std::string::npos ||
         reason.find("yaw-rate") != std::string::npos;
}

}  // namespace

SvsdfRuntime::CandidateResult SvsdfRuntime::EvaluateCandidate(
    const std::vector<MotionSegment>& segments,
    const SE2State& requested_start, const SE2State& requested_goal,
    const ros::WallTime& deadline,
    bool frontend_fallback_candidate) {
  CandidateResult result;
  if (segments.empty()) {
    result.validation_failure_reason = "candidate has no motion segments";
    return result;
  }

  result.assessed_segments = segments;
  std::vector<MotionSegment> working_segments = segments;
  // Enforce baselines at the evaluation boundary too, because fallback
  // corridors are generated after normal preprocessing. R2 stays monolithic;
  // all-strict preserves the adaptive segment partition.
  if (experiment_options_.force_all_low) {
    MotionSegment merged;
    merged.risk = RiskLevel::LOW;
    for (const MotionSegment& segment : working_segments) {
      AppendUniqueWaypoints(segment.waypoints, &merged.waypoints);
    }
    working_segments.assign(1, std::move(merged));
  } else if (experiment_options_.force_all_strict) {
    ForceAllSegmentsStrict(&working_segments);
  }
  result.support_points = CountSupportPoints(working_segments);
  result.segments = working_segments;
  std::vector<Trajectory> segment_trajectories(working_segments.size());
  std::vector<bool> segment_upgraded(working_segments.size(), false);
  SegmentCache cache;
  const ros::WallTime candidate_start = ros::WallTime::now();

  const auto mark_budget_exhausted = [&](const char* stage) {
    result.budget_exhausted = true;
    result.validation_failure_reason =
        std::string("deadline exhausted during ") + stage;
    ROS_WARN(
        "Candidate evaluation hit deadline during %s after %.3fs (segments=%zu support=%d)",
        stage, (ros::WallTime::now() - candidate_start).toSec(), segments.size(),
        result.support_points);
  };

  const auto solve_at_level = [&](size_t index, RiskLevel level,
                                  FullSvsdfCallSource strict_source) {
    if (index >= working_segments.size() ||
        working_segments[index].waypoints.size() < 2) {
      return false;
    }
    const double segment_time =
        EstimateSegmentTime(working_segments[index].waypoints);
    Trajectory solved;
    bool ok = false;
    if (level == RiskLevel::LOW) {
      ok = OptimizeLowRiskCached(index, working_segments[index].waypoints,
                                 segment_time, &solved, cache);
    } else if (level == RiskLevel::MEDIUM) {
      ok = OptimizeMediumRiskCached(index, working_segments[index].waypoints,
                                    segment_time, &solved, cache);
    } else {
      // HIGH is the only level that is allowed to call the complete swept-volume
      // backend.  It never takes preserve_shape as a successful substitute.
      ok = SolveStrictCached(index, working_segments[index].waypoints, &solved,
                             cache, true, strict_source);
    }
    if (!ok || solved.empty()) return false;
    working_segments[index].risk = level;
    segment_trajectories[index] = solved;
    ROS_INFO("Segment %zu solved at level %s, risk=%.3f, duration=%.3f", index,
             RiskLevelName(level), working_segments[index].max_risk_score,
             solved.totalDuration());
    return true;
  };

  const ros::WallTime segment_solve_start = ros::WallTime::now();
  for (size_t i = 0; i < working_segments.size(); ++i) {
    if (DeadlineExpired(deadline)) {
      result.segment_solve_time =
          (ros::WallTime::now() - segment_solve_start).toSec();
      mark_budget_exhausted("initial segment solve");
      return result;
    }

    RiskLevel level = working_segments[i].risk;
    bool initial_upgrade_pending = false;
    bool upgraded_this_segment = false;
    FullSvsdfCallSource strict_source =
        frontend_fallback_candidate
            ? FullSvsdfCallSource::FRONTEND_FALLBACK
            : FullSvsdfCallSource::INITIAL_ASSIGNED;
    while (!solve_at_level(i, level, strict_source)) {
      if (DeadlineExpired(deadline)) {
        result.segment_solve_time =
            (ros::WallTime::now() - segment_solve_start).toSec();
        mark_budget_exhausted("segment escalation");
        return result;
      }
      if (!InitialLevelEscalationAllowed(experiment_options_)) {
        result.segment_solve_time =
            (ros::WallTime::now() - segment_solve_start).toSec();
        result.validation_failure_reason =
            std::string("solver failed at forced LOW level on segment ") +
            std::to_string(i);
        return result;
      }
      if (level == RiskLevel::HIGH) {
        result.segment_solve_time =
            (ros::WallTime::now() - segment_solve_start).toSec();
        result.validation_failure_reason =
            std::string("solver failed at HIGH level on segment ") +
            std::to_string(i);
        return result;
      }
      level = UpgradeRiskLevel(level);
      upgraded_this_segment = true;
      ++result.escalated_segments;
      if (!frontend_fallback_candidate) {
        ++experiment_telemetry_.initial_level_upgrade_attempt_count;
        initial_upgrade_pending = true;
        strict_source = FullSvsdfCallSource::INITIAL_ESCALATION;
      }
      ROS_WARN("Segment %zu solver failed; upgrading one level to %s", i,
               RiskLevelName(level));
    }
    if (initial_upgrade_pending) {
      ++experiment_telemetry_.initial_level_upgrade_success_count;
    }
    segment_upgraded[i] = upgraded_this_segment;
  }

  // An escalated segment can end up with the same effective solver level as
  // its neighbor. Solving the two pieces independently inserts a zero-speed
  // endpoint on both sides and forces stitching to negotiate an artificial
  // cusp. In the Ackermann deployment profile, re-solve only these newly
  // equal-level pairs as one segment so the optimizer owns the entire corner.
  if (coalesce_escalated_equal_level_segments_ &&
      working_segments.size() > 1) {
    size_t index = 0;
    int merged_count = 0;
    while (index + 1 < working_segments.size() && !DeadlineExpired(deadline)) {
      if (working_segments[index].risk != working_segments[index + 1].risk ||
          (!segment_upgraded[index] && !segment_upgraded[index + 1])) {
        ++index;
        continue;
      }

      MotionSegment merged = working_segments[index];
      AppendUniqueWaypoints(working_segments[index + 1].waypoints,
                            &merged.waypoints);
      merged.risk_profile.insert(
          merged.risk_profile.end(),
          working_segments[index + 1].risk_profile.begin(),
          working_segments[index + 1].risk_profile.end());
      merged.max_risk_score =
          std::max(working_segments[index].max_risk_score,
                   working_segments[index + 1].max_risk_score);
      const double first_weight = std::max<size_t>(
          1, working_segments[index].risk_profile.size());
      const double second_weight = std::max<size_t>(
          1, working_segments[index + 1].risk_profile.size());
      merged.mean_risk_score =
          (first_weight * working_segments[index].mean_risk_score +
           second_weight * working_segments[index + 1].mean_risk_score) /
          (first_weight + second_weight);

      Trajectory merged_trajectory;
      const double merged_time = EstimateSegmentTime(merged.waypoints);
      bool merged_ok = false;
      if (merged.risk == RiskLevel::LOW) {
        merged_ok = OptimizeLowRiskSegment(merged.waypoints, merged_time,
                                           &merged_trajectory);
      } else if (merged.risk == RiskLevel::MEDIUM) {
        merged_ok = OptimizeMediumRiskSegment(
            merged.waypoints, merged_time, &merged_trajectory);
      } else {
        merged_ok = SolveStrictSegment(
            merged.waypoints, &merged_trajectory, true, true,
            FullSvsdfCallSource::INITIAL_ESCALATION);
      }
      if (!merged_ok || merged_trajectory.empty()) {
        ++index;
        continue;
      }

      working_segments[index] = std::move(merged);
      segment_trajectories[index] = std::move(merged_trajectory);
      segment_upgraded[index] = true;
      working_segments.erase(working_segments.begin() + index + 1);
      segment_trajectories.erase(segment_trajectories.begin() + index + 1);
      segment_upgraded.erase(segment_upgraded.begin() + index + 1);
      ++merged_count;
    }
    if (merged_count > 0) {
      ROS_INFO("Ackermann transition conditioning merged and re-solved %d "
               "adjacent equal-level segment boundaries",
               merged_count);
    }
  }
  result.segment_solve_time =
      (ros::WallTime::now() - segment_solve_start).toSec();

  // Normalize each independently solved segment to the configured dynamics
  // limits before stitching.  Exact time scaling leaves geometry unchanged
  // and avoids slowing an entire long route because one strict corner has a
  // local velocity or acceleration peak.
  int locally_retimed_segments = 0;
  for (Trajectory& segment_trajectory : segment_trajectories) {
    if (segment_trajectory.empty()) continue;
    double segment_clearance = -kInf;
    double segment_max_velocity = 0.0;
    double segment_max_acceleration = 0.0;
    double segment_max_yaw_rate = 0.0;
    std::string segment_reason;
    IsTrajectoryFeasible(segment_trajectory, &segment_clearance,
                         &segment_max_velocity, &segment_max_acceleration,
                         &segment_max_yaw_rate, &segment_reason);

    double segment_max_yaw_acceleration = 0.0;
    const double duration = segment_trajectory.totalDuration();
    const int samples =
        std::max(1, static_cast<int>(std::ceil(duration / 0.02)));
    for (int sample = 0; sample <= samples; ++sample) {
      const double time = duration * static_cast<double>(sample) /
                          static_cast<double>(samples);
      segment_max_yaw_acceleration =
          std::max(segment_max_yaw_acceleration,
                   std::abs(segment_trajectory.sampleYawAcceleration(time)));
    }

    double required_scale = 1.0;
    required_scale = std::max(
        required_scale,
        segment_max_velocity /
            std::max(1e-6, optimizer_params_.max_vel +
                               validation_velocity_tolerance_));
    required_scale = std::max(
        required_scale,
        std::sqrt(segment_max_acceleration /
                  std::max(1e-6, optimizer_params_.max_acc +
                                     validation_acceleration_tolerance_)));
    required_scale = std::max(
        required_scale,
        segment_max_yaw_rate /
            std::max(1e-6, optimizer_params_.max_yaw_rate +
                               validation_yaw_rate_tolerance_));
    required_scale = std::max(
        required_scale,
        std::sqrt(segment_max_yaw_acceleration /
                  std::max(1e-6, validation_max_yaw_acceleration_ +
                                     validation_yaw_acceleration_tolerance_)));
    if (required_scale > 1.0 + 1e-6) {
      ScaleTrajectoryTime(1.02 * required_scale, &segment_trajectory);
      ++locally_retimed_segments;
    }
  }
  if (locally_retimed_segments > 0) {
    ROS_INFO("Pre-stitch dynamics normalization retimed %d/%zu segments",
             locally_retimed_segments, segment_trajectories.size());
  }

  const auto count_levels = [&](const std::vector<MotionSegment>& level_segments) {
    result.high_risk_segments = 0;
    result.medium_risk_segments = 0;
    for (const MotionSegment& segment : level_segments) {
      if (segment.risk == RiskLevel::HIGH) ++result.high_risk_segments;
      if (segment.risk == RiskLevel::MEDIUM) ++result.medium_risk_segments;
    }
  };

  const auto stitch = [&]() {
    const std::chrono::steady_clock::time_point begin =
        std::chrono::steady_clock::now();
    const Trajectory trajectory =
        optimizer_.stitch(working_segments, segment_trajectories);
    experiment_telemetry_.t_stitch_ms +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin)
            .count();
    return trajectory;
  };

  double validation_failure_time =
      std::numeric_limits<double>::quiet_NaN();
  const auto validate = [&](const Trajectory& trajectory, double* clearance,
                            double* max_velocity, double* max_acceleration,
                            std::string* reason) {
    const ros::WallTime begin = ros::WallTime::now();
    bool valid = IsTrajectoryFeasible(
        trajectory, clearance, max_velocity, max_acceleration, nullptr, reason,
        &validation_failure_time);
    if (valid) {
      const SE2State actual_start = trajectory.sample(0.0);
      const SE2State actual_goal = trajectory.sample(trajectory.totalDuration());
      const SE2State& expected_start = working_segments.front().waypoints.front();
      const SE2State& expected_goal = working_segments.back().waypoints.back();
      if ((actual_start.position() - expected_start.position()).norm() > 1e-3 ||
          std::abs(normalizeAngle(actual_start.yaw - expected_start.yaw)) > 1e-2) {
        valid = false;
        validation_failure_time = 0.0;
        if (reason != nullptr) *reason = "trajectory start boundary mismatch";
      } else if ((actual_goal.position() - expected_goal.position()).norm() > 1e-3 ||
                 std::abs(normalizeAngle(actual_goal.yaw - expected_goal.yaw)) >
                     1e-2) {
        valid = false;
        validation_failure_time = trajectory.totalDuration();
        if (reason != nullptr) *reason = "trajectory goal boundary mismatch";
      }
    }
    const double elapsed = (ros::WallTime::now() - begin).toSec();
    result.full_feasibility_time += elapsed;
    experiment_telemetry_.t_internal_validation_ms += elapsed * 1000.0;
    if (!valid) ++experiment_telemetry_.validation_failure_count;
    return valid;
  };

  // Recovery is not counted as successful until it passes the same raw-grid,
  // dense-footprint contract used by the experiment-layer final validator.
  // Initial candidates are checked by consider_incumbent(); recovered
  // candidates are checked here so the recovery funnel cannot over-report
  // successes that are later rejected outside the planner.
  const auto validate_recovery_gate = [&](const Trajectory& trajectory,
                                          double* clearance,
                                          std::string* reason) {
    ++experiment_telemetry_.external_equivalent_gate_attempt_count;
    double footprint_clearance = -kInf;
    double footprint_failure_time =
        std::numeric_limits<double>::quiet_NaN();
    double external_max_velocity = 0.0;
    double external_max_acceleration = 0.0;
    if (!PassesExternalEquivalentValidationGate(
            trajectory, requested_start, requested_goal,
            &footprint_clearance, &external_max_velocity,
            &external_max_acceleration, reason, &footprint_failure_time)) {
      ++experiment_telemetry_.external_equivalent_gate_reject_count;
      validation_failure_time = footprint_failure_time;
      return false;
    }
    if (clearance != nullptr) {
      *clearance = std::min(*clearance, footprint_clearance);
    }
    return true;
  };

  Trajectory current = stitch();
  const Trajectory local_repair_baseline = current;
  const std::vector<Trajectory> local_repair_baseline_segments =
      segment_trajectories;
  std::vector<bool> locally_replanned_segments(segment_trajectories.size(), false);
  bool validation_recovery_started = false;
  const auto begin_validation_recovery = [&]() {
    if (validation_recovery_started) return;
    validation_recovery_started = true;
    ++experiment_telemetry_.validation_recovery_attempt_count;
  };
  const auto complete_validation_recovery = [&]() {
    if (!validation_recovery_started) return;
    ++experiment_telemetry_.validation_recovery_success_count;
  };
  const auto record_local_reuse = [&]() {
    const double original_length =
        ApproximateTrajectoryLength(local_repair_baseline);
    double replanned_length = 0.0;
    double patch_length = 0.0;
    bool any_replanned = false;
    for (size_t i = 0; i < locally_replanned_segments.size(); ++i) {
      if (!locally_replanned_segments[i]) continue;
      any_replanned = true;
      replanned_length +=
          ApproximateTrajectoryLength(local_repair_baseline_segments[i]);
      patch_length += ApproximateTrajectoryLength(segment_trajectories[i]);
    }
    if (!any_replanned || !std::isfinite(original_length) ||
        original_length <= 1e-9) {
      return;
    }
    result.original_trajectory_length_m = original_length;
    result.replanned_length_m = std::min(original_length, replanned_length);
    result.patch_length_m = patch_length;
    result.trajectory_reuse_ratio = LocalTrajectoryReuseRatio(
        original_length, result.replanned_length_m);
  };
  double clearance = -kInf;
  double max_velocity = 0.0;
  double max_acceleration = 0.0;
  std::string validation_reason;
  if (!current.empty() &&
      validate(current, &clearance, &max_velocity, &max_acceleration,
               &validation_reason)) {
    result.trajectory = current;
    result.segment_trajectories = segment_trajectories;
    result.segments = working_segments;
    result.min_clearance = clearance;
    result.max_vel = max_velocity;
    result.max_acc = max_acceleration;
    result.validated = true;
    count_levels(working_segments);
    return result;
  }

  result.validation_failure_reason = validation_reason.empty()
                                         ? "initial stitched validation failed"
                                         : validation_reason;
  ROS_WARN("Initial stitched validation failed: %s (clearance=%.3f)",
           result.validation_failure_reason.c_str(), clearance);

  // Dynamics are a timing problem, not a geometry-risk problem.  Preserve the
  // path and solver levels, and slow only the segment containing the reported
  // failure time with the exact polynomial time transformation.  Scaling the
  // whole trajectory made long building-map paths several minutes long for a
  // single local acceleration spike.
  if (IsTemporalValidationFailure(result.validation_failure_reason)) {
    for (int retime_pass = 0; retime_pass < 6 && !DeadlineExpired(deadline);
         ++retime_pass) {
      if (!std::isfinite(validation_failure_time)) break;
      const size_t culprit =
          SegmentIndexFromTime(segment_trajectories, validation_failure_time);
      if (culprit >= segment_trajectories.size()) break;
      ScaleTrajectoryTime(1.5, &segment_trajectories[culprit]);
      current = stitch();
      validation_reason.clear();
      clearance = -kInf;
      max_velocity = 0.0;
      max_acceleration = 0.0;
      if (!current.empty() &&
          validate(current, &clearance, &max_velocity, &max_acceleration,
                   &validation_reason) &&
          validate_recovery_gate(current, &clearance, &validation_reason)) {
        result.trajectory = current;
        result.segment_trajectories = segment_trajectories;
        result.segments = working_segments;
        result.min_clearance = clearance;
        result.max_vel = max_velocity;
        result.max_acc = max_acceleration;
        result.validated = true;
        result.external_equivalent_gate_passed = true;
        count_levels(working_segments);
        ROS_INFO("Local temporal retiming accepted on pass %d (segment=%zu)",
                 retime_pass + 1, culprit);
        return result;
      }
      result.validation_failure_reason = validation_reason;
      if (!IsTemporalValidationFailure(validation_reason)) break;
    }
  }

  // Validation-driven local escalation.  Each pass upgrades only a guarded
  // bottleneck neighborhood by one level, then restitches and revalidates.
  if (result.validation_failure_reason.find("clearance") != std::string::npos &&
      experiment_options_.recovery_enabled && experiment_options_.local_upgrade) {
    begin_validation_recovery();
    for (int pass = 0; pass < 2 && !DeadlineExpired(deadline); ++pass) {
      const ClearanceProbe bottleneck =
          FindWorstClearanceSample(current, svsdf_evaluator_);
      if (!bottleneck.valid && !std::isfinite(validation_failure_time)) break;
      const double culprit_time = std::isfinite(validation_failure_time)
                                      ? validation_failure_time
                                      : bottleneck.time;
      const size_t culprit =
          SegmentIndexFromTime(segment_trajectories, culprit_time);
      const size_t begin = culprit > 0 ? culprit - 1 : 0;
      const size_t end =
          std::min(working_segments.size() - 1, culprit + static_cast<size_t>(1));

      bool upgraded_any = false;
      const ros::WallTime upgrade_start = ros::WallTime::now();
      for (size_t i = begin; i <= end; ++i) {
        if (working_segments[i].risk == RiskLevel::HIGH) continue;
        const RiskLevel upgraded = UpgradeRiskLevel(working_segments[i].risk);
        ++experiment_telemetry_.local_upgrade_attempt_count;
        if (!solve_at_level(i, upgraded,
                            FullSvsdfCallSource::VALIDATION_RECOVERY)) {
          ROS_WARN("Guarded bottleneck upgrade failed on segment %zu at %s", i,
                   RiskLevelName(upgraded));
          continue;
        }
        ++experiment_telemetry_.local_upgrade_success_count;
        ++result.escalated_segments;
        locally_replanned_segments[i] = true;
        upgraded_any = true;
      }
      result.segment_solve_time +=
          (ros::WallTime::now() - upgrade_start).toSec();
      if (!upgraded_any) break;

      current = stitch();
      validation_reason.clear();
      clearance = -kInf;
      max_velocity = 0.0;
      max_acceleration = 0.0;
      if (!current.empty() &&
          validate(current, &clearance, &max_velocity, &max_acceleration,
                   &validation_reason) &&
          validate_recovery_gate(current, &clearance, &validation_reason)) {
        result.trajectory = current;
        result.segment_trajectories = segment_trajectories;
        result.segments = working_segments;
        result.min_clearance = clearance;
        result.max_vel = max_velocity;
        result.max_acc = max_acceleration;
        result.validated = true;
        result.external_equivalent_gate_passed = true;
        record_local_reuse();
        count_levels(working_segments);
        complete_validation_recovery();
        ROS_INFO(
            "Guarded bottleneck upgrade accepted: pass=%d culprit=%zu window=[%zu,%zu] clearance=%.3f",
            pass + 1, culprit, begin, end, clearance);
        return result;
      }
      result.validation_failure_reason =
          validation_reason.empty() ? "post-upgrade validation failed"
                                    : validation_reason;
    }
  }

  if (DeadlineExpired(deadline)) {
    mark_budget_exhausted("bottleneck recovery");
    return result;
  }

  // Continuous local optimization has been exhausted.  Search patches are
  // still local to the same bottleneck and must pass the identical validator.
  if (result.validation_failure_reason.find("clearance") != std::string::npos &&
      experiment_options_.recovery_enabled &&
      (experiment_options_.strict_local_patch || experiment_options_.swept_astar ||
       experiment_options_.hybrid_astar_recovery)) {
    begin_validation_recovery();
    const ros::WallTime recovery_start = ros::WallTime::now();
    result.segments = working_segments;
    result.segment_trajectories = segment_trajectories;
    result.trajectory = current;
    result.min_clearance = clearance;
    RecoveryManager recovery(this);
    if (recovery.TryRecoverBottleneck(working_segments, segment_trajectories,
                                      current, deadline, &result)) {
      const double elapsed = (ros::WallTime::now() - recovery_start).toSec();
      result.recovery_time += elapsed;
      experiment_telemetry_.t_recovery_ms += elapsed * 1000.0;
      std::string recovered_reason;
      if (validate(result.trajectory, &result.min_clearance, &result.max_vel,
                   &result.max_acc, &recovered_reason) &&
          validate_recovery_gate(result.trajectory, &result.min_clearance,
                                 &recovered_reason)) {
        result.validated = true;
        result.degraded = false;
        result.external_equivalent_gate_passed = true;
        count_levels(result.segments);
        experiment_telemetry_.recovery_success = true;
        complete_validation_recovery();
        return result;
      }
      result.validated = false;
      result.validation_failure_reason =
          recovered_reason.empty() ? "recovered trajectory failed validation"
                                   : recovered_reason;
    } else {
      const double elapsed = (ros::WallTime::now() - recovery_start).toSec();
      result.recovery_time += elapsed;
      experiment_telemetry_.t_recovery_ms += elapsed * 1000.0;
    }
  }

  // Keep no executable trajectory on failure.  This prevents diagnostic or
  // centerline-only paths from entering the incumbent by accident.
  result.trajectory = Trajectory();
  result.validated = false;
  result.degraded = false;
  count_levels(working_segments);
  return result;
}

}  // namespace isweep_planner
