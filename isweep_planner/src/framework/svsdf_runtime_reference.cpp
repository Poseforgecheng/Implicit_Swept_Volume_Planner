#include "isweep_planner/framework/svsdf_runtime.h"
#include "isweep_planner/core/trajectory_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace isweep_planner {

namespace {

bool SameReferencePointPosition(const GlobalReferencePointData& lhs,
                                const GlobalReferencePointData& rhs) {
  return std::abs(lhs.x - rhs.x) < 1e-6 && std::abs(lhs.y - rhs.y) < 1e-6;
}

RiskSample RiskAtState(const MotionSegment& segment, const SE2State& state) {
  if (segment.risk_profile.empty()) {
    RiskSample fallback;
    fallback.state = state;
    fallback.score = segment.max_risk_score;
    fallback.level = segment.risk;
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    fallback.footprint_clearance = unavailable;
    fallback.effective_clearance = unavailable;
    fallback.feasible_yaw_width = unavailable;
    fallback.feasible_yaw_overlap = unavailable;
    fallback.correction_distance = unavailable;
    fallback.uncertainty_erosion = unavailable;
    fallback.clearance_risk = unavailable;
    fallback.orientation_width_risk = unavailable;
    fallback.orientation_continuity_risk = unavailable;
    fallback.correction_risk = unavailable;
    fallback.uncertainty_risk = unavailable;
    return fallback;
  }
  double best_distance = kInf;
  const RiskSample* best_sample = &segment.risk_profile.front();
  for (const RiskSample& sample : segment.risk_profile) {
    const double position_distance =
        (sample.state.position() - state.position()).squaredNorm();
    const double yaw_distance =
        0.01 * std::abs(normalizeAngle(sample.state.yaw - state.yaw));
    const double distance = position_distance + yaw_distance;
    if (distance < best_distance) {
      best_distance = distance;
      best_sample = &sample;
    }
  }
  return *best_sample;
}

void AppendReferenceSample(const SE2State& state, int segment_id,
                           RiskLevel risk, const RiskSample& risk_sample,
                           double clearance,
                           double preferred_speed,
                           RiskAwareGlobalReferenceData* reference) {
  if (reference == nullptr) {
    return;
  }

  GlobalReferencePointData point;
  point.x = state.x;
  point.y = state.y;
  point.yaw = state.yaw;
  point.risk_level = RiskLevelRank(risk);
  point.risk_score = std::max(0.0, std::min(1.0, risk_sample.score));
  point.effective_clearance = risk_sample.effective_clearance;
  point.feasible_yaw_width = risk_sample.feasible_yaw_width;
  point.feasible_yaw_overlap = risk_sample.feasible_yaw_overlap;
  point.correction_distance = risk_sample.correction_distance;
  point.uncertainty_erosion = risk_sample.uncertainty_erosion;
  point.clearance_risk = risk_sample.clearance_risk;
  point.orientation_width_risk = risk_sample.orientation_width_risk;
  point.orientation_continuity_risk =
      risk_sample.orientation_continuity_risk;
  point.correction_risk = risk_sample.correction_risk;
  point.uncertainty_risk = risk_sample.uncertainty_risk;
  point.clearance = clearance;
  point.segment_id = segment_id;
  point.preferred_speed = preferred_speed;

  if (!reference->points.empty()) {
    const GlobalReferencePointData& previous = reference->points.back();
    if (SameReferencePointPosition(previous, point)) {
      point.s = previous.s;
      reference->points.back() = point;
      return;
    }

    const double dx = point.x - previous.x;
    const double dy = point.y - previous.y;
    point.s = previous.s + std::sqrt(dx * dx + dy * dy);
  }

  reference->points.push_back(point);
}

}  // namespace

RiskAwareGlobalReferenceData SvsdfRuntime::BuildRiskAwareGlobalReference(
    const CandidateResult& candidate) const {
  RiskAwareGlobalReferenceData reference;
  reference.stamp = ros::Time::now();
  reference.frame_id =
      latest_map_.header.frame_id.empty() ? "map" : latest_map_.header.frame_id;
  const bool assessed_partition_aligned =
      !candidate.assessed_segments.empty() &&
      candidate.assessed_segments.size() == candidate.segments.size();
  const std::vector<MotionSegment>& reference_segments =
      assessed_partition_aligned ? candidate.assessed_segments
                                 : candidate.segments;
  reference.num_segments = static_cast<int>(reference_segments.size());
  for (const MotionSegment& segment : reference_segments) {
    if (segment.risk == RiskLevel::MEDIUM) {
      ++reference.num_medium_risk_segments;
    } else if (segment.risk == RiskLevel::HIGH) {
      ++reference.num_high_risk_segments;
    }
  }

  if (candidate.trajectory.empty() || candidate.segment_trajectories.empty() ||
      reference_segments.empty()) {
    return reference;
  }

  const size_t segment_count =
      std::min(reference_segments.size(),
               candidate.segment_trajectories.size());
  const double total_duration = candidate.trajectory.totalDuration();
  if (segment_count == 0 || total_duration <= 0.0) {
    return reference;
  }

  if (reference_segments.size() != candidate.segment_trajectories.size()) {
    ROS_WARN(
        "Risk reference metadata mismatch: reference_segments=%zu segment_trajectories=%zu; using %zu aligned entries",
        reference_segments.size(), candidate.segment_trajectories.size(),
        segment_count);
  }

  // Post-stitch smoothing is allowed to refit and globally retime the final
  // trajectory.  Consequently, the sum of the independently solved segment
  // durations is not a reliable clock for sampling the published trajectory.
  // Preserve the risk partition by mapping its geometric arc-length fractions
  // onto the complete final trajectory instead.
  std::vector<double> segment_boundaries(segment_count, 0.0);
  double source_total_length = 0.0;
  double source_total_duration = 0.0;
  for (size_t i = 0; i < segment_count; ++i) {
    const Trajectory& segment_traj = candidate.segment_trajectories[i];
    source_total_duration +=
        segment_traj.empty() ? 0.0 : segment_traj.totalDuration();

    double weight =
        segment_traj.empty() ? kInf : ApproximateTrajectoryLength(segment_traj);
    if (!std::isfinite(weight) || weight <= 1e-9) {
      weight = WaypointPathLength(reference_segments[i].waypoints);
    }
    if (!std::isfinite(weight) || weight <= 1e-9) {
      weight = segment_traj.empty() ? 1.0 : segment_traj.totalDuration();
    }
    if (!std::isfinite(weight) || weight <= 1e-9) {
      weight = 1.0;
    }
    source_total_length += weight;
    segment_boundaries[i] = source_total_length;
  }

  const double duration_scale =
      source_total_duration > 1e-9 ? total_duration / source_total_duration
                                   : kInf;
  if (!std::isfinite(duration_scale) || std::abs(duration_scale - 1.0) > 0.01) {
    ROS_WARN(
        "Risk reference remapping final trajectory after retiming: final_duration=%.3f segment_duration_sum=%.3f scale=%.3f",
        total_duration, source_total_duration, duration_scale);
  }

  const double sample_dt = std::max(0.02, reference_sample_dt_);
  std::vector<SE2State> final_samples;
  final_samples.reserve(
      static_cast<size_t>(std::ceil(total_duration / sample_dt)) + 1);
  for (double t = 0.0; t < total_duration; t += sample_dt) {
    final_samples.push_back(candidate.trajectory.sample(t));
  }
  const SE2State final_state = candidate.trajectory.sample(total_duration);
  if (final_samples.empty() ||
      (final_samples.back().position() - final_state.position()).norm() > 1e-9 ||
      std::abs(normalizeAngle(final_samples.back().yaw - final_state.yaw)) >
          1e-9) {
    final_samples.push_back(final_state);
  } else {
    final_samples.back() = final_state;
  }

  std::vector<double> final_arc_lengths(final_samples.size(), 0.0);
  for (size_t i = 1; i < final_samples.size(); ++i) {
    final_arc_lengths[i] =
        final_arc_lengths[i - 1] +
        (final_samples[i].position() - final_samples[i - 1].position()).norm();
  }
  const double final_total_length = final_arc_lengths.back();

  for (size_t sample_index = 0; sample_index < final_samples.size();
       ++sample_index) {
    const double progress =
        final_total_length > 1e-9
            ? std::max(0.0, std::min(1.0, final_arc_lengths[sample_index] /
                                              final_total_length))
            : static_cast<double>(sample_index) /
                  static_cast<double>(std::max<size_t>(1, final_samples.size() - 1));
    const double source_arc = progress * source_total_length;
    const std::vector<double>::const_iterator boundary = std::upper_bound(
        segment_boundaries.begin(), segment_boundaries.end(), source_arc);
    const size_t segment_index = std::min(
        segment_count - 1,
        static_cast<size_t>(boundary - segment_boundaries.begin()));
    const MotionSegment& segment = reference_segments[segment_index];
    const RiskLevel risk = segment.risk;
    const SE2State& state = final_samples[sample_index];
    AppendReferenceSample(state, static_cast<int>(segment_index), risk,
                          RiskAtState(segment, state),
                          svsdf_evaluator_.evaluate(state),
                          PreferredSpeedForRisk(risk), &reference);
  }

  if (!reference.points.empty()) {
    reference.total_length = reference.points.back().s;
    const GlobalReferencePointData& last = reference.points.back();
    const double endpoint_error =
        (Eigen::Vector2d(last.x, last.y) - final_state.position()).norm();
    const double endpoint_yaw_error =
        std::abs(normalizeAngle(last.yaw - final_state.yaw));
    if (endpoint_error > 1e-6 || endpoint_yaw_error > 1e-5) {
      ROS_ERROR(
          "Risk reference does not attain final trajectory endpoint: position_error=%.9f yaw_error=%.9f",
          endpoint_error, endpoint_yaw_error);
    }
  }
  return reference;
}

}  // namespace isweep_planner
