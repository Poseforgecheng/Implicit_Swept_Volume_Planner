#include "isweep_planner/optimization/midend/se2_sequence_generator.h"
#include "isweep_planner/core/math_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>

namespace isweep_planner {

namespace {



int WrappedBinDistance(int lhs, int rhs, int bins) {
  const int raw = std::abs(lhs - rhs);
  return std::min(raw, bins - raw);
}

bool IsHighRiskConfiguration(const GridMap& map, const CollisionChecker& checker,
                             const SE2State& desired, const SE2State& assigned) {
  const double esdf = map.getEsdf(desired.x, desired.y);
  const int safe_count =
      static_cast<int>(checker.safeYawCount(desired.x, desired.y));
  const int bins = checker.numYawBins();
  const int desired_bin = checker.binFromYaw(desired.yaw);
  const int assigned_bin = checker.binFromYaw(assigned.yaw);
  const int yaw_distance = WrappedBinDistance(desired_bin, assigned_bin, bins);

  const double critical_clearance =
      std::max(2.0 * checker.footprint().inscribedRadius(),
               1.35 * checker.footprint().circumscribedRadius());
  const double caution_clearance =
      std::max(2.5 * checker.footprint().inscribedRadius(),
               1.65 * checker.footprint().circumscribedRadius());
  const int constrained_yaw_threshold =
      std::max(2, static_cast<int>(std::ceil(0.30 * static_cast<double>(bins))));
  const int limited_yaw_threshold =
      std::max(3, static_cast<int>(std::ceil(0.45 * static_cast<double>(bins))));

  const bool severe_clearance = !std::isfinite(esdf) || esdf < critical_clearance;
  const bool caution_clearance_hit = esdf < caution_clearance;
  const bool constrained_yaw = safe_count <= constrained_yaw_threshold;
  const bool limited_yaw = safe_count <= limited_yaw_threshold;
  const bool strong_yaw_mismatch = yaw_distance > 4;

  return severe_clearance || constrained_yaw ||
         (caution_clearance_hit && (limited_yaw || strong_yaw_mismatch));
}

 double AdaptiveDiscretizationStep(const GridMap& map, double base_step,
                                  const Eigen::Vector2d& start,
                                  const Eigen::Vector2d& goal) {
  const Eigen::Vector2d midpoint = 0.5 * (start + goal);
  const double min_clearance = std::min(
      map.getEsdf(start.x(), start.y()),
      std::min(map.getEsdf(goal.x(), goal.y()),
               map.getEsdf(midpoint.x(), midpoint.y())));
  const double tight_clearance = std::max(0.40, 3.0 * map.resolution());
  const double open_clearance = std::max(1.20, 8.0 * map.resolution());
  if (!std::isfinite(min_clearance) || min_clearance <= tight_clearance) {
    return std::max(0.75 * base_step, 0.75 * map.resolution());
  }
  if (min_clearance >= open_clearance) {
    return std::max(2.25 * base_step, 1.50 * map.resolution());
  }
  const double ratio = (min_clearance - tight_clearance) /
                       std::max(1e-6, open_clearance - tight_clearance);
  const double scale = 0.75 + 1.50 * ratio;
  return std::max(scale * base_step, 0.75 * map.resolution());
}

void MergeAdjacentLowRiskSegments(std::vector<MotionSegment>* segments) {
  if (segments == nullptr || segments->empty()) {
    return;
  }

  std::vector<MotionSegment> merged;
  merged.reserve(segments->size());
  merged.push_back((*segments)[0]);
  for (size_t i = 1; i < segments->size(); ++i) {
    const MotionSegment& current = (*segments)[i];
    MotionSegment& back = merged.back();
    if (back.risk != RiskLevel::LOW || current.risk != RiskLevel::LOW) {
      merged.push_back(current);
      continue;
    }

    const size_t begin = (!back.waypoints.empty() && !current.waypoints.empty() &&
                          (back.waypoints.back().position() -
                           current.waypoints.front().position()).norm() < 1e-6)
                             ? 1
                             : 0;
    back.waypoints.insert(back.waypoints.end(), current.waypoints.begin() + begin,
                          current.waypoints.end());
  }
  segments->swap(merged);
}

struct FootprintClearanceProbe {
  double clearance = kInf;
  int safe_yaw_count = 0;
  Eigen::Vector2d worst_world = Eigen::Vector2d::Zero();
};

FootprintClearanceProbe EvaluateFootprintClearance(const GridMap& map,
                                                   const CollisionChecker& checker,
                                                   const SE2State& state) {
  FootprintClearanceProbe probe;
  probe.safe_yaw_count =
      static_cast<int>(checker.safeYawCount(state.x, state.y));

  const auto& samples = checker.footprint().denseBodySamples(
      map.resolution() * 0.5, map.resolution());
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);

  for (const Eigen::Vector2d& body : samples) {
    const Eigen::Vector2d world(state.x + c * body.x() - s * body.y(),
                                state.y + s * body.x() + c * body.y());
    const double clearance = map.getEsdf(world.x(), world.y());
    if (clearance >= probe.clearance) {
      continue;
    }
    probe.clearance = clearance;
    probe.worst_world = world;
  }

  if (!std::isfinite(probe.clearance)) {
    probe.clearance = map.getEsdf(state.x, state.y);
    probe.worst_world = state.position();
  }
  return probe;
}

double Clamp01(double value) {
  return std::max(0.0, std::min(1.0, value));
}

int WrapBin(int bin, int bins) {
  int wrapped = bin % bins;
  return wrapped < 0 ? wrapped + bins : wrapped;
}

int LongestCircularSafeRun(const std::vector<int>& safe_bins, int bins) {
  if (safe_bins.empty() || bins <= 0) return 0;
  if (static_cast<int>(safe_bins.size()) >= bins) return bins;

  std::vector<bool> safe(static_cast<size_t>(bins), false);
  for (int bin : safe_bins) safe[static_cast<size_t>(WrapBin(bin, bins))] = true;
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

double TransportedYawOverlap(const std::vector<int>& previous,
                             const std::vector<int>& current, int bins,
                             double reference_yaw_delta) {
  if (previous.empty() || current.empty() || bins <= 0) return 0.0;
  std::vector<bool> current_mask(static_cast<size_t>(bins), false);
  for (int bin : current) {
    current_mask[static_cast<size_t>(WrapBin(bin, bins))] = true;
  }
  const double bin_width = kTwoPi / static_cast<double>(bins);
  const int shift = static_cast<int>(std::lround(reference_yaw_delta / bin_width));
  int overlap = 0;
  for (int bin : previous) {
    if (current_mask[static_cast<size_t>(WrapBin(bin + shift, bins))]) ++overlap;
  }
  const int denominator = std::max(
      1, std::min(static_cast<int>(previous.size()), static_cast<int>(current.size())));
  return Clamp01(static_cast<double>(overlap) / static_cast<double>(denominator));
}

size_t NearestDesiredState(const SE2State& state,
                           const std::vector<SE2State>& desired_states) {
  size_t best_index = 0;
  double best_distance = kInf;
  for (size_t i = 0; i < desired_states.size(); ++i) {
    const double distance =
        (state.position() - desired_states[i].position()).squaredNorm();
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

RiskLevel RawRiskLevel(double score, const ContinuousRiskParams& params) {
  if (!params.enable_three_level) {
    return score >= params.high_threshold ? RiskLevel::HIGH : RiskLevel::LOW;
  }
  if (score >= params.high_threshold) return RiskLevel::HIGH;
  if (score >= params.low_threshold) return RiskLevel::MEDIUM;
  return RiskLevel::LOW;
}

RiskLevel MaxRiskLevel(RiskLevel lhs, RiskLevel rhs) {
  return RiskLevelRank(lhs) >= RiskLevelRank(rhs) ? lhs : rhs;
}

bool SameRiskState(const SE2State& lhs, const SE2State& rhs) {
  return (lhs.position() - rhs.position()).norm() < 1e-7 &&
         std::abs(normalizeAngle(lhs.yaw - rhs.yaw)) < 1e-7;
}

MotionSegment MergeRiskSegments(const MotionSegment& first,
                                const MotionSegment& second) {
  MotionSegment merged;
  merged.risk = MaxRiskLevel(first.risk, second.risk);
  merged.waypoints = first.waypoints;
  const size_t waypoint_begin =
      !merged.waypoints.empty() && !second.waypoints.empty() &&
              SameRiskState(merged.waypoints.back(), second.waypoints.front())
          ? 1
          : 0;
  merged.waypoints.insert(merged.waypoints.end(),
                          second.waypoints.begin() + waypoint_begin,
                          second.waypoints.end());

  merged.risk_profile = first.risk_profile;
  const size_t profile_begin =
      !merged.risk_profile.empty() && !second.risk_profile.empty() &&
              SameRiskState(merged.risk_profile.back().state,
                            second.risk_profile.front().state)
          ? 1
          : 0;
  merged.risk_profile.insert(merged.risk_profile.end(),
                             second.risk_profile.begin() + profile_begin,
                             second.risk_profile.end());
  return merged;
}

void CoalesceUndersupportedRiskSegments(std::vector<MotionSegment>* segments) {
  if (segments == nullptr) return;
  constexpr size_t kMinimumSolverSupport = 3;
  while (segments->size() > 1) {
    size_t short_index = segments->size();
    for (size_t i = 0; i < segments->size(); ++i) {
      if ((*segments)[i].waypoints.size() < kMinimumSolverSupport) {
        short_index = i;
        break;
      }
    }
    if (short_index == segments->size()) break;

    size_t left_index = short_index;
    size_t right_index = short_index;
    if (short_index == 0) {
      right_index = 1;
    } else if (short_index + 1 == segments->size()) {
      left_index = short_index - 1;
    } else {
      // Prefer the smaller neighbor so a one-interval HIGH spike receives
      // enough geometric context without promoting a long open-space run.
      const size_t left_size = (*segments)[short_index - 1].waypoints.size();
      const size_t right_size = (*segments)[short_index + 1].waypoints.size();
      if (left_size <= right_size) {
        left_index = short_index - 1;
      } else {
        right_index = short_index + 1;
      }
    }

    const MotionSegment merged =
        MergeRiskSegments((*segments)[left_index], (*segments)[right_index]);
    (*segments)[left_index] = merged;
    segments->erase(segments->begin() + static_cast<std::ptrdiff_t>(right_index));
  }
}

std::vector<MotionSegment> BuildContinuousRiskSegments(
    const std::vector<MotionSegment>& repaired_segments,
    const std::vector<SE2State>& desired_states, const GridMap& map,
    const CollisionChecker& checker, const ContinuousRiskParams& params,
    SE2GenerationDiagnostics* diagnostics) {
  std::vector<SE2State> states;
  for (const MotionSegment& segment : repaired_segments) {
    for (const SE2State& state : segment.waypoints) {
      if (!states.empty() &&
          (states.back().position() - state.position()).norm() < 1e-7 &&
          std::abs(normalizeAngle(states.back().yaw - state.yaw)) < 1e-7) {
        continue;
      }
      states.push_back(state);
    }
  }
  if (states.size() < 2 || desired_states.empty()) return {};

  const int bins = checker.numYawBins();
  const double uncertainty_erosion =
      std::max(0.0, params.localization_sigma_scale * params.localization_sigma +
                        params.control_error + params.map_error + params.shape_error);
  std::vector<RiskSample> samples(states.size());
  std::vector<std::vector<int>> safe_sets(states.size());
  std::vector<double> reference_yaws(states.size(), 0.0);

  for (size_t i = 0; i < states.size(); ++i) {
    const size_t nearest = NearestDesiredState(states[i], desired_states);
    reference_yaws[i] = desired_states[nearest].yaw;
    safe_sets[i] = checker.safeYawIndices(states[i].x, states[i].y);

    const FootprintClearanceProbe probe =
        EvaluateFootprintClearance(map, checker, states[i]);
    RiskSample sample;
    sample.state = states[i];
    sample.footprint_clearance = probe.clearance;
    sample.uncertainty_erosion = uncertainty_erosion;
    sample.effective_clearance = probe.clearance - uncertainty_erosion;
    sample.feasible_yaw_width =
        bins > 0 ? static_cast<double>(LongestCircularSafeRun(safe_sets[i], bins)) /
                       static_cast<double>(bins)
                 : 0.0;
    sample.feasible_yaw_overlap =
        i == 0
            ? 1.0
            : TransportedYawOverlap(
                  safe_sets[i - 1], safe_sets[i], bins,
                  normalizeAngle(reference_yaws[i] - reference_yaws[i - 1]));
    sample.correction_distance =
        (states[i].position() - desired_states[nearest].position()).norm();

    if (!std::isfinite(sample.effective_clearance) ||
        sample.effective_clearance <= 0.0) {
      sample.clearance_risk = 1.0;
    } else {
      sample.clearance_risk =
          std::exp(-sample.effective_clearance /
                   std::max(1e-3, params.clearance_scale));
    }
    sample.orientation_width_risk = 1.0 - sample.feasible_yaw_width;
    sample.orientation_continuity_risk = 1.0 - sample.feasible_yaw_overlap;
    const double yaw_correction =
        std::abs(normalizeAngle(states[i].yaw - reference_yaws[i])) / kPi;
    sample.correction_risk = Clamp01(
        0.75 * sample.correction_distance /
            std::max(1e-3, params.correction_distance_scale) +
        0.25 * yaw_correction);
    const double nominal_clearance =
        std::isfinite(probe.clearance) ? std::max(0.0, probe.clearance) : 0.0;
    sample.uncertainty_risk = Clamp01(
        uncertainty_erosion /
        std::max(1e-6, nominal_clearance + uncertainty_erosion));

    const double survival =
        (1.0 - Clamp01(params.weight_clearance * sample.clearance_risk)) *
        (1.0 - Clamp01(params.weight_orientation_width *
                       sample.orientation_width_risk)) *
        (1.0 - Clamp01(params.weight_orientation_continuity *
                       sample.orientation_continuity_risk)) *
        (1.0 - Clamp01(params.weight_correction * sample.correction_risk)) *
        (1.0 - Clamp01(params.weight_uncertainty * sample.uncertainty_risk));
    sample.score = Clamp01(1.0 - survival);
    if (safe_sets[i].empty() || sample.effective_clearance <= 0.0) {
      sample.score = 1.0;
    }
    sample.level = RawRiskLevel(sample.score, params);
    samples[i] = sample;
  }

  // Schmitt-trigger hysteresis prevents repeated level toggles around the two
  // thresholds.  The previous accepted level owns the overlap band.
  RiskLevel previous_level = samples.front().level;
  for (size_t i = 1; i < samples.size(); ++i) {
    const double score = samples[i].score;
    RiskLevel level = samples[i].level;
    if (previous_level == RiskLevel::LOW &&
        score < params.low_threshold + params.hysteresis) {
      level = RiskLevel::LOW;
    } else if (previous_level == RiskLevel::MEDIUM) {
      if (score > params.high_threshold + params.hysteresis) {
        level = RiskLevel::HIGH;
      } else if (score >= params.low_threshold - params.hysteresis) {
        level = RiskLevel::MEDIUM;
      }
    } else if (previous_level == RiskLevel::HIGH &&
               score > params.high_threshold - params.hysteresis) {
      level = RiskLevel::HIGH;
    }
    samples[i].level = level;
    previous_level = level;
  }

  // A high-risk state receives a medium-risk guard band.  This preserves the
  // high-fidelity bottleneck while giving stitching one level of context.
  const std::vector<RiskSample> unguarded = samples;
  const int guard = std::max(0, params.guard_states);
  for (size_t i = 0; i < unguarded.size(); ++i) {
    if (unguarded[i].level != RiskLevel::HIGH) continue;
    const size_t begin = i > static_cast<size_t>(guard) ? i - guard : 0;
    const size_t end = std::min(samples.size() - 1, i + static_cast<size_t>(guard));
    for (size_t j = begin; j <= end; ++j) {
      if (samples[j].level == RiskLevel::LOW) {
        samples[j].level = params.enable_three_level ? RiskLevel::MEDIUM
                                                     : RiskLevel::HIGH;
      }
    }
  }

  if (diagnostics != nullptr) {
    diagnostics->risk_sample_count = static_cast<int>(samples.size());
    diagnostics->min_risk_score = 1.0;
    diagnostics->max_risk_score = 0.0;
    diagnostics->mean_risk_score = 0.0;
    for (const RiskSample& sample : samples) {
      diagnostics->min_risk_score =
          std::min(diagnostics->min_risk_score, sample.score);
      diagnostics->max_risk_score =
          std::max(diagnostics->max_risk_score, sample.score);
      diagnostics->mean_risk_score += sample.score;
    }
    diagnostics->mean_risk_score /= static_cast<double>(samples.size());
  }

  std::vector<MotionSegment> output;
  for (size_t i = 0; i + 1 < states.size(); ++i) {
    const RiskLevel interval_level =
        MaxRiskLevel(samples[i].level, samples[i + 1].level);
    if (output.empty() || output.back().risk != interval_level) {
      MotionSegment segment;
      segment.risk = interval_level;
      segment.waypoints.push_back(states[i]);
      segment.risk_profile.push_back(samples[i]);
      output.push_back(segment);
    }
    MotionSegment& segment = output.back();
    segment.waypoints.push_back(states[i + 1]);
    segment.risk_profile.push_back(samples[i + 1]);
  }

  // Risk transitions can create a two-state segment at a corner.  Such a
  // segment has no interior MINCO support and used to trigger an expensive
  // full-SVSDF solve with no useful geometric context.  Merge only those
  // undersupported runs while retaining the maximum risk of both sides.
  CoalesceUndersupportedRiskSegments(&output);

  for (MotionSegment& segment : output) {
    segment.max_risk_score = 0.0;
    double sum = 0.0;
    for (const RiskSample& sample : segment.risk_profile) {
      segment.max_risk_score = std::max(segment.max_risk_score, sample.score);
      sum += sample.score;
    }
    segment.mean_risk_score = segment.risk_profile.empty()
                                  ? 0.0
                                  : sum / segment.risk_profile.size();
  }
  return output;
}

}  // namespace

SE2SequenceGenerator::SE2SequenceGenerator() {}

void SE2SequenceGenerator::init(const GridMap& map, const CollisionChecker& checker,
                                double discretization_step, int max_push_attempts,
                                const ContinuousRiskParams& risk_params,
                                int corner_smoothing_iterations,
                                double corner_smoothing_alpha) {
  map_ = &map;
  checker_ = &checker;
  disc_step_ = discretization_step;
  max_push_ = max_push_attempts;
  corner_smoothing_iterations_ = std::max(0, corner_smoothing_iterations);
  corner_smoothing_alpha_ =
      std::max(0.0, std::min(0.75, corner_smoothing_alpha));
  risk_params_ = risk_params;
}

bool SE2SequenceGenerator::tangentConnectionFree(
    const Eigen::Vector2d& start, const Eigen::Vector2d& goal) const {
  if (map_ == nullptr || checker_ == nullptr) return false;
  const Eigen::Vector2d delta = goal - start;
  const double length = delta.norm();
  if (length < 1e-9) return true;
  const double yaw = std::atan2(delta.y(), delta.x());
  const double step = std::max(0.02, 0.5 * map_->resolution());
  const int samples =
      std::max(1, static_cast<int>(std::ceil(length / step)));
  for (int sample = 0; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) /
                         static_cast<double>(samples);
    const Eigen::Vector2d position = start + ratio * delta;
    if (!checker_->isFree(SE2State(position.x(), position.y(), yaw))) {
      return false;
    }
  }
  return true;
}

std::vector<SE2State> SE2SequenceGenerator::smoothPathStates(
    const std::vector<SE2State>& states) const {
  if (states.size() < 3 || corner_smoothing_iterations_ <= 0 ||
      corner_smoothing_alpha_ <= 0.0 || map_ == nullptr || checker_ == nullptr) {
    return states;
  }

  std::vector<SE2State> smoothed = states;
  for (int iteration = 0; iteration < corner_smoothing_iterations_; ++iteration) {
    std::vector<SE2State> next = smoothed;
    for (size_t i = 1; i + 1 < smoothed.size(); ++i) {
      const Eigen::Vector2d previous = smoothed[i - 1].position();
      const Eigen::Vector2d current = smoothed[i].position();
      const Eigen::Vector2d following = smoothed[i + 1].position();
      const Eigen::Vector2d target =
          (1.0 - corner_smoothing_alpha_) * current +
          0.5 * corner_smoothing_alpha_ * (previous + following);
      const Eigen::Vector2d tangent = following - previous;
      if (tangent.norm() < 1e-6) continue;

      const double yaw = std::atan2(tangent.y(), tangent.x());
      const SE2State candidate(target.x(), target.y(), yaw);
      if (!checker_->isFree(candidate) ||
          !tangentConnectionFree(previous, target) ||
          !tangentConnectionFree(target, following)) {
        continue;
      }
      next[i] = candidate;
    }
    smoothed.swap(next);
  }

  // Recompute interior headings from the final geometry.  Keep an existing
  // safe orientation if the tangent orientation is not footprint-feasible.
  for (size_t i = 1; i + 1 < smoothed.size(); ++i) {
    const Eigen::Vector2d tangent =
        smoothed[i + 1].position() - smoothed[i - 1].position();
    if (tangent.norm() < 1e-6) continue;
    const double yaw = std::atan2(tangent.y(), tangent.x());
    const SE2State tangent_state(smoothed[i].x, smoothed[i].y, yaw);
    if (checker_->isFree(tangent_state)) {
      smoothed[i].yaw = yaw;
    }
  }
  smoothed.front() = states.front();
  smoothed.back() = states.back();
  return smoothed;
}

std::vector<SE2State> SE2SequenceGenerator::discretizePath(const TopoPath& path,
                                                           const SE2State& start,
                                                           const SE2State& goal) {
  std::vector<SE2State> states;
  if (path.waypoints.size() < 2) {
    return states;
  }

  states.push_back(start);
  for (size_t i = 1; i < path.waypoints.size(); ++i) {
    const Eigen::Vector2d delta = path.waypoints[i].pos - path.waypoints[i - 1].pos;
    const double segment_length = delta.norm();
    if (segment_length < 1e-9) {
      continue;
    }

    double tangent_yaw = std::atan2(delta.y(), delta.x());
    if (path.waypoints[i].has_yaw) {
      tangent_yaw = path.waypoints[i].yaw;
    }

    const double local_step =
        AdaptiveDiscretizationStep(*map_, disc_step_, path.waypoints[i - 1].pos,
                                   path.waypoints[i].pos);
    const int samples =
        std::max(1, static_cast<int>(std::ceil(segment_length / local_step)));
    for (int s = 1; s <= samples; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(samples);
      const Eigen::Vector2d position = path.waypoints[i - 1].pos + t * delta;
      SE2State state(position.x(), position.y(), tangent_yaw);
      if (i == path.waypoints.size() - 1 && s == samples) {
        state.yaw = goal.yaw;
      }
      states.push_back(state);
    }
  }

  if (!states.empty()) {
    states.back() = goal;
  }
  return states;
}

bool SE2SequenceGenerator::safeYaw(SE2State& state, double desired_yaw) {
  const int bins = checker_->numYawBins();
  const auto wrap_bin = [bins](int bin) {
    int wrapped = bin % bins;
    return wrapped < 0 ? wrapped + bins : wrapped;
  };

  const int desired_bin = checker_->binFromYaw(desired_yaw);
  if (checker_->isYawSafe(state.x, state.y, desired_bin)) {
    state.yaw = checker_->yawFromBin(desired_bin);
    return true;
  }

  for (int offset = 1; offset <= bins / 2; ++offset) {
    for (int sign : {-1, 1}) {
      const int candidate = wrap_bin(desired_bin + sign * offset);
      if (checker_->isYawSafe(state.x, state.y, candidate)) {
        state.yaw = checker_->yawFromBin(candidate);
        return true;
      }
    }
  }

  return false;
}

bool SE2SequenceGenerator::pushStateFromObstacle(SE2State& state, double desired_yaw) {
  const double eps = map_->resolution();
  const double target_clearance =
      checker_->footprint().inscribedRadius() + map_->resolution();
  for (int attempt = 0; attempt < max_push_; ++attempt) {
    SE2State assigned = state;
    if (safeYaw(assigned, desired_yaw)) {
      state = assigned;
      return true;
    }

    bool found_nearby_safe = false;
    SE2State best_nearby = state;
    double best_score = -kInf;
    const double radius = static_cast<double>(attempt + 1) * eps;
    for (int k = 0; k < 24; ++k) {
      const double theta = kTwoPi * static_cast<double>(k) / 24.0;
      SE2State nearby(state.x + radius * std::cos(theta), state.y + radius * std::sin(theta),
                      desired_yaw);
      SE2State nearby_assigned = nearby;
      if (!safeYaw(nearby_assigned, desired_yaw)) {
        continue;
      }
      const FootprintClearanceProbe probe =
          EvaluateFootprintClearance(*map_, *checker_, nearby_assigned);
      const double score =
          probe.clearance + 0.1 * static_cast<double>(probe.safe_yaw_count);
      if (!found_nearby_safe || score > best_score) {
        found_nearby_safe = true;
        best_score = score;
        best_nearby = nearby_assigned;
      }
    }
    if (found_nearby_safe) {
      state = best_nearby;
      return true;
    }

    const FootprintClearanceProbe probe =
        EvaluateFootprintClearance(*map_, *checker_, state);
    Eigen::Vector2d direction =
        math::EsdfAscentDirection(*map_, probe.worst_world, eps);
    if (direction.norm() < 1e-9) {
      direction = math::EsdfAscentDirection(*map_, state.position(), eps);
    }
    if (direction.norm() < 1e-9) {
      return false;
    }

    const double deficit = std::max(0.0, target_clearance - probe.clearance);
    const double push = std::min(std::max(eps, deficit + eps), 4.0 * eps);
    state.x += push * direction.x();
    state.y += push * direction.y();
    state.yaw = desired_yaw;
  }

  return safeYaw(state, desired_yaw);
}

std::vector<SE2State> SE2SequenceGenerator::buildLinearSegment(const SE2State& start,
                                                               const SE2State& goal) const {
  std::vector<SE2State> segment;
  segment.push_back(start);

  const Eigen::Vector2d delta = goal.position() - start.position();
  const double length = delta.norm();
  if (length < 1e-9) {
    segment.push_back(goal);
    return segment;
  }

  const double tangent_yaw = std::atan2(delta.y(), delta.x());
  const int samples = std::max(1, static_cast<int>(std::ceil(length / disc_step_)));
  for (int s = 1; s < samples; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(samples);
    const Eigen::Vector2d position = start.position() + t * delta;
    segment.push_back(SE2State(position.x(), position.y(), tangent_yaw));
  }

  segment.push_back(goal);
  return segment;
}

bool SE2SequenceGenerator::segAdjustRecursive(const std::vector<SE2State>& seed, int depth,
                                              std::vector<SE2State>& repaired) {
  if (seed.size() < 2) {
    return false;
  }

  const int max_depth = std::max(8, max_push_ * 3);
  std::vector<SE2State> trial = seed;
  size_t fail_index = trial.size();
  for (size_t i = 1; i + 1 < trial.size(); ++i) {
    SE2State assigned = trial[i];
    if (!safeYaw(assigned, seed[i].yaw)) {
      fail_index = i;
      break;
    }
    trial[i] = assigned;
  }

  if (fail_index == trial.size()) {
    repaired = trial;
    return true;
  }

  if (depth >= max_depth || trial.size() < 3) {
    return false;
  }

  SE2State pivot = seed[fail_index];
  if (!pushStateFromObstacle(pivot, seed[fail_index].yaw)) {
    return false;
  }

  std::vector<SE2State> left;
  std::vector<SE2State> right;
  if (!segAdjustRecursive(buildLinearSegment(trial.front(), pivot), depth + 1, left)) {
    return false;
  }
  if (!segAdjustRecursive(buildLinearSegment(pivot, trial.back()), depth + 1, right)) {
    return false;
  }

  repaired = left;
  repaired.insert(repaired.end(), right.begin() + 1, right.end());
  return true;
}

std::vector<MotionSegment> SE2SequenceGenerator::generate(const TopoPath& path,
                                                          const SE2State& start,
                                                          const SE2State& goal,
                                                          SE2GenerationDiagnostics* diagnostics) {
  if (diagnostics != nullptr) *diagnostics = SE2GenerationDiagnostics();
  const auto classify = [&](const SE2State& desired, const SE2State& assigned) {
    const std::chrono::steady_clock::time_point begin =
        std::chrono::steady_clock::now();
    const bool high = IsHighRiskConfiguration(*map_, *checker_, desired, assigned);
    if (diagnostics != nullptr) {
      diagnostics->risk_classification_ms +=
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - begin).count();
    }
    return high;
  };
  if (path.waypoints.size() < 2) {
    return {};
  }

  std::vector<SE2State> states = discretizePath(path, start, goal);
  if (states.size() < 2) {
    return {};
  }
  if (corner_smoothing_iterations_ > 0) {
    const std::chrono::steady_clock::time_point smoothing_begin =
        std::chrono::steady_clock::now();
    states = smoothPathStates(states);
    if (diagnostics != nullptr) {
      diagnostics->path_adjust_ms +=
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - smoothing_begin)
              .count();
    }
  }

  std::vector<MotionSegment> segments;
  MotionSegment current;
  current.risk = RiskLevel::LOW;
  current.waypoints.push_back(states.front());

  size_t i = 1;
  while (i + 1 < states.size()) {
    SE2State assigned = states[i];
    const bool has_safe_yaw = safeYaw(assigned, states[i].yaw);
    const bool high_risk = !has_safe_yaw || classify(states[i], assigned);
    if (!high_risk) {
      current.waypoints.push_back(assigned);
      ++i;
      continue;
    }

    if (current.waypoints.size() > 1) {
      segments.push_back(current);
    }

    size_t next_safe_index = i;
    SE2State next_safe = states[next_safe_index];
    bool found_next_anchor = false;
    for (; next_safe_index < states.size(); ++next_safe_index) {
      next_safe = states[next_safe_index];
      const bool candidate_safe = safeYaw(next_safe, states[next_safe_index].yaw);
      const bool candidate_high =
          !candidate_safe || classify(states[next_safe_index], next_safe);
      if (next_safe_index == states.size() - 1 || !candidate_high) {
        found_next_anchor = true;
        break;
      }
    }
    if (!found_next_anchor) {
      next_safe_index = states.size() - 1;
      next_safe = states.back();
    }

    std::vector<SE2State> seed;
    seed.reserve(next_safe_index - i + 2);
    seed.push_back(current.waypoints.back());
    for (size_t k = i; k <= next_safe_index; ++k) {
      seed.push_back(states[k]);
    }
    seed.back() = next_safe;

    std::vector<SE2State> repaired;
    const std::chrono::steady_clock::time_point adjust_begin =
        std::chrono::steady_clock::now();
    const bool adjusted = segAdjustRecursive(seed, 0, repaired);
    if (diagnostics != nullptr) {
      diagnostics->path_adjust_ms +=
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - adjust_begin).count();
    }
    if (adjusted) {
      bool repaired_high_risk = false;
      for (size_t k = 1; k + 1 < repaired.size(); ++k) {
        SE2State assigned = repaired[k];
        if (!safeYaw(assigned, repaired[k].yaw) ||
            classify(repaired[k], assigned)) {
          repaired_high_risk = true;
          break;
        }
        repaired[k] = assigned;
      }

      MotionSegment repaired_segment;
      repaired_segment.risk = repaired_high_risk ? RiskLevel::HIGH : RiskLevel::LOW;
      repaired_segment.waypoints = repaired;
      segments.push_back(repaired_segment);
      i = next_safe_index + 1;
    } else {
      MotionSegment high_risk_segment;
      high_risk_segment.risk = RiskLevel::HIGH;
      high_risk_segment.waypoints = seed;
      segments.push_back(high_risk_segment);
      i = next_safe_index + 1;
    }

    current = MotionSegment();
    current.risk = RiskLevel::LOW;
    current.waypoints.push_back(segments.back().waypoints.back());
  }

  if (current.waypoints.empty()) {
    current.waypoints.push_back(states.front());
  }
  if ((current.waypoints.back().position() - states.back().position()).norm() > 1e-6) {
    current.waypoints.push_back(states.back());
  } else {
    current.waypoints.back() = states.back();
  }
  if (current.waypoints.size() >= 2) {
    segments.push_back(current);
  }

  const std::chrono::steady_clock::time_point risk_begin =
      std::chrono::steady_clock::now();
  std::vector<MotionSegment> continuous_segments = BuildContinuousRiskSegments(
      segments, states, *map_, *checker_, risk_params_, diagnostics);
  if (diagnostics != nullptr) {
    diagnostics->risk_classification_ms +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - risk_begin)
            .count();
  }
  return continuous_segments;
}

}  // namespace isweep_planner
