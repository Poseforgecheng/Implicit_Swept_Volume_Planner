#include "isweep_planner/local_planner/risk_aware_local_planner.h"

#include <algorithm>
#include <cmath>

namespace isweep_planner {

namespace {

double Distance2d(double x0, double y0, double x1, double y1) {
  const double dx = x0 - x1;
  const double dy = y0 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

const char* ToString(LocalPlannerMode mode) {
  switch (mode) {
    case LocalPlannerMode::LOW_RISK_FOLLOW:
      return "LOW_RISK_FOLLOW";
    case LocalPlannerMode::HIGH_RISK_STRICT:
      return "HIGH_RISK_STRICT";
    case LocalPlannerMode::BLOCKED_RECOVERY:
      return "BLOCKED_RECOVERY";
  }
  return "UNKNOWN_MODE";
}

const char* ToString(LocalPlannerStatus status) {
  switch (status) {
    case LocalPlannerStatus::OK:
      return "OK";
    case LocalPlannerStatus::BLOCKED:
      return "BLOCKED";
    case LocalPlannerStatus::DIVERGED:
      return "DIVERGED";
    case LocalPlannerStatus::NEED_REPLAN:
      return "NEED_REPLAN";
    case LocalPlannerStatus::NO_REFERENCE:
      return "NO_REFERENCE";
  }
  return "UNKNOWN_STATUS";
}

RiskAwareLocalPlanner::RiskAwareLocalPlanner(const GridMap& map,
                                             const CollisionChecker& checker,
                                             const FootprintModel& footprint)
    : map_(&map), checker_(&checker), footprint_(&footprint) {}

bool RiskAwareLocalPlanner::Initialize(ros::NodeHandle& pnh) {
  pnh.param("local_planner/window_length", window_length_, 3.0);
  pnh.param("local_planner/window_resample_spacing", window_resample_spacing_, 0.25);
  pnh.param("local_planner/high_risk_ratio_threshold", high_risk_ratio_threshold_, 0.35);
  pnh.param("local_planner/low_risk_smoothing_alpha", low_risk_smoothing_alpha_, 0.35);
  pnh.param("local_planner/high_risk_lateral_replan_threshold",
            high_risk_lateral_replan_threshold_, 0.35);
  pnh.param("local_planner/high_risk_yaw_replan_threshold",
            high_risk_yaw_replan_threshold_, 0.45);
  pnh.param("local_planner/strict_reference_deviation_threshold",
            strict_reference_deviation_threshold_, 0.40);
  pnh.param("local_planner/low_risk_clearance_margin", low_risk_clearance_margin_, 0.10);
  pnh.param("local_planner/high_risk_clearance_margin", high_risk_clearance_margin_, 0.05);
  pnh.param("local_planner/enforce_clearance_target", enforce_clearance_target_, true);
  pnh.param("local_planner/terminal_pose_control_enabled",
            terminal_pose_control_enabled_, false);
  pnh.param("local_planner/terminal_control_distance", terminal_control_distance_, 0.80);
  pnh.param("local_planner/terminal_position_tolerance",
            terminal_position_tolerance_, 0.10);
  pnh.param("local_planner/terminal_yaw_tolerance", terminal_yaw_tolerance_, 0.20);
  pnh.param("local_planner/terminal_linear_gain", terminal_linear_gain_, 0.80);
  pnh.param("local_planner/terminal_heading_gain", terminal_heading_gain_, 1.50);
  pnh.param("local_planner/terminal_yaw_gain", terminal_yaw_gain_, 1.00);
  pnh.param("local_planner/terminal_yaw_blend_distance",
            terminal_yaw_blend_distance_, 0.40);
  pnh.param("local_planner/terminal_min_speed", terminal_min_speed_, 0.04);
  pnh.param("local_planner/terminal_max_speed", terminal_max_speed_, 0.20);
  pnh.param("local_planner/terminal_max_yaw_rate", terminal_max_yaw_rate_, 0.80);
  pnh.param("local_planner/terminal_min_turn_radius",
            terminal_min_turn_radius_, 0.30);
  pnh.param("local_planner/terminal_orientation_radius",
            terminal_orientation_radius_, 0.12);
  pnh.param("local_planner/terminal_orientation_half_cycles",
            terminal_orientation_half_cycles_, 20);
  pnh.param("local_planner/push_step_size", push_step_size_, 0.05);
  pnh.param("local_planner/push_max_iters", push_max_iters_, 8);
  pnh.param("local_planner/consecutive_failure_limit", consecutive_failure_limit_, 3);
  pnh.param("local_planner/blocked_window_limit", blocked_window_limit_, 2);
  return map_ != nullptr && checker_ != nullptr && footprint_ != nullptr;
}

LocalPlanningResult RiskAwareLocalPlanner::Plan(
    const LocalPlannerState& current_state,
    const RiskAwareGlobalReferenceData& global_ref) {
  LocalPlanningResult result;
  if (map_ == nullptr || checker_ == nullptr || footprint_ == nullptr ||
      global_ref.points.empty()) {
    result.status = LocalPlannerStatus::NO_REFERENCE;
    result.debug_reason = "missing map/checker/reference";
    return result;
  }

  if (terminal_pose_control_enabled_ &&
      BuildTerminalPoseCommand(current_state, global_ref, &result)) {
    return result;
  }

  const int nearest_index = FindNearestReferenceIndex(current_state, global_ref);
  if (nearest_index < 0) {
    result.status = LocalPlannerStatus::NO_REFERENCE;
    result.debug_reason = "failed to match current pose to global reference";
    return result;
  }

  double high_risk_ratio = 0.0;
  result.local_reference =
      ExtractLocalReferenceWindow(global_ref, nearest_index, &high_risk_ratio);
  result.matched_index = nearest_index;
  if (result.local_reference.size() < 2) {
    result.status = LocalPlannerStatus::NO_REFERENCE;
    result.debug_reason = "local reference window too short";
    return result;
  }

  const GlobalReferencePointData& matched = global_ref.points[nearest_index];
  result.tracking_error = std::abs(SignedLateralError(current_state, matched));
  result.yaw_error = std::abs(WrappedYawError(current_state.yaw, matched.yaw));
  result.mode = DetermineLocalPlannerMode(result.local_reference, high_risk_ratio);

  if (matched.risk_level >= RiskLevelRank(RiskLevel::HIGH) &&
      (result.tracking_error > high_risk_lateral_replan_threshold_ ||
       result.yaw_error > high_risk_yaw_replan_threshold_)) {
    ++consecutive_failures_;
    result.need_replan = true;
    result.status = LocalPlannerStatus::NEED_REPLAN;
    result.consecutive_failures = consecutive_failures_;
    result.debug_reason = "high-risk divergence from global reference";
    return result;
  }

  if (!BuildLocalTrajectory(current_state, result.local_reference, result.mode, &result)) {
    ++consecutive_failures_;
    ++consecutive_blocked_cycles_;
    result.consecutive_failures = consecutive_failures_;
    result.blocked = true;
    if (consecutive_failures_ >= consecutive_failure_limit_ ||
        consecutive_blocked_cycles_ >= blocked_window_limit_) {
      result.need_replan = true;
      result.status = LocalPlannerStatus::NEED_REPLAN;
      if (result.debug_reason.empty()) {
        result.debug_reason = "local planning failed repeatedly";
      }
    } else {
      result.status = LocalPlannerStatus::BLOCKED;
      if (result.debug_reason.empty()) {
        result.debug_reason = "local window blocked";
      }
    }
    return result;
  }

  if (!EvaluateLocalTrajectory(result.local_path, result.local_reference, &result)) {
    ++consecutive_failures_;
    ++consecutive_blocked_cycles_;
    result.consecutive_failures = consecutive_failures_;
    result.blocked = true;
    result.need_replan =
        consecutive_failures_ >= consecutive_failure_limit_ ||
        consecutive_blocked_cycles_ >= blocked_window_limit_;
    result.status = result.need_replan ? LocalPlannerStatus::NEED_REPLAN
                                       : LocalPlannerStatus::BLOCKED;
    if (result.debug_reason.empty()) {
      result.debug_reason = result.need_replan ? "trajectory validation failed repeatedly"
                                               : "trajectory validation failed";
    }
    return result;
  }

  result.need_replan = CheckNeedReplan(current_state, result.local_reference, result);
  result.success = !result.local_path.empty() && !result.need_replan;
  result.status = result.need_replan ? LocalPlannerStatus::NEED_REPLAN
                                     : LocalPlannerStatus::OK;
  result.consecutive_failures = result.need_replan ? consecutive_failures_ + 1 : 0;
  if (result.need_replan) {
    ++consecutive_failures_;
    if (result.debug_reason.empty()) {
      result.debug_reason = "strict local path deviates from high-risk reference";
    }
    return result;
  }

  consecutive_failures_ = 0;
  consecutive_blocked_cycles_ = 0;
  result.consecutive_failures = 0;
  UpdateCommand(current_state, result.local_reference, &result);
  return result;
}

bool RiskAwareLocalPlanner::BuildTerminalPoseCommand(
    const LocalPlannerState& current_state,
    const RiskAwareGlobalReferenceData& global_ref,
    LocalPlanningResult* result) {
  if (result == nullptr || global_ref.points.empty()) {
    return false;
  }

  const GlobalReferencePointData& goal = global_ref.points.back();
  const double dx = goal.x - current_state.x;
  const double dy = goal.y - current_state.y;
  const double distance = std::hypot(dx, dy);
  if (distance > std::max(terminal_control_distance_,
                          terminal_position_tolerance_)) {
    return false;
  }

  const double goal_yaw_error =
      WrappedYawError(goal.yaw, current_state.yaw);
  result->mode = LocalPlannerMode::LOW_RISK_FOLLOW;
  result->matched_index = static_cast<int>(global_ref.points.size()) - 1;
  result->tracking_error = distance;
  result->yaw_error = std::abs(goal_yaw_error);
  result->min_clearance = map_->getEsdf(current_state.x, current_state.y);
  result->local_reference.push_back(goal);
  result->local_path.emplace_back(current_state.x, current_state.y,
                                  current_state.yaw);
  result->local_path.emplace_back(goal.x, goal.y, goal.yaw);

  if (distance <= terminal_position_tolerance_ &&
      std::abs(goal_yaw_error) <= terminal_yaw_tolerance_) {
    result->success = true;
    result->status = LocalPlannerStatus::OK;
    result->debug_reason = "goal pose attained";
    return true;
  }

  if (!checker_->isFree(result->local_path.front())) {
    result->blocked = true;
    result->need_replan = true;
    result->status = LocalPlannerStatus::NEED_REPLAN;
    result->debug_reason = "terminal controller starts in collision";
    return true;
  }

  const double min_speed = std::max(0.0, terminal_min_speed_);
  const double max_speed = std::max(min_speed, terminal_max_speed_);
  double linear = 0.0;
  double angular = 0.0;
  bool reverse = false;

  if (distance <= std::max(terminal_position_tolerance_,
                           terminal_orientation_radius_) &&
      std::abs(goal_yaw_error) > terminal_yaw_tolerance_) {
    const int half_cycles = std::max(1, terminal_orientation_half_cycles_);
    if (terminal_orientation_cycle_count_ >= half_cycles) {
      terminal_orientation_direction_ *= -1.0;
      terminal_orientation_cycle_count_ = 0;
    }
    ++terminal_orientation_cycle_count_;
    reverse = terminal_orientation_direction_ < 0.0;
    linear = terminal_orientation_direction_ * min_speed;
    const double radius = std::max(1.0e-3, terminal_min_turn_radius_);
    angular = std::copysign(std::abs(linear) / radius, goal_yaw_error);
  } else {
    terminal_orientation_direction_ = 1.0;
    terminal_orientation_cycle_count_ = 0;
    const double bearing = std::atan2(dy, dx);
    const double pi = std::acos(-1.0);
    const double forward_heading_error =
        WrappedYawError(bearing, current_state.yaw);
    reverse = std::abs(forward_heading_error) > 0.5 * pi;
    const double motion_heading =
        reverse ? normalizeAngle(current_state.yaw + pi) : current_state.yaw;
    const double motion_heading_error =
        WrappedYawError(bearing, motion_heading);
    const double direction = reverse ? -1.0 : 1.0;
    const double speed = std::min(
        max_speed,
        std::max(min_speed,
                 std::max(0.0, terminal_linear_gain_) * distance));
    linear = direction * speed;
    const double yaw_blend = std::exp(
        -distance / std::max(1.0e-3, terminal_yaw_blend_distance_));
    angular = terminal_heading_gain_ * motion_heading_error +
              terminal_yaw_gain_ * yaw_blend * goal_yaw_error;
  }
  const double max_yaw = std::max(0.0, terminal_max_yaw_rate_);
  angular = std::max(-max_yaw, std::min(max_yaw, angular));
  if (terminal_min_turn_radius_ > 1.0e-6) {
    const double radius_limit = std::abs(linear) / terminal_min_turn_radius_;
    angular = std::max(-radius_limit, std::min(radius_limit, angular));
  }

  // Validate a short forward-Euler footprint sample before issuing the
  // terminal command. The independent IR-SIM collision gate remains final.
  const double preview_dt = 0.20;
  const SE2State preview(
      current_state.x + preview_dt * linear * std::cos(current_state.yaw),
      current_state.y + preview_dt * linear * std::sin(current_state.yaw),
      normalizeAngle(current_state.yaw + preview_dt * angular));
  if (!checker_->isFree(preview)) {
    result->blocked = true;
    result->need_replan = true;
    result->status = LocalPlannerStatus::NEED_REPLAN;
    result->debug_reason = "terminal command footprint preview collides";
    return true;
  }

  result->local_cmd.linear_velocity = linear;
  result->local_cmd.angular_velocity = angular;
  result->success = true;
  result->status = LocalPlannerStatus::OK;
  result->debug_reason = reverse ? "terminal pose control reverse"
                                 : "terminal pose control forward";
  return true;
}

std::vector<GlobalReferencePointData> RiskAwareLocalPlanner::ExtractLocalReferenceWindow(
    const RiskAwareGlobalReferenceData& global_ref, int nearest_index,
    double* high_risk_ratio) const {
  std::vector<GlobalReferencePointData> window;
  if (nearest_index < 0 || nearest_index >= static_cast<int>(global_ref.points.size())) {
    return window;
  }

  const double start_s = global_ref.points[nearest_index].s;
  double next_emit_s = start_s;
  int high_risk_count = 0;

  for (size_t i = static_cast<size_t>(nearest_index); i < global_ref.points.size(); ++i) {
    const GlobalReferencePointData& point = global_ref.points[i];
    if (point.s > start_s + window_length_ + 1e-6) {
      break;
    }
    if (!window.empty() && point.s + 1e-6 < next_emit_s &&
        i + 1 < global_ref.points.size()) {
      continue;
    }
    window.push_back(point);
    next_emit_s = point.s + window_resample_spacing_;
    if (point.risk_level >= RiskLevelRank(RiskLevel::MEDIUM)) {
      ++high_risk_count;
    }
  }

  if (window.size() == 1 && nearest_index + 1 < static_cast<int>(global_ref.points.size())) {
    window.push_back(global_ref.points[nearest_index + 1]);
    if (global_ref.points[nearest_index + 1].risk_level >=
        RiskLevelRank(RiskLevel::MEDIUM)) {
      ++high_risk_count;
    }
  }

  if (high_risk_ratio != nullptr) {
    *high_risk_ratio = window.empty()
                           ? 0.0
                           : static_cast<double>(high_risk_count) /
                                 static_cast<double>(window.size());
  }
  return window;
}

LocalPlannerMode RiskAwareLocalPlanner::DetermineLocalPlannerMode(
    const std::vector<GlobalReferencePointData>& window,
    double high_risk_ratio) const {
  if (window.empty()) {
    return LocalPlannerMode::BLOCKED_RECOVERY;
  }
  if (high_risk_ratio >= high_risk_ratio_threshold_ ||
      window.front().risk_level >= RiskLevelRank(RiskLevel::MEDIUM)) {
    return LocalPlannerMode::HIGH_RISK_STRICT;
  }
  if (consecutive_failures_ > 0 || consecutive_blocked_cycles_ > 0) {
    return LocalPlannerMode::BLOCKED_RECOVERY;
  }
  return LocalPlannerMode::LOW_RISK_FOLLOW;
}

bool RiskAwareLocalPlanner::BuildLocalTrajectory(
    const LocalPlannerState& current_state,
    const std::vector<GlobalReferencePointData>& window,
    LocalPlannerMode mode, LocalPlanningResult* result) const {
  if (result == nullptr || window.size() < 2) {
    return false;
  }

  std::vector<SE2State> path;
  path.reserve(window.size() + 1);
  path.push_back(SE2State(current_state.x, current_state.y, current_state.yaw));

  for (size_t i = 0; i < window.size(); ++i) {
    const GlobalReferencePointData& point = window[i];
    Eigen::Vector2d target(point.x, point.y);
    if (mode == LocalPlannerMode::LOW_RISK_FOLLOW && i > 0 && i + 1 < window.size()) {
      const Eigen::Vector2d prev(window[i - 1].x, window[i - 1].y);
      const Eigen::Vector2d next(window[i + 1].x, window[i + 1].y);
      const Eigen::Vector2d smoothed = 0.25 * prev + 0.50 * target + 0.25 * next;
      target = (1.0 - low_risk_smoothing_alpha_) * target +
               low_risk_smoothing_alpha_ * smoothed;
    }

    const double target_clearance = TargetClearance(mode, point);
    const double reference_yaw = point.yaw;
    const SE2State initial_guess(target.x(), target.y(), reference_yaw);
    const bool footprint_free = checker_->isFree(initial_guess);
    const bool clearance_satisfied =
        map_->getEsdf(target.x(), target.y()) > target_clearance;
    bool point_safe = footprint_free &&
                      (!enforce_clearance_target_ || clearance_satisfied);
    if (!point_safe) {
      point_safe = math::PushPointFromObstacle(*map_, target, target_clearance,
                                              push_step_size_, push_max_iters_);
      if (!point_safe ||
          !checker_->isFree(SE2State(target.x(), target.y(), reference_yaw))) {
        result->debug_reason = "unable to push local waypoint out of collision";
        return false;
      }
    }

    if (mode == LocalPlannerMode::HIGH_RISK_STRICT &&
        Distance2d(target.x(), target.y(), point.x, point.y) >
            strict_reference_deviation_threshold_) {
      result->debug_reason = "strict mode exceeded reference deviation threshold";
      return false;
    }

    double yaw = reference_yaw;
    if (mode != LocalPlannerMode::HIGH_RISK_STRICT &&
        Distance2d(path.back().x, path.back().y, target.x(), target.y()) > 1e-4) {
      yaw = std::atan2(target.y() - path.back().y, target.x() - path.back().x);
    }
    path.push_back(SE2State(target.x(), target.y(), yaw));
  }

  if (path.size() < 2) {
    result->debug_reason = "local path has insufficient support points";
    return false;
  }

  path.front().yaw = current_state.yaw;
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    const Eigen::Vector2d delta = path[i + 1].position() - path[i].position();
    if (delta.norm() > 1e-5) {
      path[i].yaw = std::atan2(delta.y(), delta.x());
    }
  }
  path.back().yaw = window.back().yaw;

  result->local_path = path;
  return true;
}

bool RiskAwareLocalPlanner::EvaluateLocalTrajectory(
    const std::vector<SE2State>& path,
    const std::vector<GlobalReferencePointData>& window,
    LocalPlanningResult* result) const {
  if (result == nullptr || path.size() < 2) {
    return false;
  }

  result->min_clearance = kInf;
  for (size_t i = 0; i < path.size(); ++i) {
    const double clearance = map_->getEsdf(path[i].x, path[i].y);
    result->min_clearance = std::min(result->min_clearance, clearance);
    if (!std::isfinite(clearance) || !checker_->isFree(path[i])) {
      result->debug_reason = "local path endpoint collides";
      return false;
    }
  }

  const double line_step = std::max(0.05, 0.5 * map_->resolution());
  for (size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector2d delta = path[i].position() - path[i - 1].position();
    const double distance = delta.norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(distance / line_step)));
    for (int s = 0; s <= samples; ++s) {
      const double alpha = static_cast<double>(s) / static_cast<double>(samples);
      const Eigen::Vector2d interp = path[i - 1].position() + alpha * delta;
      const double yaw =
          std::atan2(std::sin((1.0 - alpha) * path[i - 1].yaw + alpha * path[i].yaw),
                     std::cos((1.0 - alpha) * path[i - 1].yaw + alpha * path[i].yaw));
      const SE2State sample(interp.x(), interp.y(), yaw);
      const double clearance = map_->getEsdf(interp.x(), interp.y());
      result->min_clearance = std::min(result->min_clearance, clearance);
      if (!std::isfinite(clearance) || !checker_->isFree(sample)) {
        result->debug_reason = "local path segment collides";
        return false;
      }
    }
  }

  if (enforce_clearance_target_ && !window.empty()) {
    const double required_clearance =
        std::max(0.0, TargetClearance(result->mode, window.front()));
    if (result->min_clearance <= required_clearance - 1e-3) {
      result->debug_reason = "trajectory clearance below local target";
      return false;
    }
  }
  return true;
}

bool RiskAwareLocalPlanner::CheckNeedReplan(
    const LocalPlannerState& current_state,
    const std::vector<GlobalReferencePointData>& window,
    const LocalPlanningResult& partial) const {
  if (window.empty()) {
    return true;
  }

  const GlobalReferencePointData& head = window.front();
  if (head.risk_level >= RiskLevelRank(RiskLevel::HIGH)) {
    const double lateral = std::abs(SignedLateralError(current_state, head));
    const double yaw_error = std::abs(WrappedYawError(current_state.yaw, head.yaw));
    if (lateral > high_risk_lateral_replan_threshold_ ||
        yaw_error > high_risk_yaw_replan_threshold_) {
      return true;
    }
  }

  if (partial.mode == LocalPlannerMode::HIGH_RISK_STRICT &&
      partial.local_path.size() > 1) {
    size_t compare_count = std::min(partial.local_path.size() - 1, window.size());
    for (size_t i = 0; i < compare_count; ++i) {
      const SE2State& local = partial.local_path[i + 1];
      const GlobalReferencePointData& ref = window[i];
      if (Distance2d(local.x, local.y, ref.x, ref.y) >
          strict_reference_deviation_threshold_) {
        return true;
      }
    }
  }

  return false;
}

int RiskAwareLocalPlanner::FindNearestReferenceIndex(
    const LocalPlannerState& current_state,
    const RiskAwareGlobalReferenceData& global_ref) const {
  int best_index = -1;
  double best_distance = kInf;
  for (size_t i = 0; i < global_ref.points.size(); ++i) {
    const double distance = Distance2d(current_state.x, current_state.y,
                                       global_ref.points[i].x,
                                       global_ref.points[i].y);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = static_cast<int>(i);
    }
  }
  return best_index;
}

double RiskAwareLocalPlanner::SignedLateralError(
    const LocalPlannerState& current_state,
    const GlobalReferencePointData& ref_point) const {
  const double dx = current_state.x - ref_point.x;
  const double dy = current_state.y - ref_point.y;
  return -std::sin(ref_point.yaw) * dx + std::cos(ref_point.yaw) * dy;
}

double RiskAwareLocalPlanner::WrappedYawError(double yaw,
                                              double reference_yaw) const {
  return normalizeAngle(yaw - reference_yaw);
}

double RiskAwareLocalPlanner::TargetClearance(
    LocalPlannerMode mode, const GlobalReferencePointData& ref_point) const {
  const double base = std::max(footprint_->inscribedRadius(), map_->resolution());
  const double reference_clearance =
      std::isfinite(ref_point.clearance) ? ref_point.clearance : base;
  if (mode == LocalPlannerMode::HIGH_RISK_STRICT) {
    return std::min(std::max(base, reference_clearance - high_risk_clearance_margin_),
                    reference_clearance);
  }
  return std::min(std::max(base, reference_clearance - low_risk_clearance_margin_),
                  std::max(reference_clearance, base));
}

void RiskAwareLocalPlanner::UpdateCommand(
    const LocalPlannerState& current_state,
    const std::vector<GlobalReferencePointData>& window,
    LocalPlanningResult* result) const {
  if (result == nullptr || result->local_path.size() < 2 || window.empty()) {
    return;
  }

  const SE2State& lookahead =
      result->local_path[std::min<size_t>(2, result->local_path.size() - 1)];
  const double heading =
      std::atan2(lookahead.y - current_state.y, lookahead.x - current_state.x);
  const double yaw_error = WrappedYawError(heading, current_state.yaw);

  double preferred_speed = 0.0;
  for (size_t i = 0; i < window.size(); ++i) {
    preferred_speed += window[i].preferred_speed;
  }
  preferred_speed /= static_cast<double>(window.size());
  if (result->mode == LocalPlannerMode::HIGH_RISK_STRICT) {
    preferred_speed *= 0.8;
  } else if (result->mode == LocalPlannerMode::BLOCKED_RECOVERY) {
    preferred_speed *= 0.5;
  }

  result->local_cmd.linear_velocity = std::max(0.0, preferred_speed);
  result->local_cmd.angular_velocity = std::max(-1.2, std::min(1.2, 1.5 * yaw_error));
}

}  // namespace isweep_planner
