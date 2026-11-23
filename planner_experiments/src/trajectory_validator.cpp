#include "planner_experiments/trajectory_validator.hpp"

#include <algorithm>
#include <cmath>

#include "isweep_planner/core/trajectory_footprint_validator.h"

namespace planner_experiments {
namespace {

bool Exceeds(double value, double limit) {
  return std::isfinite(limit) && value > limit + 1e-9;
}

double YawAcceleration(const isweep_planner::Trajectory& trajectory,
                       double t, double total, double nominal_step) {
  const double step = std::max(1e-5, std::min(nominal_step, 0.25 * total));
  if (total <= 1e-9) return 0.0;
  if (t <= step) {
    return (trajectory.sampleYawRate(std::min(total, t + step)) -
            trajectory.sampleYawRate(t)) /
           std::max(1e-9, std::min(total, t + step) - t);
  }
  if (t + step >= total) {
    return (trajectory.sampleYawRate(t) -
            trajectory.sampleYawRate(std::max(0.0, t - step))) /
           std::max(1e-9, t - std::max(0.0, t - step));
  }
  return (trajectory.sampleYawRate(t + step) -
          trajectory.sampleYawRate(t - step)) /
         (2.0 * step);
}

}  // namespace

ValidationResult TrajectoryValidator::Validate(
    const nav_msgs::OccupancyGrid& occupancy,
    const isweep_planner::GridMap& map,
    const isweep_planner::FootprintModel& footprint,
    const isweep_planner::Trajectory& trajectory,
    const DynamicsLimits& limits,
    const ValidationConfig& config) const {
  (void)occupancy;
  ValidationResult result;
  if (trajectory.empty() || trajectory.totalDuration() < 0.0) {
    result.collision_type = "empty_trajectory";
    return result;
  }

  const double total = trajectory.totalDuration();
  isweep_planner::TrajectoryFootprintValidationConfig footprint_config;
  footprint_config.time_step_sec = config.time_step_sec;
  footprint_config.max_translation_step_m = config.max_translation_step_m;
  footprint_config.max_yaw_step_rad = config.max_yaw_step_rad;
  footprint_config.footprint_boundary_step_m =
      config.footprint_boundary_step_m;
  footprint_config.footprint_interior_step_m =
      config.footprint_interior_step_m;
  footprint_config.collision_clearance_m = config.collision_clearance_m;
  footprint_config.max_subdivision_depth = config.max_subdivision_depth;
  const isweep_planner::TrajectoryFootprintValidationResult footprint_result =
      isweep_planner::ValidateTrajectoryFootprint(
          map, footprint, trajectory, footprint_config);
  const std::vector<double>& times = footprint_result.sample_times;
  result.external_footprint_valid = footprint_result.valid;
  result.minimum_external_clearance_m =
      footprint_result.minimum_clearance_m;
  result.collision_time_sec = footprint_result.collision_time_sec;
  result.collision_x = footprint_result.collision_x;
  result.collision_y = footprint_result.collision_y;
  result.collision_yaw = footprint_result.collision_yaw;
  result.collision_type = footprint_result.collision_type;

  std::vector<double> raw_yaw;
  raw_yaw.reserve(times.size());
  for (double time : times) raw_yaw.push_back(trajectory.sample(time).yaw);
  const std::vector<double> unwrapped_yaw = UnwrapYaw(raw_yaw);

  for (size_t i = 0; i < times.size(); ++i) {
    const double time = times[i];
    const isweep_planner::SE2State state = trajectory.sample(time);
    const Eigen::Vector2d velocity = trajectory.sampleVelocity(time);
    const Eigen::Vector2d acceleration = trajectory.sampleAcceleration(time);
    TrajectorySample sample;
    sample.t = time;
    sample.x = state.x;
    sample.y = state.y;
    sample.yaw = unwrapped_yaw[i];
    sample.vx = velocity.x();
    sample.vy = velocity.y();
    sample.ax = acceleration.x();
    sample.ay = acceleration.y();
    sample.yaw_rate = trajectory.sampleYawRate(time);
    sample.yaw_acceleration =
        YawAcceleration(trajectory, time, total,
                        std::max(1e-4, 0.25 * config.time_step_sec));
    result.samples.push_back(sample);

    const double linear_velocity = velocity.norm();
    const double linear_acceleration = acceleration.norm();
    const double angular_velocity = std::abs(sample.yaw_rate);
    const double angular_acceleration = std::abs(sample.yaw_acceleration);
    result.max_linear_velocity =
        std::max(result.max_linear_velocity, linear_velocity);
    result.max_linear_acceleration =
        std::max(result.max_linear_acceleration, linear_acceleration);
    result.max_angular_velocity =
        std::max(result.max_angular_velocity, angular_velocity);
    result.max_angular_acceleration =
        std::max(result.max_angular_acceleration, angular_acceleration);
    if (Exceeds(linear_velocity, limits.max_linear_velocity)) {
      ++result.velocity_violation_count;
    }
    if (Exceeds(linear_acceleration, limits.max_linear_acceleration)) {
      ++result.acceleration_violation_count;
    }
    if (Exceeds(angular_velocity, limits.max_angular_velocity) ||
        Exceeds(angular_acceleration, limits.max_angular_acceleration)) {
      ++result.angular_violation_count;
    }

  }

  const bool limits_complete =
      std::isfinite(limits.max_linear_velocity) &&
      std::isfinite(limits.max_linear_acceleration) &&
      std::isfinite(limits.max_angular_velocity) &&
      std::isfinite(limits.max_angular_acceleration);
  result.dynamics_valid =
      (!config.require_dynamics || limits_complete) &&
      result.velocity_violation_count == 0 &&
      result.acceleration_violation_count == 0 &&
      result.angular_violation_count == 0;

  result.boundary_continuous = true;
  double accumulated_time = 0.0;
  for (size_t i = 0; i + 1 < trajectory.pos_pieces.size(); ++i) {
    const auto& left = trajectory.pos_pieces[i];
    const auto& right = trajectory.pos_pieces[i + 1];
    const double position_jump =
        (left.evaluate(left.duration) - right.evaluate(0.0)).norm();
    const double velocity_jump =
        (left.velocity(left.duration) - right.velocity(0.0)).norm();
    const double acceleration_jump =
        (left.acceleration(left.duration) - right.acceleration(0.0)).norm();
    double yaw_jump = 0.0;
    double yaw_rate_jump = 0.0;
    if (i + 1 < trajectory.yaw_pieces.size()) {
      yaw_jump = std::abs(isweep_planner::normalizeAngle(
          trajectory.yaw_pieces[i].evaluate(trajectory.yaw_pieces[i].duration) -
          trajectory.yaw_pieces[i + 1].evaluate(0.0)));
      yaw_rate_jump = std::abs(
          trajectory.yaw_pieces[i].velocity(trajectory.yaw_pieces[i].duration) -
          trajectory.yaw_pieces[i + 1].velocity(0.0));
    }
    const double normalized_worst = std::max(
        {position_jump / std::max(1e-12, config.max_position_jump_m),
         velocity_jump / std::max(1e-12, config.max_velocity_jump_mps),
         acceleration_jump / std::max(1e-12, config.max_acceleration_jump_mps2),
         yaw_jump / std::max(1e-12, config.max_yaw_jump_rad),
         yaw_rate_jump / std::max(1e-12, config.max_yaw_rate_jump_radps)});
    const double previous_worst = std::max(
        {result.max_position_jump_m / std::max(1e-12, config.max_position_jump_m),
         result.max_velocity_jump_mps / std::max(1e-12, config.max_velocity_jump_mps),
         result.max_acceleration_jump_mps2 /
             std::max(1e-12, config.max_acceleration_jump_mps2),
         result.max_yaw_jump_rad / std::max(1e-12, config.max_yaw_jump_rad),
         result.max_yaw_rate_jump_radps /
             std::max(1e-12, config.max_yaw_rate_jump_radps)});
    if (normalized_worst > previous_worst) {
      result.worst_boundary_index = static_cast<int>(i);
    }
    result.max_position_jump_m =
        std::max(result.max_position_jump_m, position_jump);
    result.max_velocity_jump_mps =
        std::max(result.max_velocity_jump_mps, velocity_jump);
    result.max_acceleration_jump_mps2 =
        std::max(result.max_acceleration_jump_mps2, acceleration_jump);
    result.max_yaw_jump_rad = std::max(result.max_yaw_jump_rad, yaw_jump);
    result.max_yaw_rate_jump_radps =
        std::max(result.max_yaw_rate_jump_radps, yaw_rate_jump);
    accumulated_time += left.duration;
  }
  (void)accumulated_time;
  if (result.max_position_jump_m > config.max_position_jump_m ||
      result.max_velocity_jump_mps > config.max_velocity_jump_mps ||
      result.max_acceleration_jump_mps2 > config.max_acceleration_jump_mps2 ||
      result.max_yaw_jump_rad > config.max_yaw_jump_rad ||
      result.max_yaw_rate_jump_radps > config.max_yaw_rate_jump_radps) {
    result.boundary_continuous = false;
  }
  if (!config.require_boundary_continuity) result.boundary_continuous = true;
  return result;
}

ValidationResult TrajectoryValidator::Validate(
    const nav_msgs::OccupancyGrid& occupancy,
    const isweep_planner::GridMap& map,
    const isweep_planner::FootprintModel& footprint,
    const isweep_planner::Trajectory& trajectory,
    const DynamicsLimits& limits, const ValidationConfig& config,
    const isweep_planner::SE2State& expected_start,
    const isweep_planner::SE2State& expected_goal) const {
  ValidationResult result =
      Validate(occupancy, map, footprint, trajectory, limits, config);
  result.endpoint_check_performed = true;
  if (trajectory.empty()) return result;

  const isweep_planner::SE2State actual_start = trajectory.sample(0.0);
  const isweep_planner::SE2State actual_goal =
      trajectory.sample(trajectory.totalDuration());
  result.start_position_error_m =
      (actual_start.position() - expected_start.position()).norm();
  result.start_yaw_error_rad = std::abs(isweep_planner::normalizeAngle(
      actual_start.yaw - expected_start.yaw));
  result.goal_position_error_m =
      (actual_goal.position() - expected_goal.position()).norm();
  result.goal_yaw_error_rad = std::abs(isweep_planner::normalizeAngle(
      actual_goal.yaw - expected_goal.yaw));
  result.start_matches =
      result.start_position_error_m <=
          config.max_endpoint_position_error_m + 1e-12 &&
      result.start_yaw_error_rad <=
          config.max_endpoint_yaw_error_rad + 1e-12;
  result.goal_reached =
      result.goal_position_error_m <=
          config.max_endpoint_position_error_m + 1e-12 &&
      result.goal_yaw_error_rad <=
          config.max_endpoint_yaw_error_rad + 1e-12;
  result.endpoints_match =
      !config.require_endpoint_match ||
      (result.start_matches && result.goal_reached);
  return result;
}

double TrajectoryValidator::TrajectoryLength(
    const std::vector<TrajectorySample>& samples) {
  double length = 0.0;
  for (size_t i = 1; i < samples.size(); ++i) {
    length += std::hypot(samples[i].x - samples[i - 1].x,
                         samples[i].y - samples[i - 1].y);
  }
  return length;
}

}  // namespace planner_experiments
