#include "isweep_planner/core/trajectory_footprint_validator.h"

#include <algorithm>
#include <cmath>

#include "isweep_planner/core/math_utils.h"

namespace isweep_planner {
namespace {

void SubdivideTimes(const Trajectory& trajectory, double t0, double t1,
                    int depth,
                    const TrajectoryFootprintValidationConfig& config,
                    std::vector<double>* times) {
  if (times == nullptr || t1 <= t0) return;
  const SE2State first = trajectory.sample(t0);
  const SE2State second = trajectory.sample(t1);
  const double translation = (second.position() - first.position()).norm();
  const double yaw_delta = std::abs(normalizeAngle(second.yaw - first.yaw));
  const bool split =
      (t1 - t0) > config.time_step_sec + 1e-12 ||
      translation > config.max_translation_step_m + 1e-12 ||
      yaw_delta > config.max_yaw_step_rad + 1e-12;
  if (split && depth < config.max_subdivision_depth) {
    const double middle = 0.5 * (t0 + t1);
    SubdivideTimes(trajectory, t0, middle, depth + 1, config, times);
    SubdivideTimes(trajectory, middle, t1, depth + 1, config, times);
    return;
  }
  times->push_back(t1);
}

}  // namespace

TrajectoryFootprintValidationResult ValidateTrajectoryFootprint(
    const GridMap& map, const FootprintModel& footprint,
    const Trajectory& trajectory,
    const TrajectoryFootprintValidationConfig& config) {
  TrajectoryFootprintValidationResult result;
  if (!map.ready()) {
    result.collision_type = "map_not_ready";
    return result;
  }
  if (trajectory.empty() || !std::isfinite(trajectory.totalDuration()) ||
      trajectory.totalDuration() < 0.0) {
    result.collision_type = "empty_trajectory";
    return result;
  }

  const double total = trajectory.totalDuration();
  result.sample_times.assign(1, 0.0);
  SubdivideTimes(trajectory, 0.0, total, 0, config, &result.sample_times);
  std::sort(result.sample_times.begin(), result.sample_times.end());
  result.sample_times.erase(
      std::unique(result.sample_times.begin(), result.sample_times.end()),
      result.sample_times.end());

  const auto& body_samples = footprint.denseBodySamples(
      config.footprint_boundary_step_m,
      config.footprint_interior_step_m);
  result.valid = true;
  for (double time : result.sample_times) {
    const SE2State state = trajectory.sample(time);
    const double cosine = std::cos(state.yaw);
    const double sine = std::sin(state.yaw);
    bool pose_collision = false;
    std::string pose_collision_type;
    double pose_minimum = kInf;
    for (const Eigen::Vector2d& body : body_samples) {
      const double wx = state.x + cosine * body.x() - sine * body.y();
      const double wy = state.y + sine * body.x() + cosine * body.y();
      const GridIndex cell = map.worldToGrid(wx, wy);
      if (!map.isInside(cell.x, cell.y)) {
        pose_collision = true;
        pose_collision_type = "out_of_map";
        pose_minimum = -kInf;
        break;
      }
      const double clearance = map.getEsdf(wx, wy);
      pose_minimum = std::min(pose_minimum, clearance);
      if (map.isRawOccupied(cell.x, cell.y)) {
        pose_collision = true;
        pose_collision_type = "occupied_grid";
        break;
      }
      if (clearance < config.collision_clearance_m) {
        pose_collision = true;
        pose_collision_type = "clearance_threshold";
        break;
      }
    }
    result.minimum_clearance_m =
        std::min(result.minimum_clearance_m, pose_minimum);
    if (pose_collision && result.valid) {
      result.valid = false;
      result.collision_time_sec = time;
      result.collision_x = state.x;
      result.collision_y = state.y;
      result.collision_yaw = state.yaw;
      result.collision_type = pose_collision_type;
    }
  }
  return result;
}

}  // namespace isweep_planner
