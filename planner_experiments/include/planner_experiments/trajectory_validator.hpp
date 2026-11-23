#pragma once

#include "isweep_planner/env/footprint_model.h"
#include "isweep_planner/env/grid_map.h"
#include "planner_experiments/experiment_types.hpp"

namespace planner_experiments {

class TrajectoryValidator {
 public:
  ValidationResult Validate(const nav_msgs::OccupancyGrid& occupancy,
                            const isweep_planner::GridMap& map,
                            const isweep_planner::FootprintModel& footprint,
                            const isweep_planner::Trajectory& trajectory,
                            const DynamicsLimits& limits,
                            const ValidationConfig& config) const;

  ValidationResult Validate(
      const nav_msgs::OccupancyGrid& occupancy,
      const isweep_planner::GridMap& map,
      const isweep_planner::FootprintModel& footprint,
      const isweep_planner::Trajectory& trajectory,
      const DynamicsLimits& limits, const ValidationConfig& config,
      const isweep_planner::SE2State& expected_start,
      const isweep_planner::SE2State& expected_goal) const;

  static double TrajectoryLength(const std::vector<TrajectorySample>& samples);

};

}  // namespace planner_experiments
