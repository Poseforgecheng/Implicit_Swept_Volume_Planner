#pragma once

#include <limits>
#include <string>
#include <vector>

#include "isweep_planner/core/common.h"
#include "isweep_planner/env/footprint_model.h"
#include "isweep_planner/env/grid_map.h"

namespace isweep_planner {

// Shared raw-occupancy footprint contract used by both the online runtime
// gate and the experiment-layer independent validator.  Keeping the adaptive
// trajectory sampling and dense body sampling here prevents the two validators
// from silently drifting apart.
struct TrajectoryFootprintValidationConfig {
  double time_step_sec = 0.01;
  double max_translation_step_m = 0.01;
  double max_yaw_step_rad = 0.01;
  double footprint_boundary_step_m = 0.01;
  double footprint_interior_step_m = 0.02;
  double collision_clearance_m = 0.0;
  int max_subdivision_depth = 24;
};

struct TrajectoryFootprintValidationResult {
  bool valid = false;
  double minimum_clearance_m = kInf;
  double collision_time_sec = std::numeric_limits<double>::quiet_NaN();
  double collision_x = std::numeric_limits<double>::quiet_NaN();
  double collision_y = std::numeric_limits<double>::quiet_NaN();
  double collision_yaw = std::numeric_limits<double>::quiet_NaN();
  std::string collision_type;
  std::vector<double> sample_times;
};

TrajectoryFootprintValidationResult ValidateTrajectoryFootprint(
    const GridMap& map, const FootprintModel& footprint,
    const Trajectory& trajectory,
    const TrajectoryFootprintValidationConfig& config);

}  // namespace isweep_planner
