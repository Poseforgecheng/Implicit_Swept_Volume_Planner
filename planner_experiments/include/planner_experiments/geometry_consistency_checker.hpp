#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

#include "planner_experiments/experiment_types.hpp"

namespace planner_experiments {

struct GeometryCheckConfig {
  bool enabled = true;
  double max_centroid_error_m = 0.01;
  double max_hausdorff_distance_m = 0.02;
  double outline_sample_step_m = 0.005;
  double max_relative_area_error = 0.05;
  bool fail_experiment_on_mismatch = true;
};

class GeometryConsistencyChecker {
 public:
  GeometryConsistencyResult Check(
      const std::vector<Eigen::Vector2d>& polygon,
      const std::string& mesh_path,
      const GeometryCheckConfig& config) const;

 private:
  static std::vector<Eigen::Vector2d> SampleClosedOutline(
      const std::vector<Eigen::Vector2d>& polygon, double step);
};

}  // namespace planner_experiments
