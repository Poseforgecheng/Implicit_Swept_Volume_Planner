#pragma once

#include <vector>

#include "isweep_planner/env/collision_checker.h"
#include "isweep_planner/core/common.h"
#include "isweep_planner/env/grid_map.h"

namespace isweep_planner {

struct ContinuousRiskParams {
  bool enable_three_level = true;
  double low_threshold = 0.35;
  double high_threshold = 0.65;
  double hysteresis = 0.05;
  int guard_states = 1;

  double clearance_scale = 0.35;
  double correction_distance_scale = 0.30;
  double localization_sigma = 0.02;
  double localization_sigma_scale = 3.0;
  double control_error = 0.03;
  double map_error = 0.02;
  double shape_error = 0.01;

  double weight_clearance = 0.90;
  double weight_orientation_width = 0.70;
  double weight_orientation_continuity = 0.80;
  double weight_correction = 0.60;
  double weight_uncertainty = 0.75;
};

struct SE2GenerationDiagnostics {
  double risk_classification_ms = 0.0;
  double path_adjust_ms = 0.0;
  int risk_sample_count = 0;
  double min_risk_score = 1.0;
  double max_risk_score = 0.0;
  double mean_risk_score = 0.0;
};

class SE2SequenceGenerator {
 public:
  SE2SequenceGenerator();

  void init(const GridMap& map, const CollisionChecker& checker,
            double discretization_step, int max_push_attempts,
            const ContinuousRiskParams& risk_params = ContinuousRiskParams(),
            int corner_smoothing_iterations = 0,
            double corner_smoothing_alpha = 0.45);

  std::vector<MotionSegment> generate(const TopoPath& path, const SE2State& start,
                                      const SE2State& goal,
                                      SE2GenerationDiagnostics* diagnostics = nullptr);

 private:
  std::vector<SE2State> discretizePath(const TopoPath& path, const SE2State& start,
                                       const SE2State& goal);
  bool safeYaw(SE2State& state, double desired_yaw);
  bool pushStateFromObstacle(SE2State& state, double desired_yaw);
  bool segAdjustRecursive(const std::vector<SE2State>& seed, int depth,
                          std::vector<SE2State>& repaired);
  std::vector<SE2State> buildLinearSegment(const SE2State& start,
                                           const SE2State& goal) const;
  std::vector<SE2State> smoothPathStates(
      const std::vector<SE2State>& states) const;
  bool tangentConnectionFree(const Eigen::Vector2d& start,
                             const Eigen::Vector2d& goal) const;

  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
  double disc_step_ = 0.15;
  int max_push_ = 5;
  int corner_smoothing_iterations_ = 0;
  double corner_smoothing_alpha_ = 0.45;
  ContinuousRiskParams risk_params_;
};

}  // namespace isweep_planner
