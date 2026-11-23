#pragma once

#include <vector>

#include "isweep_planner/core/common.h"
#include "isweep_planner/env/grid_map.h"
#include "isweep_planner/optimization/evaluator/svsdf_evaluator.h"

namespace isweep_planner {

struct OptimizerParams {
  int max_iterations = 15;
  double lambda_smooth = 1.0;
  double lambda_time = 1.0;
  double lambda_safety = 50.0;
  double lambda_dynamics = 1.0;
  double lambda_pos_residual = 5.0;
  double lambda_yaw_residual = 2.0;
  double max_vel = 1.0;
  double max_acc = 2.0;
  double max_yaw_rate = 1.5;
  double safety_margin = 0.1;
  double step_size = 0.01;
  // Preserve enough optimized support states when fitting the final quintic
  // pieces. Long chords through a rounded state chain recreate a cusp.
  double fit_max_piece_length_r2 = 1.0;
  double fit_max_piece_length_se2 = 1.5;
  double fit_max_heading_change = 4.0;
  // Fraction of the adjacent chord speed used at a solver-level boundary.
  // A nonzero shared velocity prevents a zero-speed C2 join from hiding a
  // geometric cusp while keeping the transition local and conservative.
  double stitch_tangent_speed_scale = 0.35;
  double stitch_max_tangent_speed_ratio = 0.60;
  int stitch_guard_pieces = 0;
  int post_stitch_smoothing_iterations = 0;
  double post_stitch_smoothing_alpha = 0.40;
  double post_stitch_sample_spacing = 0.12;
};

class TrajectoryOptimizer {
 public:
  TrajectoryOptimizer();

  void init(const GridMap& map, const SvsdfEvaluator& svsdf,
            const OptimizerParams& params);

  Trajectory optimizeSE2(const std::vector<SE2State>& waypoints, double total_time);
  Trajectory optimizeR2(const std::vector<SE2State>& waypoints, double total_time);
  Trajectory stitch(const std::vector<MotionSegment>& segments,
                    const std::vector<Trajectory>& optimized);

 private:
  void fitMincoQuintic(const std::vector<Eigen::Vector2d>& positions,
                       const std::vector<double>& durations,
                       const Eigen::Vector2d& v0, const Eigen::Vector2d& vf,
                       const Eigen::Vector2d& a0, const Eigen::Vector2d& af,
                       std::vector<PolyPiece>& pieces);
  void fitYawQuintic(const std::vector<double>& yaws,
                     const std::vector<double>& durations,
                     double omega0, double omegaf,
                     std::vector<YawPolyPiece>& pieces);
  std::vector<double> allocateTime(const std::vector<SE2State>& waypoints,
                                   double total_time);
  Trajectory smoothStitchedTrajectory(const Trajectory& trajectory);

  const GridMap* map_ = nullptr;
  const SvsdfEvaluator* svsdf_ = nullptr;
  OptimizerParams params_;
};

}  // namespace isweep_planner
