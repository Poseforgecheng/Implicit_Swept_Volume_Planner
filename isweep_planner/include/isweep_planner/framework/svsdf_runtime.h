#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <functional>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>

#include "isweep_planner/env/collision_checker.h"
#include "isweep_planner/core/common.h"
#include "isweep_planner/core/trajectory_footprint_validator.h"
#include "isweep_planner/env/footprint_model.h"
#include "isweep_planner/env/grid_map.h"
#include "isweep_planner/global_search/hybrid_astar.h"
#include "isweep_planner/optimization/midend/se2_sequence_generator.h"
#include "isweep_planner/optimization/backend/se2_svsdf_solver.h"
#include "isweep_planner/optimization/evaluator/svsdf_evaluator.h"
#include "isweep_planner/global_search/topology_planner.h"
#include "isweep_planner/optimization/backend/trajectory_optimizer.h"

namespace isweep_planner {

// Local-planner-facing semantic sample extracted from the selected global
// candidate. New-PAD can consume this as a risk-tagged reference state stream.
struct GlobalReferencePointData {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  int risk_level = 0;  // 0=low, 1=medium, 2=high
  double risk_score = 0.0;
  double effective_clearance = std::numeric_limits<double>::quiet_NaN();
  double feasible_yaw_width = std::numeric_limits<double>::quiet_NaN();
  double feasible_yaw_overlap = std::numeric_limits<double>::quiet_NaN();
  double correction_distance = std::numeric_limits<double>::quiet_NaN();
  double uncertainty_erosion = std::numeric_limits<double>::quiet_NaN();
  double clearance_risk = std::numeric_limits<double>::quiet_NaN();
  double orientation_width_risk = std::numeric_limits<double>::quiet_NaN();
  double orientation_continuity_risk =
      std::numeric_limits<double>::quiet_NaN();
  double correction_risk = std::numeric_limits<double>::quiet_NaN();
  double uncertainty_risk = std::numeric_limits<double>::quiet_NaN();
  double clearance = kInf;
  int segment_id = 0;
  double s = 0.0;
  double preferred_speed = 0.0;
};

// Container published after planning succeeds. It preserves segment semantics
// from the global planner without changing the existing Plan() control flow.
struct RiskAwareGlobalReferenceData {
  ros::Time stamp;
  std::string frame_id = "map";
  std::vector<GlobalReferencePointData> points;
  double total_length = 0.0;
  int num_segments = 0;
  int num_medium_risk_segments = 0;
  int num_high_risk_segments = 0;
};

// One accepted replacement of the validated incumbent.  The event is kept in
// monotonic planner time so an experiment can reconstruct the actual anytime
// quality staircase instead of inferring it from a final counter.
struct RuntimeIncumbentEvent {
  int update_index = 0;
  double elapsed_ms = std::numeric_limits<double>::quiet_NaN();
  int candidate_index = -1;
  double minimum_clearance_m = std::numeric_limits<double>::quiet_NaN();
  double smoothness_cost = std::numeric_limits<double>::quiet_NaN();
  double trajectory_length_m = std::numeric_limits<double>::quiet_NaN();
  double trajectory_duration_sec = std::numeric_limits<double>::quiet_NaN();
  int piece_count = 0;
  int low_segment_count = 0;
  int medium_segment_count = 0;
  int high_segment_count = 0;
  bool recovery_success = false;
};

// Experiment-only three-level solve for one selected path segment.  The
// experiment layer independently validates R2, explicit SE(2), and full-SVSDF
// trajectories; solver success alone is never treated as a ground-truth label.
struct RuntimeRiskGroundTruthTrial {
  int candidate_index = -1;
  int segment_index = -1;
  RiskLevel assigned_level = RiskLevel::LOW;
  double maximum_risk_score = std::numeric_limits<double>::quiet_NaN();
  double mean_risk_score = std::numeric_limits<double>::quiet_NaN();
  int state_count = 0;
  double segment_length_m = std::numeric_limits<double>::quiet_NaN();
  SE2State expected_start;
  SE2State expected_goal;
  bool low_solver_attempted = false;
  bool low_solver_success = false;
  double low_solve_ms = std::numeric_limits<double>::quiet_NaN();
  Trajectory low_trajectory;
  bool medium_solver_attempted = false;
  bool medium_solver_success = false;
  double medium_solve_ms = std::numeric_limits<double>::quiet_NaN();
  Trajectory medium_trajectory;
  bool strict_solver_attempted = false;
  bool strict_solver_success = false;
  bool full_svsdf_used = false;
  bool full_svsdf_success = false;
  double strict_solve_ms = std::numeric_limits<double>::quiet_NaN();
  double full_svsdf_ms = std::numeric_limits<double>::quiet_NaN();
  Trajectory strict_trajectory;
};

struct SvsdfPlanResult {
  bool success = false;
  bool validated = false;
  bool degraded = false;
  bool timeout = false;
  int selected_candidate_index = -1;
  nav_msgs::Path coarse_path;
  nav_msgs::Path trajectory;
  Trajectory raw_trajectory; // Keep the raw continuous trajectory for marker generation
  RiskAwareGlobalReferenceData risk_aware_global_reference;
  std::vector<RuntimeRiskGroundTruthTrial> risk_ground_truth_trials;
  PlanningStats stats;
};

// Experiment-only controls. Defaults preserve the pre-experiment planner
// behavior. They select existing branches; they do not replace algorithms.
struct RuntimeExperimentOptions {
  std::string method_name = "current_full";
  uint32_t random_seed = 1;
  double total_time_budget_sec = 20.0;
  bool force_all_low = false;
  bool force_all_strict = false;
  bool force_full_svsdf = false;
  bool hybrid_astar_only = false;
  bool hybrid_frontend_fallback = true;
  bool recovery_enabled = true;
  bool local_upgrade = true;
  bool strict_local_patch = true;
  bool swept_astar = true;
  bool hybrid_astar_recovery = true;
  bool candidate_switch = true;
  bool tail_refinement = true;
  // Full-SVSDF work allowed after a validated incumbent already exists.
  // Negative values mean unlimited. Tail refinement has its own stricter
  // quota because it is optional quality improvement rather than feasibility.
  int post_incumbent_svsdf_budget = 4;
  int tail_refinement_svsdf_budget = 1;
  bool collect_risk_ground_truth = false;
  bool risk_ground_truth_run_low = true;
  bool risk_ground_truth_run_medium = true;
  bool risk_ground_truth_run_strict = true;
};

// Apply one of the paper-facing comparison methods while preserving shared
// quantities such as the deadline, random seed, and strict-call quotas from
// the caller-provided base options. Returning false leaves the input unchanged.
inline bool ConfigureRuntimeMethod(
    const std::string& method_name, RuntimeExperimentOptions* options) {
  if (options == nullptr) {
    return false;
  }

  RuntimeExperimentOptions configured = *options;
  configured.method_name = method_name;
  configured.force_all_low = false;
  configured.force_all_strict = false;
  configured.force_full_svsdf = false;
  configured.hybrid_astar_only = false;
  configured.hybrid_frontend_fallback = true;
  configured.recovery_enabled = true;
  configured.local_upgrade = true;
  configured.strict_local_patch = true;
  configured.swept_astar = true;
  configured.hybrid_astar_recovery = true;
  configured.candidate_switch = true;
  configured.tail_refinement = true;

  if (method_name == "current_full") {
    // The complete adaptive method uses all scheduler and recovery branches.
  } else if (method_name == "full_r2") {
    configured.force_all_low = true;
    configured.hybrid_frontend_fallback = false;
    configured.recovery_enabled = false;
    configured.local_upgrade = false;
    configured.strict_local_patch = false;
    configured.swept_astar = false;
    configured.hybrid_astar_recovery = false;
    configured.candidate_switch = false;
    configured.tail_refinement = false;
  } else if (method_name == "full_strict") {
    configured.force_all_strict = true;
    configured.force_full_svsdf = true;
  } else if (method_name == "hybrid_astar") {
    configured.hybrid_astar_only = true;
    configured.recovery_enabled = false;
    configured.local_upgrade = false;
    configured.strict_local_patch = false;
    configured.swept_astar = false;
    configured.hybrid_astar_recovery = false;
    configured.candidate_switch = false;
    configured.tail_refinement = false;
  } else {
    return false;
  }

  *options = configured;
  return true;
}

// Attribute an actual full-SVSDF backend invocation to the planner phase that
// requested it.  The aggregate full_svsdf_call_count remains available for
// backwards compatibility, while these sources explain why it increased.
enum class FullSvsdfCallSource {
  INITIAL_ASSIGNED,
  INITIAL_ESCALATION,
  FRONTEND_FALLBACK,
  VALIDATION_RECOVERY,
  TAIL_REFINEMENT,
  MEASUREMENT,
  OTHER,
};

// A forced all-R2 baseline is a solver ablation, not a starting-level hint.
// If R2 fails, that candidate must fail instead of silently entering SE(2) or
// SVSDF and contaminating the baseline.
inline bool InitialLevelEscalationAllowed(
    const RuntimeExperimentOptions& options) {
  return !options.force_all_low;
}

// Preserve the adaptive method's segment partition and change only the
// assigned solver level. Merging the candidate would pay the SVSDF setup cost
// once and confound the risk-scheduling ablation.
inline void ForceAllSegmentsStrict(std::vector<MotionSegment>* segments) {
  if (segments == nullptr) return;
  for (MotionSegment& segment : *segments) {
    segment.risk = RiskLevel::HIGH;
  }
}

// Recovery and refinement can generate new segments after the initial
// baseline transformation. Reapply the forced-strict contract at every such
// solver boundary so a LOW/MEDIUM label cannot escape into a cheaper solver.
inline RiskLevel EffectiveSolverLevel(
    const RuntimeExperimentOptions& options, RiskLevel assigned_level) {
  return options.force_all_strict ? RiskLevel::HIGH : assigned_level;
}

struct RuntimeExperimentTelemetry {
  int candidate_count = 0;
  int selected_candidate_index = -1;
  bool degraded = false;
  bool timeout = false;
  int low_solver_call_count = 0;
  int low_solver_success_count = 0;
  int medium_solver_call_count = 0;
  int medium_solver_success_count = 0;
  int strict_solver_call_count = 0;
  int strict_solver_success_count = 0;
  int full_svsdf_call_count = 0;
  int full_svsdf_success_count = 0;
  int full_svsdf_initial_assigned_call_count = 0;
  int full_svsdf_initial_escalation_call_count = 0;
  int full_svsdf_frontend_fallback_call_count = 0;
  int full_svsdf_validation_recovery_call_count = 0;
  int full_svsdf_tail_refinement_call_count = 0;
  int full_svsdf_other_call_count = 0;
  int preserve_shape_bypass_count = 0;
  int initial_level_upgrade_attempt_count = 0;
  int initial_level_upgrade_success_count = 0;
  int frontend_fallback_attempt_count = 0;
  int frontend_fallback_success_count = 0;
  int validation_recovery_attempt_count = 0;
  int validation_recovery_success_count = 0;
  int local_upgrade_attempt_count = 0;
  int local_upgrade_success_count = 0;
  int strict_patch_attempt_count = 0;
  int strict_patch_success_count = 0;
  int swept_astar_attempt_count = 0;
  int swept_astar_success_count = 0;
  int swept_astar_expanded_nodes = 0;
  int hybrid_astar_attempt_count = 0;
  int hybrid_astar_success_count = 0;
  int candidate_switch_count = 0;
  int post_incumbent_svsdf_call_count = 0;
  int post_incumbent_svsdf_budget_reject_count = 0;
  int tail_refinement_svsdf_budget_reject_count = 0;
  int external_equivalent_gate_attempt_count = 0;
  int external_equivalent_gate_reject_count = 0;
  int incumbent_update_count = 0;
  int incumbent_reject_count = 0;
  int validation_failure_count = 0;
  bool recovery_success = false;
  std::vector<RuntimeIncumbentEvent> incumbent_events;
  double original_trajectory_length_m =
      std::numeric_limits<double>::quiet_NaN();
  double replanned_length_m = std::numeric_limits<double>::quiet_NaN();
  double patch_length_m = std::numeric_limits<double>::quiet_NaN();
  double trajectory_reuse_ratio = std::numeric_limits<double>::quiet_NaN();
  double t_roadmap_ms = std::numeric_limits<double>::quiet_NaN();
  double t_dijkstra_ms = std::numeric_limits<double>::quiet_NaN();
  double t_shortcut_ms = std::numeric_limits<double>::quiet_NaN();
  double t_path_adjust_ms = std::numeric_limits<double>::quiet_NaN();
  double t_se2_sequence_ms = std::numeric_limits<double>::quiet_NaN();
  double t_risk_classification_ms = std::numeric_limits<double>::quiet_NaN();
  double t_low_solve_ms = 0.0;
  double t_medium_solve_ms = 0.0;
  double t_strict_solve_ms = 0.0;
  double t_full_svsdf_ms = 0.0;
  double t_stitch_ms = 0.0;
  double t_internal_validation_ms = 0.0;
  double t_recovery_ms = 0.0;
  double t_first_feasible_ms = std::numeric_limits<double>::quiet_NaN();
  double t_risk_ground_truth_ms = 0.0;
};

class SvsdfRuntime {
 public:
  struct CandidateResult;
  using ExternalValidationGate = std::function<bool(
      const Trajectory&, const SE2State&, const SE2State&, double*, double*,
      double*, std::string*, double*)>;

  void Initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  bool UpdateMap(const nav_msgs::OccupancyGrid& map);
  SvsdfPlanResult Plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
  void SetExperimentOptions(const RuntimeExperimentOptions& options) {
    experiment_options_ = options;
  }
  void SetFootprintValidationConfig(
      const TrajectoryFootprintValidationConfig& config) {
    footprint_validation_config_ = config;
  }
  void SetExternalValidationGate(ExternalValidationGate gate) {
    external_validation_gate_ = std::move(gate);
  }
  const RuntimeExperimentOptions& experiment_options() const {
    return experiment_options_;
  }
  const RuntimeExperimentTelemetry& last_experiment_telemetry() const {
    return experiment_telemetry_;
  }
  // Build a local-planner-ready reference from the selected candidate:
  // geometry from the optimized trajectory, risk from the continuous
  // assessment labels (when their partition is aligned), and clearance from
  // the existing SVSDF evaluator. Forced baseline solver levels are not
  // published as geometric risk.
  RiskAwareGlobalReferenceData BuildRiskAwareGlobalReference(
      const CandidateResult& candidate) const;

  bool map_ready() const { return map_ready_; }
  const FootprintModel& footprint() const { return footprint_; }
  const GridMap& grid_map() const { return grid_map_; }
  const CollisionChecker& collision_checker() const { return collision_checker_; }
  const OptimizerParams& optimizer_params() const { return optimizer_params_; }
  double last_initialization_ms() const { return last_initialization_ms_; }
  double last_esdf_build_ms() const { return last_esdf_build_ms_; }
  double last_map_precompute_ms() const { return last_map_precompute_ms_; }

  struct CandidateResult {
    int candidate_index = -1;
    Trajectory trajectory;
    // Continuous geometric risk before a baseline forces its solver level.
    // This remains authoritative for local control and visualization.
    std::vector<MotionSegment> assessed_segments;
    // Effective segments and levels used by the optimization backend.
    std::vector<MotionSegment> segments;
    std::vector<Trajectory> segment_trajectories;
    double min_clearance = -kInf;
    double max_vel = 0.0;
    double max_acc = 0.0;
    int support_points = 0;
    int high_risk_segments = 0;
    int medium_risk_segments = 0;
    int escalated_segments = 0;
    double segment_solve_time = 0.0;
    double full_feasibility_time = 0.0;
    double tail_refine_time = 0.0;
    double recovery_time = 0.0;
    int tail_attempts = 0;
    int tail_pruned = 0;
    bool degraded = false;
    bool validated = false;
    bool budget_exhausted = false;
    bool external_equivalent_gate_passed = false;
    double original_trajectory_length_m =
        std::numeric_limits<double>::quiet_NaN();
    double replanned_length_m = std::numeric_limits<double>::quiet_NaN();
    double patch_length_m = std::numeric_limits<double>::quiet_NaN();
    double trajectory_reuse_ratio = std::numeric_limits<double>::quiet_NaN();
    std::string validation_failure_reason;
  };

 private:
  friend class RecoveryManager;

  void LoadParameters(ros::NodeHandle& pnh);
  double EstimateSegmentTime(const std::vector<SE2State>& waypoints) const;
  CandidateResult EvaluateCandidate(const std::vector<MotionSegment>& segments,
                                    const SE2State& requested_start,
                                    const SE2State& requested_goal,
                                    const ros::WallTime& deadline,
                                    bool frontend_fallback_candidate = false);
  void PushSegmentWaypointsFromObstacles(std::vector<MotionSegment>* segments) const;
  bool SolveStrictSegment(const std::vector<SE2State>& waypoints,
                          Trajectory* trajectory,
                          bool require_full_svsdf = false,
                          bool allow_shape_preserving_rollback = true,
                          FullSvsdfCallSource source =
                              FullSvsdfCallSource::OTHER);
  bool OptimizeMediumRiskSegment(const std::vector<SE2State>& waypoints,
                                 double seg_time, Trajectory* trajectory,
                                 bool preserve_failed_trajectory = false);
  bool OptimizeLowRiskSegment(const std::vector<SE2State>& waypoints,
                              double seg_time, Trajectory* trajectory);
  bool TryRecoverBottleneck(const std::vector<MotionSegment>& segments,
                            const std::vector<Trajectory>& segment_trajectories,
                            const Trajectory& current_traj,
                            const ros::WallTime& deadline,
                            CandidateResult* result);
  void MaybeImproveTailTrajectory(const std::vector<MotionSegment>& segments,
                                  const std::vector<Trajectory>& segment_trajectories,
                                  const ros::WallTime& deadline,
                                  CandidateResult* result);
  bool IsTrajectoryFeasible(const Trajectory& traj, double* min_clearance,
                            double* max_vel, double* max_acc,
                            double* max_yaw_rate = nullptr,
                            std::string* failure_reason = nullptr,
                            double* failure_time = nullptr) const;
  bool PassesExternalEquivalentFootprintGate(
      const Trajectory& traj, double* min_clearance = nullptr,
      std::string* failure_reason = nullptr,
      double* failure_time = nullptr) const;
  bool PassesExternalEquivalentValidationGate(
      const Trajectory& traj, const SE2State& requested_start,
      const SE2State& requested_goal, double* min_clearance = nullptr,
      double* max_vel = nullptr, double* max_acc = nullptr,
      std::string* failure_reason = nullptr,
      double* failure_time = nullptr) const;
  std::vector<RuntimeRiskGroundTruthTrial> CollectRiskGroundTruth(
      const CandidateResult& candidate);

  // Segment solve cache
  using SegmentCacheKey = std::pair<size_t, int>; // (segment index, RiskLevel rank)
  struct SegmentCacheHash {
    size_t operator()(const SegmentCacheKey& k) const {
      return std::hash<size_t>()(k.first) ^ (std::hash<int>()(k.second) << 16);
    }
  };
  using SegmentCache = std::unordered_map<SegmentCacheKey, Trajectory, SegmentCacheHash>;

  bool SolveStrictCached(size_t idx, const std::vector<SE2State>& wp,
                         Trajectory* traj, SegmentCache& cache,
                         bool require_full_svsdf = false,
                         FullSvsdfCallSource source =
                             FullSvsdfCallSource::OTHER);
  bool OptimizeMediumRiskCached(size_t idx, const std::vector<SE2State>& wp,
                                double t, Trajectory* traj,
                                SegmentCache& cache);
  bool OptimizeLowRiskCached(size_t idx, const std::vector<SE2State>& wp,
                             double t, Trajectory* traj, SegmentCache& cache);
  double PreferredSpeedForRisk(RiskLevel risk) const;

  GridMap grid_map_;
  FootprintModel footprint_;
  CollisionChecker collision_checker_;
  SE2SequenceGenerator se2_generator_;
  HybridAStarPlanner hybrid_astar_;
  TopologyPlanner topology_planner_;
  SvsdfEvaluator svsdf_evaluator_;
  TrajectoryOptimizer optimizer_;
  Se2SvsdfSolver strict_solver_;

  OptimizerParams optimizer_params_;
  HybridAStarParams hybrid_astar_params_;
  ContinuousRiskParams continuous_risk_params_;

  int topology_num_samples_ = 520;
  int topology_knn_ = 18;
  int topology_max_paths_ = 14;
  double se2_disc_step_ = 0.15;
  int yaw_bins_ = 18;
  int max_push_attempts_ = 5;
  int se2_corner_smoothing_iterations_ = 0;
  double se2_corner_smoothing_alpha_ = 0.45;
  bool coalesce_escalated_equal_level_segments_ = false;
  double reference_sample_dt_ = 0.15;
  double reference_min_preferred_speed_ = 0.35;
  double reference_max_preferred_speed_ = 0.85;
  double validation_min_clearance_ = 0.0;
  double validation_velocity_tolerance_ = 1e-3;
  double validation_acceleration_tolerance_ = 1e-2;
  double validation_yaw_rate_tolerance_ = 1e-3;
  double validation_max_yaw_acceleration_ = 6.0;
  double validation_yaw_acceleration_tolerance_ = 1e-2;
  double validation_position_jump_tolerance_ = 1e-4;
  double validation_velocity_jump_tolerance_ = 1e-3;
  double validation_acceleration_jump_tolerance_ = 1e-2;
  double validation_yaw_jump_tolerance_ = 1e-3;
  double validation_yaw_rate_jump_tolerance_ = 1e-2;

  nav_msgs::OccupancyGrid latest_map_;
  RuntimeExperimentOptions experiment_options_;
  RuntimeExperimentTelemetry experiment_telemetry_;
  TrajectoryFootprintValidationConfig footprint_validation_config_;
  ExternalValidationGate external_validation_gate_;
  bool post_incumbent_budget_active_ = false;
  double last_initialization_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_esdf_build_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_map_precompute_ms_ = std::numeric_limits<double>::quiet_NaN();
  bool initialized_ = false;
  bool map_ready_ = false;
};

}  // namespace isweep_planner
