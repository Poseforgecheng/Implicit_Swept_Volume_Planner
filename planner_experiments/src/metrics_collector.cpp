#include "planner_experiments/metrics_collector.hpp"
#include "planner_experiments/trajectory_validator.hpp"

#include <cmath>

namespace planner_experiments {

double MetricsCollector::ControlEffort(
    const std::vector<TrajectorySample>& samples) {
  double integral = 0.0;
  for (size_t i = 1; i < samples.size(); ++i) {
    const double dt = samples[i].t - samples[i - 1].t;
    if (dt <= 0.0) continue;
    const double previous = samples[i - 1].ax * samples[i - 1].ax +
                            samples[i - 1].ay * samples[i - 1].ay +
                            samples[i - 1].yaw_acceleration *
                                samples[i - 1].yaw_acceleration;
    const double current = samples[i].ax * samples[i].ax +
                           samples[i].ay * samples[i].ay +
                           samples[i].yaw_acceleration *
                               samples[i].yaw_acceleration;
    integral += 0.5 * dt * (previous + current);
  }
  return integral;
}

void MetricsCollector::ApplyValidation(const ValidationResult& validation,
                                       PlanningResult* result) {
  if (!result) return;
  ExperimentMetrics& metrics = result->metrics;
  metrics.validation = validation;
  metrics.external_footprint_valid = validation.external_footprint_valid;
  metrics.dynamics_valid = validation.dynamics_valid;
  metrics.boundary_continuous = validation.boundary_continuous;
  metrics.endpoint_check_performed = validation.endpoint_check_performed;
  metrics.start_matches = validation.start_matches;
  metrics.goal_reached = validation.goal_reached;
  metrics.endpoints_match = validation.endpoints_match;
  metrics.minimum_external_clearance_m =
      validation.minimum_external_clearance_m;
  metrics.control_effort = ControlEffort(validation.samples);
  metrics.trajectory_length_m =
      TrajectoryValidator::TrajectoryLength(validation.samples);
  metrics.executable =
      result->planner_returned && validation.external_footprint_valid &&
      validation.dynamics_valid && validation.boundary_continuous &&
      validation.endpoint_check_performed && validation.endpoints_match &&
      !result->degraded;
  if (metrics.executable) {
    result->status = PlannerStatus::EXECUTABLE;
  } else if (result->degraded) {
    result->status = PlannerStatus::DEGRADED_DIAGNOSTIC;
  } else if (result->timeout && !result->planner_returned) {
    result->status = PlannerStatus::TIMEOUT;
  } else if (result->planner_returned && !validation.external_footprint_valid) {
    result->status = PlannerStatus::PLANNER_RETURNED;
    result->failure_stage = "external_footprint_validation";
    result->failure_reason = validation.collision_type;
  } else if (result->planner_returned && !validation.dynamics_valid) {
    result->status = PlannerStatus::FOOTPRINT_VALIDATED;
    result->failure_stage = "dynamics_validation";
    result->failure_reason = "dynamics limit violation or missing required limit";
  } else if (result->planner_returned && !validation.boundary_continuous) {
    result->status = PlannerStatus::DYNAMICS_VALIDATED;
    result->failure_stage = "boundary_continuity";
    result->failure_reason = "piece boundary discontinuity";
  } else if (result->planner_returned &&
             (!validation.endpoint_check_performed ||
              !validation.endpoints_match)) {
    result->status = PlannerStatus::DYNAMICS_VALIDATED;
    result->failure_stage = "endpoint_validation";
    result->failure_reason = !validation.endpoint_check_performed
                                 ? "endpoint check was not performed"
                                 : (!validation.goal_reached
                                        ? "trajectory did not reach requested goal"
                                        : "trajectory changed requested start");
  }
  metrics.planner_status = result->status;
  metrics.failure_stage = result->failure_stage;
  metrics.failure_reason = result->failure_reason;
}

}  // namespace planner_experiments
