#pragma once

#include "planner_experiments/experiment_types.hpp"

namespace planner_experiments {

class MetricsCollector {
 public:
  static void ApplyValidation(const ValidationResult& validation,
                              PlanningResult* result);
  static double ControlEffort(const std::vector<TrajectorySample>& samples);
};

}  // namespace planner_experiments
