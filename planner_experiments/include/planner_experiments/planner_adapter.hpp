#pragma once

#include <ros/ros.h>

#include "isweep_planner/framework/svsdf_runtime.h"
#include "planner_experiments/experiment_types.hpp"

namespace planner_experiments {

class PlannerAdapter {
 public:
  PlannerAdapter() = default;

  bool Initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                  const nav_msgs::OccupancyGrid& map,
                  const MethodConfig& method,
                  const ValidationConfig& validation,
                  std::string* error);
  PlanningResult Plan(const PlanningRequest& request);
  void SetExternalValidationLimits(const DynamicsLimits& limits) {
    external_validation_limits_ = limits;
  }

  const isweep_planner::SvsdfRuntime& runtime() const { return runtime_; }
  const DynamicsLimits& plannerDynamicsLimits() const { return planner_limits_; }

 private:
  void CollectReferenceMetrics(
      const PlanningRequest& request,
      const isweep_planner::RiskAwareGlobalReferenceData& reference,
      PlanningResult* result) const;

  isweep_planner::SvsdfRuntime runtime_;
  MethodConfig method_;
  ValidationConfig validation_config_;
  nav_msgs::OccupancyGrid validation_map_;
  DynamicsLimits planner_limits_;
  DynamicsLimits external_validation_limits_;
  bool initialized_ = false;
};

}  // namespace planner_experiments
