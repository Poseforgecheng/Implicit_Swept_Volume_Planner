#pragma once

#include <ros/ros.h>

#include <string>
#include <vector>

#include "planner_experiments/experiment_types.hpp"

namespace planner_experiments {

struct RunnerArguments {
  std::string config_path;
  std::string tasks_path;
  std::string method_name;
  std::string task_id;
  std::string map_yaml;
  std::string robot_yaml;
  std::string output_directory;
  std::string run_id;
  std::string configuration_hash;
  uint32_t seed = 1;
};

class ExperimentRunner {
 public:
  bool Run(ros::NodeHandle& nh, ros::NodeHandle& pnh,
           const RunnerArguments& arguments, std::string* error) const;
};

}  // namespace planner_experiments
