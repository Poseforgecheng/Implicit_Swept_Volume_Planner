#include <ros/ros.h>

#include <iostream>
#include <string>

#include "planner_experiments/experiment_runner.hpp"

namespace {

bool ParseArguments(int argc, char** argv,
                    planner_experiments::RunnerArguments* output) {
  for (int i = 1; i < argc; ++i) {
    const std::string argument(argv[i]);
    if (argument.compare(0, 2, "__") == 0) continue;
    if (i + 1 >= argc) return false;
    const std::string value(argv[++i]);
    if (argument == "--config") output->config_path = value;
    else if (argument == "--tasks") output->tasks_path = value;
    else if (argument == "--method") output->method_name = value;
    else if (argument == "--task-id") output->task_id = value;
    else if (argument == "--map-yaml") output->map_yaml = value;
    else if (argument == "--robot-yaml") output->robot_yaml = value;
    else if (argument == "--output-dir") output->output_directory = value;
    else if (argument == "--run-id") output->run_id = value;
    else if (argument == "--configuration-hash") output->configuration_hash = value;
    else if (argument == "--seed") output->seed = static_cast<uint32_t>(std::stoul(value));
    else return false;
  }
  return !output->config_path.empty() && !output->tasks_path.empty() &&
         !output->method_name.empty() && !output->task_id.empty() &&
         !output->map_yaml.empty() && !output->robot_yaml.empty() &&
         !output->output_directory.empty() && !output->run_id.empty();
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_experiments_runner");
  planner_experiments::RunnerArguments arguments;
  if (!ParseArguments(argc, argv, &arguments)) {
    std::cerr << "usage: experiment_runner_node --config FILE --tasks FILE "
                 "--method NAME --task-id ID --map-yaml FILE --robot-yaml FILE "
                 "--output-dir DIR --run-id ID --seed N\n";
    return 2;
  }
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string error;
  const bool written =
      planner_experiments::ExperimentRunner().Run(nh, pnh, arguments, &error);
  if (!error.empty()) ROS_ERROR("Experiment run: %s", error.c_str());
  return written ? 0 : 1;
}
