#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_validator_node");
  ROS_ERROR("trajectory_validator_node is an offline library entry point; use experiment_runner_node or replay_experiment.py");
  return 2;
}
