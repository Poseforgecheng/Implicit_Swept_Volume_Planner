#include "planner_experiments/experiment_runner.hpp"

#include <map_server/image_loader.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>

#include "planner_experiments/geometry_consistency_checker.hpp"
#include "planner_experiments/metrics_collector.hpp"
#include "planner_experiments/planner_adapter.hpp"
#include "planner_experiments/result_writer.hpp"
#include "planner_experiments/trajectory_validator.hpp"

namespace planner_experiments {
namespace {

struct RobotGeometry {
  std::string name;
  std::vector<Eigen::Vector2d> polygon;
  std::string mesh_path;
  double safety_margin = 0.0;
  double inscribed_radius = MissingDouble();
};

std::string DirectoryName(const std::string& path) {
  const size_t slash = path.find_last_of('/');
  return slash == std::string::npos ? "." : path.substr(0, slash);
}

bool IsAbsolute(const std::string& path) {
  return !path.empty() && path.front() == '/';
}

std::string ResolveRelative(const std::string& base_file,
                            const std::string& path) {
  if (IsAbsolute(path)) return path;
  return DirectoryName(base_file) + "/" + path;
}

std::string ResolvePackageUri(const std::string& value) {
  const std::string prefix = "package://";
  if (value.compare(0, prefix.size(), prefix) != 0) return value;
  const std::string remainder = value.substr(prefix.size());
  const size_t slash = remainder.find('/');
  if (slash == std::string::npos) return value;
  const std::string package_path = ros::package::getPath(remainder.substr(0, slash));
  if (package_path.empty()) return value;
  return package_path + "/" + remainder.substr(slash + 1);
}

template <typename T>
void ReadIf(const YAML::Node& node, const char* key, T* output) {
  if (node && node[key] && !node[key].IsNull()) *output = node[key].as<T>();
}

std::string TimestampUtc() {
  const std::time_t now = std::time(nullptr);
  std::tm utc;
  gmtime_r(&now, &utc);
  std::ostringstream result;
  result << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return result.str();
}

std::string ReadFirstMatchingLine(const std::string& path,
                                  const std::string& prefix) {
  std::ifstream input(path);
  std::string line;
  while (std::getline(input, line)) {
    if (line.compare(0, prefix.size(), prefix) == 0) {
      const size_t colon = line.find(':');
      if (colon != std::string::npos) {
        const size_t first = line.find_first_not_of(" \t", colon + 1);
        return first == std::string::npos ? std::string() : line.substr(first);
      }
    }
  }
  return std::string();
}

uint64_t ReadMemoryTotal() {
  std::ifstream input("/proc/meminfo");
  std::string key;
  uint64_t value = 0;
  std::string unit;
  while (input >> key >> value >> unit) {
    if (key == "MemTotal:") return value * 1024ULL;
  }
  return 0;
}

std::string RunCommand(const std::string& command) {
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) return std::string();
  char buffer[256];
  std::string output;
  while (fgets(buffer, sizeof(buffer), pipe)) output += buffer;
  pclose(pipe);
  while (!output.empty() && (output.back() == '\n' || output.back() == '\r')) {
    output.pop_back();
  }
  return output;
}

double InscribedRadius(const std::vector<Eigen::Vector2d>& polygon) {
  double result = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < polygon.size(); ++i) {
    const Eigen::Vector2d& a = polygon[i];
    const Eigen::Vector2d& b = polygon[(i + 1) % polygon.size()];
    const Eigen::Vector2d edge = b - a;
    const double denom = edge.squaredNorm();
    double t = denom > 1e-12 ? -a.dot(edge) / denom : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    result = std::min(result, (a + t * edge).norm());
  }
  return result;
}

RobotGeometry LoadRobot(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  RobotGeometry robot;
  robot.name = root["name"].as<std::string>();
  for (const YAML::Node& point : root["footprint"]) {
    robot.polygon.emplace_back(point[0].as<double>(), point[1].as<double>());
  }
  robot.mesh_path = ResolvePackageUri(root["mesh_path"].as<std::string>());
  if (!IsAbsolute(robot.mesh_path)) robot.mesh_path = ResolveRelative(path, robot.mesh_path);
  ReadIf(root, "safety_margin", &robot.safety_margin);
  ReadIf(root, "inscribed_radius", &robot.inscribed_radius);
  if (!std::isfinite(robot.inscribed_radius)) {
    robot.inscribed_radius = InscribedRadius(robot.polygon);
  }
  return robot;
}

nav_msgs::OccupancyGrid LoadMap(const std::string& map_yaml,
                                std::string* image_path,
                                std::string* map_hash) {
  const YAML::Node root = YAML::LoadFile(map_yaml);
  const std::string image = ResolveRelative(
      map_yaml, root["image"].as<std::string>());
  const double resolution = root["resolution"].as<double>();
  const int negate = root["negate"] ? root["negate"].as<int>() : 0;
  const double occupied = root["occupied_thresh"]
                              ? root["occupied_thresh"].as<double>() : 0.65;
  const double free = root["free_thresh"]
                          ? root["free_thresh"].as<double>() : 0.196;
  double origin[3] = {0.0, 0.0, 0.0};
  const YAML::Node origin_node = root["origin"];
  for (size_t i = 0; i < 3 && origin_node && i < origin_node.size(); ++i) {
    origin[i] = origin_node[i].as<double>();
  }
  MapMode mode = TRINARY;
  if (root["mode"]) {
    const std::string mode_name = root["mode"].as<std::string>();
    if (mode_name == "scale") mode = SCALE;
    if (mode_name == "raw") mode = RAW;
  }
  nav_msgs::GetMap::Response response;
  map_server::loadMapFromFile(&response, image.c_str(), resolution, negate != 0,
                              occupied, free, origin, mode);
  if (image_path) *image_path = image;
  if (map_hash) {
    *map_hash = root["map_hash"] ? root["map_hash"].as<std::string>()
                                  : ResultWriter::Sha256File(image);
  }
  return response.map;
}

MethodConfig LoadMethod(const YAML::Node& config,
                        const std::string& config_path,
                        const std::string& method_name) {
  std::string methods_path;
  if (config["paths"] && config["paths"]["methods_config"]) {
    methods_path = ResolvePackageUri(
        config["paths"]["methods_config"].as<std::string>());
    if (!IsAbsolute(methods_path)) {
      methods_path = ResolveRelative(config_path, methods_path);
    }
  }
  YAML::Node method;
  if (!methods_path.empty()) {
    const YAML::Node methods = YAML::LoadFile(methods_path);
    method = methods["methods"] ? methods["methods"][method_name]
                                : methods[method_name];
  }
  if (!method) throw std::runtime_error("unknown method: " + method_name);
  MethodConfig result;
  result.name = method_name;
  ReadIf(method, "force_all_low", &result.force_all_low);
  ReadIf(method, "force_all_strict", &result.force_all_strict);
  ReadIf(method, "force_full_svsdf", &result.force_full_svsdf);
  ReadIf(method, "hybrid_astar_only", &result.hybrid_astar_only);
  const YAML::Node recovery = method["recovery"];
  ReadIf(recovery, "enabled", &result.recovery.enabled);
  ReadIf(recovery, "local_upgrade", &result.recovery.local_upgrade);
  ReadIf(recovery, "strict_local_patch", &result.recovery.strict_local_patch);
  ReadIf(recovery, "swept_astar", &result.recovery.swept_astar);
  ReadIf(recovery, "hybrid_astar", &result.recovery.hybrid_astar);
  ReadIf(recovery, "candidate_switch", &result.recovery.candidate_switch);
  ReadIf(recovery, "tail_refinement", &result.recovery.tail_refinement);
  ReadIf(recovery, "hybrid_frontend_fallback",
         &result.recovery.hybrid_frontend_fallback);
  ReadIf(recovery, "post_incumbent_svsdf_budget",
         &result.recovery.post_incumbent_svsdf_budget);
  ReadIf(recovery, "tail_refinement_svsdf_budget",
         &result.recovery.tail_refinement_svsdf_budget);
  return result;
}

ValidationConfig LoadValidation(const YAML::Node& root) {
  ValidationConfig result;
  const YAML::Node node = root["validation"];
  ReadIf(node, "time_step_sec", &result.time_step_sec);
  ReadIf(node, "max_translation_step_m", &result.max_translation_step_m);
  ReadIf(node, "max_yaw_step_rad", &result.max_yaw_step_rad);
  ReadIf(node, "footprint_boundary_step_m", &result.footprint_boundary_step_m);
  ReadIf(node, "footprint_interior_step_m", &result.footprint_interior_step_m);
  ReadIf(node, "collision_clearance_m", &result.collision_clearance_m);
  ReadIf(node, "require_dynamics", &result.require_dynamics);
  ReadIf(node, "require_boundary_continuity", &result.require_boundary_continuity);
  ReadIf(node, "require_endpoint_match", &result.require_endpoint_match);
  ReadIf(node, "max_endpoint_position_error_m",
         &result.max_endpoint_position_error_m);
  ReadIf(node, "max_endpoint_yaw_error_rad",
         &result.max_endpoint_yaw_error_rad);
  return result;
}

GeometryCheckConfig LoadGeometryCheck(const YAML::Node& root) {
  GeometryCheckConfig result;
  const YAML::Node node = root["geometry_check"];
  ReadIf(node, "enabled", &result.enabled);
  ReadIf(node, "max_centroid_error_m", &result.max_centroid_error_m);
  ReadIf(node, "max_hausdorff_distance_m", &result.max_hausdorff_distance_m);
  ReadIf(node, "outline_sample_step_m", &result.outline_sample_step_m);
  ReadIf(node, "max_relative_area_error", &result.max_relative_area_error);
  ReadIf(node, "fail_experiment_on_mismatch", &result.fail_experiment_on_mismatch);
  return result;
}

DynamicsLimits LoadLimits(const YAML::Node& root,
                          const DynamicsLimits& fallback) {
  DynamicsLimits result = fallback;
  const YAML::Node node = root["limits"];
  ReadIf(node, "max_linear_velocity", &result.max_linear_velocity);
  ReadIf(node, "max_linear_acceleration", &result.max_linear_acceleration);
  ReadIf(node, "max_angular_velocity", &result.max_angular_velocity);
  ReadIf(node, "max_angular_acceleration", &result.max_angular_acceleration);
  return result;
}

bool FootprintPoseValid(const nav_msgs::OccupancyGrid& occupancy,
                        const std::vector<Eigen::Vector2d>& polygon,
                        const geometry_msgs::Pose2D& pose, double step) {
  isweep_planner::GridMap map;
  nav_msgs::OccupancyGrid::Ptr pointer(new nav_msgs::OccupancyGrid(occupancy));
  map.fromOccupancyGrid(pointer);
  isweep_planner::FootprintModel footprint;
  footprint.setPolygon(polygon);
  const auto& samples = footprint.denseBodySamples(step, step);
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  for (const Eigen::Vector2d& point : samples) {
    const double x = pose.x + c * point.x() - s * point.y();
    const double y = pose.y + s * point.x() + c * point.y();
    const isweep_planner::GridIndex cell = map.worldToGrid(x, y);
    if (!map.isInside(cell.x, cell.y) || map.isRawOccupied(cell.x, cell.y)) return false;
  }
  return true;
}

geometry_msgs::Pose2D ParsePose(const YAML::Node& node) {
  geometry_msgs::Pose2D pose;
  pose.x = node[0].as<double>();
  pose.y = node[1].as<double>();
  pose.theta = node[2].as<double>();
  return pose;
}

bool SamePose(const geometry_msgs::Pose2D& first,
              const geometry_msgs::Pose2D& second) {
  return std::hypot(first.x - second.x, first.y - second.y) < 1e-9 &&
         std::abs(isweep_planner::normalizeAngle(first.theta - second.theta)) < 1e-9;
}

}  // namespace

bool ExperimentRunner::Run(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                           const RunnerArguments& arguments,
                           std::string* error) const {
  const std::chrono::steady_clock::time_point run_begin =
      std::chrono::steady_clock::now();
  const auto elapsed_run_ms = [&]() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - run_begin).count();
  };
  PlanningResult result;
  try {
    const YAML::Node config = YAML::LoadFile(arguments.config_path);
    const YAML::Node tasks = YAML::LoadFile(arguments.tasks_path);
    const MethodConfig method =
        LoadMethod(config, arguments.config_path, arguments.method_name);
    const RobotGeometry robot = LoadRobot(arguments.robot_yaml);

    const std::chrono::steady_clock::time_point map_begin =
        std::chrono::steady_clock::now();
    std::string map_image;
    std::string map_hash;
    const nav_msgs::OccupancyGrid map =
        LoadMap(arguments.map_yaml, &map_image, &map_hash);
    const double map_load_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - map_begin).count();

    YAML::Node selected_task;
    std::set<std::string> ids;
    std::set<std::string> pose_keys;
    bool duplicate_task = false;
    for (const YAML::Node& task : tasks["tasks"]) {
      const std::string id = task["id"].as<std::string>();
      if (!ids.insert(id).second) duplicate_task = true;
      const geometry_msgs::Pose2D start = ParsePose(task["start"]);
      const geometry_msgs::Pose2D goal = ParsePose(task["goal"]);
      std::ostringstream key;
      key << std::setprecision(17) << start.x << ',' << start.y << ',' << start.theta
          << ',' << goal.x << ',' << goal.y << ',' << goal.theta;
      if (!pose_keys.insert(key.str()).second) duplicate_task = true;
      if (id == arguments.task_id) selected_task = task;
    }
    if (duplicate_task) throw std::runtime_error("task file contains duplicate id or pose pair");
    if (!selected_task) throw std::runtime_error("task id not found: " + arguments.task_id);

    PlanningRequest request;
    request.run_id = arguments.run_id;
    request.configuration_hash = arguments.configuration_hash;
    request.task_id = arguments.task_id;
    request.method_name = arguments.method_name;
    request.map_name = tasks["map"] ? tasks["map"].as<std::string>()
                                     : arguments.map_yaml;
    request.map_hash = map_hash;
    request.robot_shape_name = robot.name;
    request.start = ParsePose(selected_task["start"]);
    request.goal = ParsePose(selected_task["goal"]);
    request.random_seed = arguments.seed;
    request.total_time_budget_sec = config["experiment"]["total_time_budget_sec"].as<double>();
    const YAML::Node risk_ground_truth = config["risk_ground_truth"];
    ReadIf(risk_ground_truth, "enabled", &request.risk_ground_truth_enabled);
    ReadIf(risk_ground_truth, "run_low_solver_for_all_segments",
           &request.risk_ground_truth_run_low);
    ReadIf(risk_ground_truth, "run_medium_solver_for_all_segments",
           &request.risk_ground_truth_run_medium);
    ReadIf(risk_ground_truth, "run_strict_solver_for_all_segments",
           &request.risk_ground_truth_run_strict);

    if (!std::isfinite(request.start.theta) || !std::isfinite(request.goal.theta)) {
      throw std::runtime_error("start or goal yaw is not finite");
    }
    if (SamePose(request.start, request.goal)) {
      throw std::runtime_error("start and goal are identical");
    }
    const ValidationConfig validation_config = LoadValidation(config);
    if (!FootprintPoseValid(map, robot.polygon, request.start,
                            validation_config.footprint_interior_step_m)) {
      throw std::runtime_error("start footprint is outside map or in collision");
    }
    if (!FootprintPoseValid(map, robot.polygon, request.goal,
                            validation_config.footprint_interior_step_m)) {
      throw std::runtime_error("goal footprint is outside map or in collision");
    }

    const GeometryCheckConfig geometry_config = LoadGeometryCheck(config);
    const std::chrono::steady_clock::time_point geometry_begin =
        std::chrono::steady_clock::now();
    const GeometryConsistencyResult geometry =
        GeometryConsistencyChecker().Check(robot.polygon, robot.mesh_path,
                                           geometry_config);
    const double geometry_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - geometry_begin).count();

    if (!geometry.consistency_passed && geometry_config.fail_experiment_on_mismatch) {
      result.status = PlannerStatus::CONFIGURATION_ERROR;
      result.failure_stage = "geometry_consistency";
      result.failure_reason = geometry.failure_reason;
      result.metrics.run_id = request.run_id;
      result.metrics.configuration_hash = request.configuration_hash;
      result.metrics.task_id = request.task_id;
      result.metrics.method_name = request.method_name;
      result.metrics.map_name = request.map_name;
      result.metrics.map_hash = request.map_hash;
      result.metrics.map_resolution = map.info.resolution;
      result.metrics.robot_shape = request.robot_shape_name;
      result.metrics.start = request.start;
      result.metrics.goal = request.goal;
      result.metrics.random_seed = request.random_seed;
      result.metrics.total_time_budget_sec = request.total_time_budget_sec;
      result.metrics.planner_status = result.status;
      result.metrics.failure_stage = result.failure_stage;
      result.metrics.failure_reason = result.failure_reason;
      result.metrics.geometry = geometry;
      result.metrics.geometry_consistency_passed = false;
      result.metrics.t_map_load_ms = map_load_ms;
      result.metrics.t_geometry_precompute_ms = geometry_ms;
      result.metrics.t_total_ms = elapsed_run_ms();
      result.metrics.polygon_id = ResultWriter::Sha256File(arguments.robot_yaml);
      result.metrics.mesh_id = ResultWriter::Sha256File(robot.mesh_path);
      result.metrics.timestamp = TimestampUtc();
      return ResultWriter().WriteRunDirectory(arguments.output_directory, result,
                                              error);
    }

    std::vector<double> flat_footprint;
    for (const Eigen::Vector2d& point : robot.polygon) {
      flat_footprint.push_back(point.x());
      flat_footprint.push_back(point.y());
    }
    pnh.setParam("footprint", flat_footprint);
    pnh.setParam("inputdata", robot.mesh_path);
    pnh.setParam("inscribed_radius", robot.inscribed_radius);
    if (config["reproducibility"] &&
        config["reproducibility"]["fixed_cpu_threads"]) {
      pnh.setParam("threads_num",
                   config["reproducibility"]["fixed_cpu_threads"].as<int>());
    }

    PlannerAdapter adapter;
    std::string adapter_error;
    if (!adapter.Initialize(nh, pnh, map, method, validation_config,
                            &adapter_error)) {
      throw std::runtime_error(adapter_error);
    }
    const DynamicsLimits limits =
        LoadLimits(config, adapter.plannerDynamicsLimits());
    adapter.SetExternalValidationLimits(limits);
    result = adapter.Plan(request);
    result.metrics.t_map_load_ms = map_load_ms;
    result.metrics.t_esdf_ms = adapter.runtime().last_esdf_build_ms();
    result.metrics.t_geometry_precompute_ms =
        geometry_ms + adapter.runtime().last_initialization_ms() +
        adapter.runtime().last_map_precompute_ms();
    result.metrics.geometry = geometry;
    result.metrics.geometry_consistency_passed = geometry.consistency_passed;
    result.metrics.polygon_id = ResultWriter::Sha256File(arguments.robot_yaml);
    result.metrics.mesh_id = ResultWriter::Sha256File(robot.mesh_path);
    const isweep_planner::SE2State requested_start(
        request.start.x, request.start.y, request.start.theta);
    const isweep_planner::SE2State requested_goal(
        request.goal.x, request.goal.y, request.goal.theta);
    if (result.planner_returned) {
      const std::chrono::steady_clock::time_point validation_begin =
          std::chrono::steady_clock::now();
      const ValidationResult validation = TrajectoryValidator().Validate(
          map, adapter.runtime().grid_map(), adapter.runtime().footprint(),
          result.trajectory, limits, validation_config, requested_start,
          requested_goal);
      result.metrics.t_external_validation_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - validation_begin).count();
      MetricsCollector::ApplyValidation(validation, &result);
    }

    for (RiskGroundTruthMetric& metric : result.risk_ground_truth_metrics) {
      if (metric.low_solver_attempted &&
          !metric.low_trajectory.empty()) {
        metric.low_validation = TrajectoryValidator().Validate(
            map, adapter.runtime().grid_map(), adapter.runtime().footprint(),
            metric.low_trajectory, limits, validation_config,
            metric.expected_start, metric.expected_goal);
        metric.low_endpoints_match = metric.low_validation.endpoints_match;
        metric.low_executable =
            metric.low_endpoints_match &&
            metric.low_validation.external_footprint_valid &&
            metric.low_validation.dynamics_valid &&
            metric.low_validation.boundary_continuous;
      }
      if (metric.medium_solver_attempted &&
          !metric.medium_trajectory.empty()) {
        metric.medium_validation = TrajectoryValidator().Validate(
            map, adapter.runtime().grid_map(), adapter.runtime().footprint(),
            metric.medium_trajectory, limits, validation_config,
            metric.expected_start, metric.expected_goal);
        metric.medium_endpoints_match =
            metric.medium_validation.endpoints_match;
        metric.medium_executable =
            metric.medium_endpoints_match &&
            metric.medium_validation.external_footprint_valid &&
            metric.medium_validation.dynamics_valid &&
            metric.medium_validation.boundary_continuous;
      }
      if (metric.strict_solver_attempted &&
          !metric.strict_trajectory.empty()) {
        metric.strict_validation = TrajectoryValidator().Validate(
            map, adapter.runtime().grid_map(), adapter.runtime().footprint(),
            metric.strict_trajectory, limits, validation_config,
            metric.expected_start, metric.expected_goal);
        metric.strict_endpoints_match =
            metric.strict_validation.endpoints_match;
        metric.strict_executable =
            metric.strict_endpoints_match &&
            metric.strict_validation.external_footprint_valid &&
            metric.strict_validation.dynamics_valid &&
            metric.strict_validation.boundary_continuous;
      }

      FinalizeRiskGroundTruthLabel(&metric);
    }
    result.metrics.risk_ground_truth_trial_count =
        static_cast<int>(result.risk_ground_truth_metrics.size());
    for (const RiskGroundTruthMetric& metric :
         result.risk_ground_truth_metrics) {
      if (metric.label_available) {
        ++result.metrics.risk_ground_truth_labeled_count;
      }
      if (metric.l2_required) {
        ++result.metrics.risk_ground_truth_l2_required_count;
      }
      if (metric.l3_required) {
        ++result.metrics.risk_ground_truth_l3_required_count;
      }
      if (metric.strict_required) {
        ++result.metrics.risk_ground_truth_strict_required_count;
      }
    }

    result.metrics.timestamp = TimestampUtc();
    result.metrics.compiler_version = __VERSION__;
    const char* ros_distro = std::getenv("ROS_DISTRO");
    result.metrics.ros_distro = ros_distro ? ros_distro : "";
    result.metrics.cpu_model = ReadFirstMatchingLine("/proc/cpuinfo", "model name");
    result.metrics.memory_total = ReadMemoryTotal();
    const char* omp_threads = std::getenv("OMP_NUM_THREADS");
    result.metrics.thread_count = omp_threads ? std::max(1, std::atoi(omp_threads)) : 1;
#ifdef PLANNER_EXPERIMENTS_BUILD_TYPE
    result.metrics.build_type = PLANNER_EXPERIMENTS_BUILD_TYPE;
#endif
    const std::string package_path = ros::package::getPath("isweep_planner");
    const std::string repository = DirectoryName(package_path);
    result.metrics.git_commit = RunCommand("git -C '" + repository + "' rev-parse HEAD 2>/dev/null");
    result.metrics.git_dirty =
        !RunCommand("git -C '" + repository + "' status --porcelain 2>/dev/null").empty();
    result.metrics.t_total_ms = elapsed_run_ms();

    return ResultWriter().WriteRunDirectory(arguments.output_directory, result,
                                            error);
  } catch (const std::exception& exception) {
    result.status = PlannerStatus::CONFIGURATION_ERROR;
    result.failure_stage = "input_validation";
    result.failure_reason = exception.what();
    result.metrics.run_id = arguments.run_id;
    result.metrics.configuration_hash = arguments.configuration_hash;
    result.metrics.task_id = arguments.task_id;
    result.metrics.method_name = arguments.method_name;
    result.metrics.random_seed = arguments.seed;
    result.metrics.planner_status = result.status;
    result.metrics.failure_stage = result.failure_stage;
    result.metrics.failure_reason = result.failure_reason;
    result.metrics.timestamp = TimestampUtc();
    result.metrics.t_total_ms = elapsed_run_ms();
    std::string write_error;
    const bool written = ResultWriter().WriteRunDirectory(
        arguments.output_directory, result, &write_error);
    if (error) {
      *error = exception.what();
      if (!written) *error += "; also failed to write result: " + write_error;
    }
    return written;
  }
}

}  // namespace planner_experiments
