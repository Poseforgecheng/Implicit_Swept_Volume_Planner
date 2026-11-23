#!/usr/bin/env python3
"""Run IR-SIM as the authoritative plant for the ROS1 iSweep planner."""

import json
import math
import os
import tempfile
import threading
import time

import numpy as np
from PIL import Image
import yaml

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from irsim_bridge.core import AckermannLimits
from irsim_bridge.core import build_irsim_world_config
from irsim_bridge.core import load_ros_map
from irsim_bridge.core import quaternion_from_yaw
from irsim_bridge.core import sanitize_ackermann_command
from irsim_bridge.core import steering_angle_to_yaw_rate
from irsim_bridge.core import yaw_rate_to_steering_angle
from irsim_bridge.core import yaw_from_quaternion


def _pose_yaw(pose):
    quaternion = pose.orientation
    return yaw_from_quaternion(
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    )


def _set_pose_orientation(pose, yaw):
    x, y, z, w = quaternion_from_yaw(yaw)
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w


def _flat_array(value):
    return np.asarray(value, dtype=float).reshape(-1)


class IrSimRosBridge:
    def __init__(self):
        self._lock = threading.RLock()
        self._shutdown = False
        self._load_parameters()

        if not self.display_irsim:
            os.environ.setdefault("MPLBACKEND", "Agg")

        try:
            import irsim
        except ImportError as error:
            raise RuntimeError(
                "IR-SIM is unavailable in this Python interpreter. Run "
                "irsim_bridge/scripts/setup_irsim_env.sh and launch with the "
                "generated .venv Python."
            ) from error

        self._irsim = irsim
        self._temporary_directory = tempfile.TemporaryDirectory(
            prefix="isweep_irsim_"
        )
        self._prepare_environment_files()
        self.env = self._irsim.make(
            self.generated_world_file,
            display=self.display_irsim,
            disable_all_plot=not self.display_irsim,
            log_level=self.irsim_log_level,
            log_file=None,
            seed=self.seed,
        )
        self.robot = self.env.robot

        self.latest_command = (0.0, 0.0)
        self.applied_command = (0.0, 0.0)
        self.applied_steering_angle = 0.0
        self.latest_command_wall_time = None
        self.simulation_steps = 0
        self.path_poses = []
        self.last_status_wall_time = 0.0
        self.last_slow_warning_wall_time = 0.0
        self.goal = list(self.default_goal)

        self._setup_ros_interfaces()
        self._publish_static_messages()
        self._publish_state(force_path=True)
        rospy.on_shutdown(self.shutdown)
        # The navigation adapter needs one localization sample before it can
        # derive the planner start pose. Republish the latched default goal
        # after startup so node scheduling cannot leave it with goal-only data.
        self.startup_goal_timer = rospy.Timer(
            rospy.Duration(1.0), self._republish_startup_goal, oneshot=True
        )

        rospy.loginfo(
            "IR-SIM bridge ready: map=%s (%dx%d @ %.3fm), start=(%.2f, %.2f, %.2f), "
            "goal=(%.2f, %.2f, %.2f), dt=%.3fs",
            self.map_data.yaml_path,
            self.map_data.width,
            self.map_data.height,
            self.map_data.resolution,
            self.default_start[0],
            self.default_start[1],
            self.default_start[2],
            self.default_goal[0],
            self.default_goal[1],
            self.default_goal[2],
            self.step_time,
        )

    def _param(self, name, default):
        return rospy.get_param("~" + name, default)

    def _republish_startup_goal(self, _event):
        if not self._shutdown:
            self.goal_pub.publish(self._make_goal(self.goal))

    def _load_parameters(self):
        map_yaml = self._param("map_yaml", "")
        if not map_yaml:
            raise ValueError("~map_yaml is required")
        self.unknown_is_occupied = bool(
            self._param("unknown_is_occupied", True)
        )
        self.map_data = load_ros_map(map_yaml, self.unknown_is_occupied)

        self.frame_id = str(self._param("frame_id", "map"))
        self.base_frame = str(self._param("base_frame", "base_link"))
        self.scan_frame = str(self._param("scan_frame", "base_scan"))
        self.map_topic = str(self._param("map_topic", "/map"))
        self.cmd_topic = str(self._param("cmd_topic", "/irsim/cmd_vel"))
        self.start_topic = str(self._param("start_topic", "/initialpose"))
        self.goal_topic = str(
            self._param("goal_topic", "/move_base_simple/goal")
        )
        self.current_pose_topic = str(
            self._param(
                "current_pose_topic", "/isweep_planner/current_pose"
            )
        )
        self.current_velocity_topic = str(
            self._param(
                "current_velocity_topic", "/isweep_planner/current_velocity"
            )
        )
        self.odom_topic = str(self._param("odom_topic", "/irsim/odom"))
        self.localization_topic = str(
            self._param("localization_topic", "/irsim/localization")
        )
        self.scan_topic = str(self._param("scan_topic", "/irsim/scan"))
        self.actual_path_topic = str(
            self._param("actual_path_topic", "/irsim/actual_path")
        )
        self.marker_topic = str(self._param("marker_topic", "/irsim/markers"))
        self.status_topic = str(self._param("status_topic", "/irsim/status"))

        self.default_start = [
            float(self._param("start/x", 1.0)),
            float(self._param("start/y", 1.0)),
            float(self._param("start/yaw", 0.0)),
        ]
        self.default_goal = [
            float(self._param("goal/x", 9.0)),
            float(self._param("goal/y", 9.0)),
            float(self._param("goal/yaw", 0.0)),
        ]
        self.step_time = max(0.01, float(self._param("step_time", 0.10)))
        self.display_irsim = bool(self._param("display_irsim", False))
        self.irsim_log_level = str(self._param("irsim_log_level", "ERROR"))
        self.seed = int(self._param("seed", 7))
        self.planner_method = str(
            self._param("planner_method", "current_full")
        )
        self.command_timeout = max(
            self.step_time, float(self._param("command_timeout", 0.50))
        )
        self.stop_on_collision = bool(self._param("stop_on_collision", True))
        self.stop_on_arrival = bool(self._param("stop_on_arrival", True))
        self.accept_rviz_start = bool(self._param("accept_rviz_start", True))
        self.max_path_points = max(100, int(self._param("max_path_points", 10000)))
        self.path_min_distance = max(
            0.0, float(self._param("path_min_distance", 0.01))
        )
        self.goal_position_tolerance = max(
            0.0, float(self._param("goal_position_tolerance", 0.10))
        )
        self.goal_yaw_tolerance = max(
            0.0, float(self._param("goal_yaw_tolerance", 0.20))
        )

        self.robot_length = float(self._param("robot/length", 0.45))
        self.robot_width = float(self._param("robot/width", 0.36))
        self.wheelbase = float(self._param("robot/wheelbase", 0.30))
        self.max_linear_speed = float(
            self._param("robot/max_linear_speed", 0.45)
        )
        self.max_yaw_rate = float(self._param("robot/max_yaw_rate", 1.0))
        self.linear_acceleration = float(
            self._param("robot/linear_acceleration", 0.50)
        )
        self.max_steering_rate = float(
            self._param(
                "robot/max_steering_rate",
                self._param("robot/yaw_acceleration", 1.0),
            )
        )
        self.min_turn_radius = float(self._param("robot/min_turn_radius", 0.30))
        if self.wheelbase <= 0.0 or self.min_turn_radius <= 0.0:
            raise ValueError("wheelbase and minimum turn radius must be positive")
        self.max_steering_angle = math.atan(
            self.wheelbase / self.min_turn_radius
        )
        self.limits = AckermannLimits(
            max_linear_speed=self.max_linear_speed,
            max_yaw_rate=self.max_yaw_rate,
            min_turn_radius=self.min_turn_radius,
            linear_deadband=float(self._param("robot/linear_deadband", 0.01)),
            angular_deadband=float(self._param("robot/angular_deadband", 0.01)),
        )

        self.lidar_range_min = float(self._param("lidar/range_min", 0.05))
        self.lidar_range_max = float(self._param("lidar/range_max", 8.0))
        self.lidar_angle_range = float(
            self._param("lidar/angle_range", 2.0 * math.pi)
        )
        self.lidar_beams = max(1, int(self._param("lidar/beams", 180)))
        self.lidar_noise = bool(self._param("lidar/noise", False))

    def _prepare_environment_files(self):
        directory = self._temporary_directory.name
        binary_map_path = os.path.join(directory, "collision_truth.png")
        Image.fromarray(self.map_data.binary_image).save(binary_map_path)
        config = build_irsim_world_config(
            map_data=self.map_data,
            binary_map_path=binary_map_path,
            start=self.default_start,
            goal=self.default_goal,
            step_time=self.step_time,
            robot_length=self.robot_length,
            robot_width=self.robot_width,
            wheelbase=self.wheelbase,
            min_turn_radius=self.min_turn_radius,
            max_linear_speed=self.max_linear_speed,
            max_steering_rate=self.max_steering_rate,
            linear_acceleration=self.linear_acceleration,
            goal_tolerance=self.goal_position_tolerance,
            lidar_range_min=self.lidar_range_min,
            lidar_range_max=self.lidar_range_max,
            lidar_angle_range=self.lidar_angle_range,
            lidar_beams=self.lidar_beams,
            lidar_noise=self.lidar_noise,
            seed=self.seed,
        )
        self.generated_world_file = os.path.join(directory, "world.yaml")
        with open(self.generated_world_file, "w", encoding="utf-8") as stream:
            yaml.safe_dump(config, stream, sort_keys=False)
        rospy.set_param("~generated_world_file", self.generated_world_file)
        rospy.set_param("~map_checksum", self.map_data.checksum)

    def _setup_ros_interfaces(self):
        self.map_pub = rospy.Publisher(
            self.map_topic, OccupancyGrid, queue_size=1, latch=True
        )
        self.initial_pose_pub = rospy.Publisher(
            self.start_topic, PoseWithCovarianceStamped, queue_size=1, latch=True
        )
        self.goal_pub = rospy.Publisher(
            self.goal_topic, PoseStamped, queue_size=1, latch=True
        )
        self.current_pose_pub = rospy.Publisher(
            self.current_pose_topic, PoseStamped, queue_size=10
        )
        self.current_velocity_pub = rospy.Publisher(
            self.current_velocity_topic, TwistStamped, queue_size=10
        )
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.localization_pub = rospy.Publisher(
            self.localization_topic, Odometry, queue_size=10
        )
        self.scan_pub = rospy.Publisher(self.scan_topic, LaserScan, queue_size=10)
        self.actual_path_pub = rospy.Publisher(
            self.actual_path_topic, Path, queue_size=1, latch=True
        )
        self.marker_pub = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=10, latch=True
        )
        self.collision_pub = rospy.Publisher(
            "/irsim/collision", Bool, queue_size=1, latch=True
        )
        self.arrived_pub = rospy.Publisher(
            "/irsim/arrived", Bool, queue_size=1, latch=True
        )
        self.tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=10)
        self.tf_static_pub = rospy.Publisher(
            "/tf_static", TFMessage, queue_size=1, latch=True
        )
        self.metadata_pub = rospy.Publisher(
            "/irsim/metadata", String, queue_size=1, latch=True
        )

        self.cmd_sub = rospy.Subscriber(
            self.cmd_topic, Twist, self._command_callback, queue_size=20
        )
        self.goal_sub = rospy.Subscriber(
            self.goal_topic, PoseStamped, self._goal_callback, queue_size=5
        )
        self.start_sub = rospy.Subscriber(
            self.start_topic,
            PoseWithCovarianceStamped,
            self._start_callback,
            queue_size=5,
        )
        self.reset_service = rospy.Service(
            "/irsim/reset", Trigger, self._reset_service
        )

    def _publish_static_messages(self):
        self.map_pub.publish(self._make_map_message())
        self.tf_static_pub.publish(
            TFMessage(
                transforms=[
                    self._make_transform(
                        self.base_frame, self.scan_frame, 0.0, 0.0, 0.0
                    )
                ]
            )
        )
        self.metadata_pub.publish(
            String(
                data=json.dumps(
                    {
                        "simulator": "IR-SIM",
                        "map_yaml": self.map_data.yaml_path,
                        "map_checksum": self.map_data.checksum,
                        "unknown_is_occupied": self.unknown_is_occupied,
                        "seed": self.seed,
                        "planner_method": self.planner_method,
                        "step_time_s": self.step_time,
                        "robot": {
                            "kinematics": "acker-steer",
                            "length_m": self.robot_length,
                            "width_m": self.robot_width,
                            "wheelbase_m": self.wheelbase,
                            "max_steering_angle_rad": self.max_steering_angle,
                        },
                    },
                    sort_keys=True,
                )
            )
        )
        self.initial_pose_pub.publish(self._make_initial_pose(self.default_start))

    def _make_map_message(self):
        message = OccupancyGrid()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame_id
        message.info.map_load_time = message.header.stamp
        message.info.resolution = self.map_data.resolution
        message.info.width = self.map_data.width
        message.info.height = self.map_data.height
        message.info.origin.position.x = self.map_data.origin[0]
        message.info.origin.position.y = self.map_data.origin[1]
        _set_pose_orientation(message.info.origin, self.map_data.origin[2])
        message.data = self.map_data.occupancy.reshape(-1).astype(int).tolist()
        return message

    def _make_initial_pose(self, values):
        message = PoseWithCovarianceStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame_id
        message.pose.pose.position.x = values[0]
        message.pose.pose.position.y = values[1]
        _set_pose_orientation(message.pose.pose, values[2])
        message.pose.covariance[0] = 1.0e-6
        message.pose.covariance[7] = 1.0e-6
        message.pose.covariance[35] = 1.0e-6
        return message

    def _make_goal(self, values):
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame_id
        message.pose.position.x = values[0]
        message.pose.position.y = values[1]
        _set_pose_orientation(message.pose, values[2])
        return message

    def _make_transform(self, parent, child, x, y, yaw):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform

    def _command_callback(self, message):
        with self._lock:
            self.latest_command = (float(message.linear.x), float(message.angular.z))
            self.latest_command_wall_time = time.monotonic()

    def _goal_callback(self, message):
        if message.header.frame_id and message.header.frame_id != self.frame_id:
            rospy.logwarn(
                "Ignoring goal in frame %s; expected %s",
                message.header.frame_id,
                self.frame_id,
            )
            return
        goal = [
            float(message.pose.position.x),
            float(message.pose.position.y),
            _pose_yaw(message.pose),
        ]
        with self._lock:
            self.goal = goal
            if hasattr(self.robot, "set_goal"):
                self.robot.set_goal(goal)
            if hasattr(self.robot, "arrive_flag"):
                self.robot.arrive_flag = False
            if hasattr(self.robot, "stop_flag") and not bool(
                getattr(self.robot, "collision", False)
            ):
                self.robot.stop_flag = False
        rospy.loginfo("IR-SIM goal updated: (%.2f, %.2f, %.2f)", *goal)

    def _start_callback(self, message):
        if not self.accept_rviz_start:
            return
        if message.header.frame_id and message.header.frame_id != self.frame_id:
            rospy.logwarn(
                "Ignoring initial pose in frame %s; expected %s",
                message.header.frame_id,
                self.frame_id,
            )
            return
        start = [
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            _pose_yaw(message.pose.pose),
        ]
        self._reset_robot(start)
        rospy.loginfo("IR-SIM start reset: (%.2f, %.2f, %.2f)", *start)

    def _reset_service(self, _request):
        self._reset_robot(self.default_start)
        self.initial_pose_pub.publish(self._make_initial_pose(self.default_start))
        self.goal_pub.publish(self._make_goal(self.goal))
        return TriggerResponse(success=True, message="IR-SIM robot reset")

    def _reset_robot(self, start):
        with self._lock:
            state_dimension = _flat_array(self.robot.state).size
            state = [float(start[0]), float(start[1]), float(start[2])]
            state.extend([0.0] * max(0, state_dimension - len(state)))
            self.robot.set_state(state[:state_dimension], init=False)
            if hasattr(self.robot, "set_velocity"):
                velocity_dimension = _flat_array(self.robot.velocity).size
                self.robot.set_velocity([0.0] * velocity_dimension, init=False)
            for flag in ("stop_flag", "arrive_flag", "collision_flag"):
                if hasattr(self.robot, flag):
                    setattr(self.robot, flag, False)
            if hasattr(self.env, "refresh"):
                self.env.refresh()
            self.latest_command = (0.0, 0.0)
            self.applied_command = (0.0, 0.0)
            self.applied_steering_angle = 0.0
            self.latest_command_wall_time = None
            self.path_poses = []
        self._publish_state(force_path=True)

    def _current_command(self, now):
        if self.latest_command_wall_time is None:
            return 0.0, 0.0
        if now - self.latest_command_wall_time > self.command_timeout:
            return 0.0, 0.0
        return sanitize_ackermann_command(
            self.latest_command[0], self.latest_command[1], self.limits
        )

    def _goal_metrics(self, state):
        dx = self.goal[0] - state[0]
        dy = self.goal[1] - state[1]
        position_error = math.hypot(dx, dy)
        yaw_error = math.atan2(
            math.sin(self.goal[2] - state[2]),
            math.cos(self.goal[2] - state[2]),
        )
        arrived = (
            position_error <= self.goal_position_tolerance
            and abs(yaw_error) <= self.goal_yaw_tolerance
        )
        return position_error, yaw_error, arrived

    def step(self):
        with self._lock:
            now = time.monotonic()
            state_before = _flat_array(self.robot.state)
            _, _, arrived_before = self._goal_metrics(state_before)
            collision_before = bool(getattr(self.robot, "collision", False))
            if (self.stop_on_collision and collision_before) or (
                self.stop_on_arrival and arrived_before
            ):
                command = (0.0, 0.0)
            else:
                command = self._current_command(now)
            self.applied_command = command
            self.applied_steering_angle = yaw_rate_to_steering_angle(
                command[0],
                command[1],
                self.wheelbase,
                self.max_steering_angle,
            )
            # IR-SIM 2.10.x normalizes a flat Python list as a list of actions
            # for consecutive object IDs.  A (2, 1) ndarray unambiguously
            # denotes one Ackermann action [linear velocity, steering angle].
            action = np.asarray(
                (command[0], self.applied_steering_angle), dtype=float
            ).reshape(2, 1)
            self.env.step(action, action_id=0)
            self.simulation_steps += 1
            if self.display_irsim:
                self.env.render(0.001)
        self._publish_state()

    def _publish_state(self, force_path=False):
        with self._lock:
            state = _flat_array(self.robot.state)
            if state.size < 3:
                raise RuntimeError("IR-SIM robot state must contain x, y, yaw")
            x, y, yaw = float(state[0]), float(state[1]), float(state[2])
            requested_command = self.latest_command
            commanded_linear, commanded_angular = self.applied_command
            actual_action = _flat_array(self.robot.velocity)
            actual_linear = (
                float(actual_action[0]) if actual_action.size >= 1 else 0.0
            )
            actual_steering = (
                float(actual_action[1]) if actual_action.size >= 2 else 0.0
            )
            actual_angular = steering_angle_to_yaw_rate(
                actual_linear, actual_steering, self.wheelbase
            )
            collision = bool(getattr(self.robot, "collision", False))
            position_error, yaw_error, arrived = self._goal_metrics(state)

        stamp = rospy.Time.now()
        pose_message = PoseStamped()
        pose_message.header.stamp = stamp
        pose_message.header.frame_id = self.frame_id
        pose_message.pose.position.x = x
        pose_message.pose.position.y = y
        _set_pose_orientation(pose_message.pose, yaw)
        self.current_pose_pub.publish(pose_message)

        velocity_message = TwistStamped()
        velocity_message.header = pose_message.header
        velocity_message.twist.linear.x = actual_linear
        velocity_message.twist.angular.z = actual_angular
        self.current_velocity_pub.publish(velocity_message)

        odometry = Odometry()
        odometry.header = pose_message.header
        odometry.child_frame_id = self.base_frame
        odometry.pose.pose = pose_message.pose
        odometry.twist.twist = velocity_message.twist
        self.odom_pub.publish(odometry)
        self.localization_pub.publish(odometry)

        self.tf_pub.publish(
            TFMessage(
                transforms=[
                    self._make_transform(self.frame_id, self.base_frame, x, y, yaw)
                ]
            )
        )
        self._publish_lidar(stamp)
        self._update_actual_path(pose_message, force_path)
        self.marker_pub.publish(
            self._make_markers(stamp, x, y, yaw, collision, arrived)
        )
        self.collision_pub.publish(Bool(data=collision))
        self.arrived_pub.publish(Bool(data=arrived))

        now = time.monotonic()
        if force_path or now - self.last_status_wall_time >= 0.5:
            command_age = (
                None
                if self.latest_command_wall_time is None
                else max(0.0, now - self.latest_command_wall_time)
            )
            status = {
                "step": self.simulation_steps,
                "state": [x, y, yaw],
                "goal": list(self.goal),
                "position_error_m": position_error,
                "yaw_error_rad": yaw_error,
                "collision": collision,
                "arrived": arrived,
                "requested_command": list(requested_command),
                "applied_command": [commanded_linear, commanded_angular],
                "actual_velocity": [actual_linear, actual_angular],
                "steering_angle_rad": actual_steering,
                "command_age_s": command_age,
                "map_checksum": self.map_data.checksum,
                "planner_method": self.planner_method,
            }
            self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))
            self.last_status_wall_time = now

    def _publish_lidar(self, stamp):
        try:
            scan_data = self.env.get_lidar_scan(0)
        except (AttributeError, IndexError, RuntimeError):
            return
        if not scan_data:
            return
        ranges = _flat_array(scan_data.get("ranges", []))
        if ranges.size == 0:
            return
        message = LaserScan()
        message.header.stamp = stamp
        message.header.frame_id = self.scan_frame
        message.angle_min = float(scan_data.get("angle_min", -math.pi))
        message.angle_max = float(scan_data.get("angle_max", math.pi))
        message.angle_increment = float(
            scan_data.get(
                "angle_increment",
                (message.angle_max - message.angle_min) / max(1, ranges.size - 1),
            )
        )
        message.time_increment = float(scan_data.get("time_increment", 0.0))
        message.scan_time = float(scan_data.get("scan_time", self.step_time))
        message.range_min = float(scan_data.get("range_min", self.lidar_range_min))
        message.range_max = float(scan_data.get("range_max", self.lidar_range_max))
        message.ranges = ranges.astype(float).tolist()
        intensities = scan_data.get("intensities")
        if intensities is not None:
            message.intensities = _flat_array(intensities).astype(float).tolist()
        self.scan_pub.publish(message)

    def _update_actual_path(self, pose_message, force):
        append = force or not self.path_poses
        if not append:
            previous = self.path_poses[-1].pose.position
            distance = math.hypot(
                pose_message.pose.position.x - previous.x,
                pose_message.pose.position.y - previous.y,
            )
            append = distance >= self.path_min_distance
        if append:
            self.path_poses.append(pose_message)
            if len(self.path_poses) > self.max_path_points:
                self.path_poses = self.path_poses[-self.max_path_points :]
        path = Path()
        path.header = pose_message.header
        path.poses = list(self.path_poses)
        self.actual_path_pub.publish(path)

    def _make_markers(self, stamp, x, y, yaw, collision, arrived):
        markers = MarkerArray()

        robot = Marker()
        robot.header.stamp = stamp
        robot.header.frame_id = self.frame_id
        robot.ns = "irsim_robot"
        robot.id = 0
        robot.type = Marker.CUBE
        robot.action = Marker.ADD
        robot.pose.position.x = x
        robot.pose.position.y = y
        robot.pose.position.z = 0.06
        _set_pose_orientation(robot.pose, yaw)
        robot.scale.x = self.robot_length
        robot.scale.y = self.robot_width
        robot.scale.z = 0.12
        if collision:
            robot.color.r, robot.color.g, robot.color.b = 0.85, 0.10, 0.10
        elif arrived:
            robot.color.r, robot.color.g, robot.color.b = 0.10, 0.65, 0.25
        else:
            robot.color.r, robot.color.g, robot.color.b = 0.12, 0.32, 0.68
        robot.color.a = 0.85
        markers.markers.append(robot)

        heading = Marker()
        heading.header = robot.header
        heading.ns = "irsim_robot"
        heading.id = 1
        heading.type = Marker.ARROW
        heading.action = Marker.ADD
        heading.pose = robot.pose
        heading.scale.x = 0.55 * self.robot_length
        heading.scale.y = 0.06
        heading.scale.z = 0.06
        heading.color.r, heading.color.g, heading.color.b, heading.color.a = (
            0.05,
            0.05,
            0.10,
            1.0,
        )
        markers.markers.append(heading)

        goal = Marker()
        goal.header = robot.header
        goal.ns = "irsim_goal"
        goal.id = 0
        goal.type = Marker.ARROW
        goal.action = Marker.ADD
        goal.pose.position.x = self.goal[0]
        goal.pose.position.y = self.goal[1]
        goal.pose.position.z = 0.02
        _set_pose_orientation(goal.pose, self.goal[2])
        goal.scale.x = 0.50
        goal.scale.y = 0.10
        goal.scale.z = 0.10
        goal.color.r, goal.color.g, goal.color.b, goal.color.a = 0.80, 0.15, 0.12, 0.95
        markers.markers.append(goal)
        return markers

    def run(self):
        next_step = time.monotonic()
        while not rospy.is_shutdown() and not self._shutdown:
            self.step()
            next_step += self.step_time
            delay = next_step - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                now = time.monotonic()
                if now - self.last_slow_warning_wall_time >= 5.0:
                    rospy.logwarn(
                        "IR-SIM loop is %.3fs behind real time; consider fewer LiDAR beams "
                        "or a larger step_time.",
                        -delay,
                    )
                    self.last_slow_warning_wall_time = now
                next_step = now

    def shutdown(self):
        if self._shutdown:
            return
        self._shutdown = True
        try:
            if hasattr(self, "env"):
                self.env.end()
        finally:
            if hasattr(self, "_temporary_directory"):
                self._temporary_directory.cleanup()


def main():
    rospy.init_node("irsim_ros_bridge")
    try:
        bridge = IrSimRosBridge()
        bridge.run()
    except Exception as error:
        rospy.logfatal("IR-SIM bridge failed: %s", error)
        raise


if __name__ == "__main__":
    main()
