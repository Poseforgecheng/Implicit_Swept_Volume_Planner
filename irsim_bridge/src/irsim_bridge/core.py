"""Pure, testable helpers shared by the IR-SIM ROS bridge.

This module intentionally has no ROS or IR-SIM imports.  Keeping map conversion
and Ackermann command handling here makes the two simulator interfaces auditable
without starting a ROS master.
"""

from dataclasses import dataclass
import hashlib
import math
import os
from typing import Any, Dict, Sequence, Tuple

import numpy as np
from PIL import Image
import yaml


@dataclass(frozen=True)
class RosMapData:
    yaml_path: str
    image_path: str
    resolution: float
    origin: Tuple[float, float, float]
    occupancy: np.ndarray
    binary_image: np.ndarray
    checksum: str

    @property
    def width(self) -> int:
        return int(self.occupancy.shape[1])

    @property
    def height(self) -> int:
        return int(self.occupancy.shape[0])

    @property
    def width_m(self) -> float:
        return self.width * self.resolution

    @property
    def height_m(self) -> float:
        return self.height * self.resolution


@dataclass(frozen=True)
class AckermannLimits:
    max_linear_speed: float = 0.45
    max_yaw_rate: float = 1.0
    min_turn_radius: float = 0.30
    linear_deadband: float = 0.01
    angular_deadband: float = 0.01


def _absolute_image_path(yaml_path: str, image_value: str) -> str:
    expanded = os.path.expanduser(str(image_value))
    if os.path.isabs(expanded):
        return os.path.abspath(expanded)
    return os.path.abspath(os.path.join(os.path.dirname(yaml_path), expanded))


def load_ros_map(yaml_path: str, unknown_is_occupied: bool = True) -> RosMapData:
    """Load a ROS map_server YAML and produce one grid for ROS and IR-SIM.

    ``occupancy`` is stored in ROS world-row order (row zero is the map's
    lowest y row).  ``binary_image`` remains in image order and is written to
    the temporary IR-SIM world.  Unknown cells are occupied by default so the
    planner and the simulator use the same conservative collision truth.
    """

    yaml_path = os.path.abspath(os.path.expanduser(yaml_path))
    with open(yaml_path, "r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)

    required = ("image", "resolution", "origin")
    missing = [key for key in required if key not in metadata]
    if missing:
        raise ValueError("map YAML missing fields: {}".format(", ".join(missing)))

    image_path = _absolute_image_path(yaml_path, metadata["image"])
    grayscale = np.asarray(Image.open(image_path).convert("L"), dtype=np.float64)
    resolution = float(metadata["resolution"])
    if resolution <= 0.0:
        raise ValueError("map resolution must be positive")

    origin_values = list(metadata["origin"])
    if len(origin_values) != 3:
        raise ValueError("map origin must contain [x, y, yaw]")
    origin = tuple(float(value) for value in origin_values)
    if abs(origin[2]) > 1.0e-9:
        raise ValueError("IR-SIM image maps currently require zero map-origin yaw")

    negate = int(metadata.get("negate", 0))
    occupied_threshold = float(metadata.get("occupied_thresh", 0.65))
    free_threshold = float(metadata.get("free_thresh", 0.196))
    if not 0.0 <= free_threshold < occupied_threshold <= 1.0:
        raise ValueError("map thresholds must satisfy 0 <= free < occupied <= 1")

    occupancy_probability = (
        grayscale / 255.0 if negate else (255.0 - grayscale) / 255.0
    )
    occupied = occupancy_probability > occupied_threshold
    free = occupancy_probability < free_threshold
    unknown = np.logical_not(np.logical_or(occupied, free))

    occupancy_image_order = np.full(grayscale.shape, -1, dtype=np.int8)
    occupancy_image_order[free] = 0
    occupancy_image_order[occupied] = 100
    if unknown_is_occupied:
        occupancy_image_order[unknown] = 100

    # IR-SIM consumes an ordinary image: black obstacles, white free space.
    binary_image = np.full(grayscale.shape, 255, dtype=np.uint8)
    binary_image[np.logical_or(occupied, unknown if unknown_is_occupied else False)] = 0

    # ROS OccupancyGrid starts at the map origin (bottom-left); images start at
    # top-left, hence the vertical flip.
    occupancy = np.flipud(occupancy_image_order).copy()

    digest = hashlib.sha256()
    digest.update(binary_image.tobytes())
    digest.update(repr((resolution, origin, unknown_is_occupied)).encode("utf-8"))
    return RosMapData(
        yaml_path=yaml_path,
        image_path=image_path,
        resolution=resolution,
        origin=origin,
        occupancy=occupancy,
        binary_image=binary_image,
        checksum=digest.hexdigest(),
    )


def sanitize_ackermann_command(
    linear_speed: float,
    yaw_rate: float,
    limits: AckermannLimits,
) -> Tuple[float, float]:
    """Clamp a Twist command to car-like ``[v, omega]`` limits.

    The bridge never invents forward motion for an in-place command.  The
    existing ``planner_bridge`` performs that policy decision upstream; this
    final gate only enforces finite values, deadbands, and the minimum radius.
    """

    if not math.isfinite(linear_speed) or not math.isfinite(yaw_rate):
        return 0.0, 0.0

    max_linear = max(0.0, float(limits.max_linear_speed))
    max_yaw = max(0.0, float(limits.max_yaw_rate))
    linear_speed = max(-max_linear, min(max_linear, float(linear_speed)))
    yaw_rate = max(-max_yaw, min(max_yaw, float(yaw_rate)))

    if abs(linear_speed) < max(0.0, float(limits.linear_deadband)):
        linear_speed = 0.0
    if abs(yaw_rate) < max(0.0, float(limits.angular_deadband)):
        yaw_rate = 0.0

    if linear_speed == 0.0:
        return 0.0, 0.0

    min_radius = max(0.0, float(limits.min_turn_radius))
    if min_radius > 0.0:
        radius_limited_yaw = abs(linear_speed) / min_radius
        if abs(yaw_rate) > radius_limited_yaw:
            yaw_rate = math.copysign(radius_limited_yaw, yaw_rate)
    return linear_speed, yaw_rate


def yaw_rate_to_steering_angle(
    linear_speed: float,
    yaw_rate: float,
    wheelbase: float,
    max_steering_angle: float,
) -> float:
    """Convert ROS ``[v, omega]`` into IR-SIM Ackermann ``[v, delta]``."""

    values = (linear_speed, yaw_rate, wheelbase, max_steering_angle)
    if not all(math.isfinite(value) for value in values):
        return 0.0
    if abs(linear_speed) <= 1.0e-9 or abs(yaw_rate) <= 1.0e-9:
        return 0.0
    if wheelbase <= 0.0 or max_steering_angle <= 0.0:
        return 0.0
    steering = math.atan(float(wheelbase) * float(yaw_rate) / float(linear_speed))
    return max(-max_steering_angle, min(max_steering_angle, steering))


def steering_angle_to_yaw_rate(
    linear_speed: float,
    steering_angle: float,
    wheelbase: float,
) -> float:
    """Convert IR-SIM's applied steering angle to actual vehicle yaw rate."""

    values = (linear_speed, steering_angle, wheelbase)
    if not all(math.isfinite(value) for value in values) or wheelbase <= 0.0:
        return 0.0
    return float(linear_speed) * math.tan(float(steering_angle)) / float(wheelbase)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


def yaw_from_quaternion(quaternion: Sequence[float]) -> float:
    x, y, z, w = (float(value) for value in quaternion)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def build_irsim_world_config(
    map_data: RosMapData,
    binary_map_path: str,
    start: Sequence[float],
    goal: Sequence[float],
    step_time: float,
    robot_length: float,
    robot_width: float,
    wheelbase: float,
    min_turn_radius: float,
    max_linear_speed: float,
    max_steering_rate: float,
    linear_acceleration: float,
    goal_tolerance: float,
    lidar_range_min: float,
    lidar_range_max: float,
    lidar_angle_range: float,
    lidar_beams: int,
    lidar_noise: bool,
    seed: int,
) -> Dict[str, Any]:
    """Create the generated IR-SIM YAML dictionary for one Ackermann robot."""

    if len(start) != 3 or len(goal) != 3:
        raise ValueError("start and goal must be [x, y, yaw]")
    if min(robot_length, robot_width, wheelbase, min_turn_radius, step_time) <= 0.0:
        raise ValueError("robot dimensions and step_time must be positive")

    max_steering_angle = math.atan(float(wheelbase) / float(min_turn_radius))

    return {
        "world": {
            "width": float(map_data.width_m),
            "height": float(map_data.height_m),
            "offset": [float(map_data.origin[0]), float(map_data.origin[1])],
            "step_time": float(step_time),
            "sample_time": float(step_time),
            "collision_mode": "stop",
            "obstacle_map": os.path.abspath(binary_map_path),
        },
        "robot": {
            "name": "isweep_ackermann",
            "kinematics": {"name": "acker", "mode": "steer"},
            "shape": {
                "name": "rectangle",
                "length": float(robot_length),
                "width": float(robot_width),
                "wheelbase": float(wheelbase),
            },
            "state": [float(start[0]), float(start[1]), float(start[2]), 0.0],
            "goal": [float(goal[0]), float(goal[1]), float(goal[2])],
            "vel_min": [-abs(float(max_linear_speed)), -max_steering_angle],
            "vel_max": [abs(float(max_linear_speed)), max_steering_angle],
            "acce": [
                abs(float(linear_acceleration)),
                abs(float(max_steering_rate)),
            ],
            "goal_threshold": float(goal_tolerance),
            "arrive_mode": "position",
            "sensors": [
                {
                    "name": "lidar2d",
                    "range_min": float(lidar_range_min),
                    "range_max": float(lidar_range_max),
                    "angle_range": float(lidar_angle_range),
                    "number": int(lidar_beams),
                    "scan_time": float(step_time),
                    "noise": bool(lidar_noise),
                    "offset": [0.0, 0.0, 0.0],
                    "plot": {"alpha": 0.15, "color": "royalblue"},
                }
            ],
            "plot": {
                "show_trail": True,
                "show_goal": True,
                "show_trajectory": False,
                "trail_alpha": 0.25,
            },
            "color": "royalblue",
        },
    }
