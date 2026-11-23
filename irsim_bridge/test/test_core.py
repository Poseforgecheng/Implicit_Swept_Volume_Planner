#!/usr/bin/env python3

import math
import os
import tempfile
import unittest

import numpy as np
from PIL import Image
import yaml

from irsim_bridge.core import AckermannLimits
from irsim_bridge.core import build_irsim_world_config
from irsim_bridge.core import load_ros_map
from irsim_bridge.core import quaternion_from_yaw
from irsim_bridge.core import sanitize_ackermann_command
from irsim_bridge.core import steering_angle_to_yaw_rate
from irsim_bridge.core import yaw_rate_to_steering_angle
from irsim_bridge.core import yaw_from_quaternion


class CoreTest(unittest.TestCase):
    def test_ros_image_rows_are_flipped_into_world_order(self):
        with tempfile.TemporaryDirectory() as directory:
            image_path = os.path.join(directory, "map.png")
            yaml_path = os.path.join(directory, "map.yaml")
            Image.fromarray(
                np.array([[0, 255], [255, 127]], dtype=np.uint8)
            ).save(image_path)
            with open(yaml_path, "w", encoding="utf-8") as stream:
                yaml.safe_dump(
                    {
                        "image": "map.png",
                        "resolution": 0.5,
                        "origin": [1.0, 2.0, 0.0],
                        "negate": 0,
                        "occupied_thresh": 0.65,
                        "free_thresh": 0.196,
                    },
                    stream,
                )

            result = load_ros_map(yaml_path, unknown_is_occupied=True)
            self.assertEqual(result.width, 2)
            self.assertEqual(result.height, 2)
            self.assertEqual(result.occupancy.tolist(), [[0, 100], [100, 0]])
            self.assertEqual(result.binary_image.tolist(), [[0, 255], [255, 0]])

    def test_ackermann_gate_rejects_in_place_turn_and_limits_radius(self):
        limits = AckermannLimits(
            max_linear_speed=0.5,
            max_yaw_rate=2.0,
            min_turn_radius=1.0,
        )
        self.assertEqual(sanitize_ackermann_command(0.0, 1.0, limits), (0.0, 0.0))
        linear, angular = sanitize_ackermann_command(0.4, 1.5, limits)
        self.assertAlmostEqual(linear, 0.4)
        self.assertAlmostEqual(angular, 0.4)

    def test_yaw_quaternion_round_trip(self):
        for yaw in (-math.pi, -0.7, 0.0, 1.2, math.pi):
            actual = yaw_from_quaternion(quaternion_from_yaw(yaw))
            self.assertAlmostEqual(math.sin(actual), math.sin(yaw), places=12)
            self.assertAlmostEqual(math.cos(actual), math.cos(yaw), places=12)

    def test_twist_and_steering_conversion_round_trip(self):
        steering = yaw_rate_to_steering_angle(0.4, 0.4, 0.3, 0.8)
        self.assertAlmostEqual(steering, math.atan(0.3), places=12)
        yaw_rate = steering_angle_to_yaw_rate(0.4, steering, 0.3)
        self.assertAlmostEqual(yaw_rate, 0.4, places=12)
        self.assertEqual(yaw_rate_to_steering_angle(0.0, 1.0, 0.3, 0.8), 0.0)

    def test_generated_world_uses_map_extent_and_steering_ackermann(self):
        with tempfile.TemporaryDirectory() as directory:
            image_path = os.path.join(directory, "map.png")
            yaml_path = os.path.join(directory, "map.yaml")
            Image.fromarray(np.full((4, 6), 255, dtype=np.uint8)).save(image_path)
            with open(yaml_path, "w", encoding="utf-8") as stream:
                yaml.safe_dump(
                    {
                        "image": "map.png",
                        "resolution": 0.5,
                        "origin": [-1.0, -2.0, 0.0],
                    },
                    stream,
                )
            map_data = load_ros_map(yaml_path)
            config = build_irsim_world_config(
                map_data=map_data,
                binary_map_path=image_path,
                start=[0.0, 0.0, 0.0],
                goal=[1.0, 1.0, 0.0],
                step_time=0.1,
                robot_length=0.45,
                robot_width=0.36,
                wheelbase=0.30,
                min_turn_radius=0.30,
                max_linear_speed=0.45,
                max_steering_rate=1.0,
                linear_acceleration=0.5,
                goal_tolerance=0.1,
                lidar_range_min=0.05,
                lidar_range_max=8.0,
                lidar_angle_range=2.0 * math.pi,
                lidar_beams=180,
                lidar_noise=False,
                seed=7,
            )
            self.assertEqual(config["world"]["width"], 3.0)
            self.assertEqual(config["world"]["height"], 2.0)
            self.assertEqual(config["world"]["offset"], [-1.0, -2.0])
            self.assertNotIn("name", config["world"])
            self.assertEqual(config["robot"]["kinematics"]["mode"], "steer")
            self.assertAlmostEqual(
                config["robot"]["vel_max"][1], math.atan(1.0), places=12
            )


if __name__ == "__main__":
    unittest.main()
