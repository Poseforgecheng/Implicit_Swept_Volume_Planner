#!/usr/bin/env python3
import pathlib
import sys
import unittest
import argparse
import os
import tempfile

import numpy as np
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from _common import resolve_catkin_executable, stable_hash
from generate_parametric_maps import make_grid
from generate_tasks import generate_task_document


class PythonUtilitiesTest(unittest.TestCase):
    def test_executable_uses_sourced_overlay_instead_of_source_parent(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            older = root / "underlay"
            active = root / "unrelated_workspace" / "devel"
            node = active / "lib" / "planner_experiments" / "experiment_runner_node"
            node.parent.mkdir(parents=True)
            node.write_text("#!/bin/sh\nexit 0\n")
            node.chmod(0o755)
            environment = {"CMAKE_PREFIX_PATH": os.pathsep.join([str(older), str(active)])}
            self.assertEqual(resolve_catkin_executable(
                "planner_experiments", "experiment_runner_node", environment), node)

    def test_missing_executable_reports_how_to_source_workspace(self):
        with self.assertRaisesRegex(FileNotFoundError, "source devel/setup.bash"):
            resolve_catkin_executable("planner_experiments", "experiment_runner_node", {})

    def test_stable_hash_order_independent(self):
        self.assertEqual(stable_hash({"a": 1, "b": 2}),
                         stable_hash({"b": 2, "a": 1}))

    def test_wilson_known_interval(self):
        # Local copy keeps the unit test independent from optional pandas/scipy.
        import math
        success, count, z = 5, 10, 1.959963984540054
        p = success / count
        denominator = 1 + z * z / count
        center = (p + z * z / (2 * count)) / denominator
        half = z * math.sqrt(p * (1 - p) / count + z * z / (4 * count * count)) / denominator
        self.assertAlmostEqual(center - half, 0.2365930905, places=8)
        self.assertAlmostEqual(center + half, 0.7634069095, places=8)

    def test_parametric_map_seed_is_reproducible(self):
        arguments = argparse.Namespace(type="random_obstacles", resolution=0.1,
                                       width=6.0, height=5.0, door_width=0.9,
                                       channel_width=1.2, obstacle_count=8, seed=7)
        self.assertTrue(np.array_equal(make_grid(arguments), make_grid(arguments)))

    def test_run_key_changes_with_configuration(self):
        base = {"method": "current_full", "seed": 1, "config": "a"}
        changed = dict(base, config="b")
        self.assertNotEqual(stable_hash(base), stable_hash(changed))

    def test_generated_tasks_are_seed_reproducible(self):
        map_path = ROOT / "maps" / "smoke_empty" / "map.yaml"
        robot_path = ROOT / "config" / "robot_shapes" / "circle.yaml"
        first = generate_task_document(
            map_path, robot_path, num_tasks=2, seed=13,
            min_distance=2.0, yaw_mode="cardinal")
        second = generate_task_document(
            map_path, robot_path, num_tasks=2, seed=13,
            min_distance=2.0, yaw_mode="cardinal")
        self.assertEqual(first, second)
        self.assertEqual(len(first["tasks"]), 2)
        self.assertNotEqual(first["tasks"][0]["start"],
                            first["tasks"][0]["goal"])

    def test_every_shipped_map_has_a_named_task_file(self):
        maps = {
            path.parent.name
            for path in (ROOT / "maps").glob("*/map.yaml")
        }
        task_files = {
            path.stem
            for path in (ROOT / "tasks").glob("*.yaml")
            if path.stem != "smoke"
        }
        self.assertEqual(maps, task_files)
        for map_name in sorted(maps):
            task_path = ROOT / "tasks" / f"{map_name}.yaml"
            document = yaml.safe_load(task_path.read_text(encoding="utf-8"))
            self.assertEqual(document["map"], map_name)
            self.assertEqual(len(document["tasks"]), 2)
            self.assertEqual(
                len({task["id"] for task in document["tasks"]}), 2)
            self.assertTrue(
                (task_path.parent / document["map_yaml"]).resolve().exists())
            self.assertTrue(
                (task_path.parent / document["robot_yaml"]).resolve().exists())


if __name__ == "__main__":
    unittest.main()
