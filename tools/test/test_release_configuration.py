#!/usr/bin/env python3
"""Portable release defaults: no ROS master or robot required."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]


class ReleaseConfigurationTest(unittest.TestCase):
    def test_standalone_launch_defaults_enable_three_levels_without_control(self):
        for name in ("demo.launch", "planner.launch"):
            tree = ET.parse(ROOT / "isweep_planner" / "launch" / name)
            args = {a.attrib["name"]: a.attrib.get("default")
                    for a in tree.getroot().findall("arg")}
            self.assertEqual(args["enable_three_level_planner"], "true")
            self.assertEqual(args["run_local_planner"], "false")
            self.assertEqual(args["planner_method"], "current_full")

    def test_offline_experiments_do_not_inherit_deployment_profile(self):
        for name in ("experiment_default.yaml", "experiment_smoke.yaml",
                     "experiment_risk_ground_truth.yaml"):
            config = yaml.safe_load((ROOT / "planner_experiments/config" / name).read_text())
            self.assertEqual(config["paths"]["planner_config"],
                             "package://isweep_planner/config/planner_benchmark.yaml")
        profile = yaml.safe_load((ROOT / "isweep_planner/config/planner_benchmark.yaml").read_text())
        self.assertTrue(profile["risk"].get("enable_three_level", True))

    def test_configured_meshes_exist(self):
        for name in ("planner.yaml", "planner_benchmark.yaml"):
            profile = yaml.safe_load((ROOT / "isweep_planner/config" / name).read_text())
            self.assertTrue((ROOT / "isweep_planner" / profile["inputdata"]).is_file())

    def test_map_yaml_files_reference_shipped_images(self):
        for base in (ROOT / "isweep_planner/maps", ROOT / "planner_experiments/maps"):
            for path in base.rglob("*.yaml"):
                data = yaml.safe_load(path.read_text())
                if isinstance(data, dict) and "image" in data:
                    image = Path(data["image"])
                    self.assertFalse(image.is_absolute(), str(path))
                    self.assertTrue((path.parent / image).is_file(), str(path))


if __name__ == "__main__":
    unittest.main()
