#!/usr/bin/env python3
"""No master, publishers, hardware, or ROS imports required for these tests."""

import csv
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "record_planning_evidence.py"
SPEC = importlib.util.spec_from_file_location("record_planning_evidence", SCRIPT)
rec = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(rec)


class GeometryTests(unittest.TestCase):
    def test_segment_projection_is_not_nearest_sample(self):
        result = rec.project_to_path(5, 1, 0, [(0, 0, 0, 0), (10, 0, 0, 10)])
        self.assertAlmostEqual(result["error_m"], 1)
        self.assertAlmostEqual(result["lateral_m"], 1)
        self.assertAlmostEqual(result["reference_s_m"], 5)

    def test_yaw_wrap_and_interpolation(self):
        result = rec.project_to_path(1, 0, math.pi, [
            (0, 0, math.radians(179), 0), (2, 0, math.radians(-179), 2)])
        self.assertAlmostEqual(result["yaw_error_rad"], 0)

    def test_duplicates_singletons_and_invalid(self):
        self.assertEqual(rec.project_to_path(0, 1, 0, [(0, 0, 0, 0)])["error_m"], 1)
        self.assertEqual(rec.project_to_path(0, 1, 0, [(0, 0, 0, 0)] * 2)["error_m"], 1)
        self.assertIsNone(rec.project_to_path(0, 0, 0, []))
        self.assertIsNone(rec.project_to_path(0, 0, 0, [(float("nan"), 0, 0, 0)]))

    def test_strict_json_and_quantile(self):
        self.assertEqual(rec.clean_json({"a": [float("nan"), float("inf")]}), {"a": [None, None]})
        self.assertAlmostEqual(rec.percentile([0, 10], 0.95), 9.5)


class StorageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.output = Path(self.temp.name) / "new_session"
        self.store = rec.EvidenceStore(self.output, dwell_time=0.5, timeout=1.0)

    def tearDown(self):
        self.store.close(200)
        self.temp.cleanup()

    def read_csv(self, name):
        self.store.flush()
        with (self.output / name).open(newline="") as stream:
            return list(csv.DictReader(stream))

    def goal(self):
        self.store.goal(100, 100, "map", 10, 0, 0)

    def test_refuses_existing_session(self):
        with self.assertRaises(FileExistsError):
            rec.EvidenceStore(self.output)

    def test_pre_goal_path_never_inherited(self):
        self.store.path("global_path", 99, 99, "map", [(0, 0, 0, 0)])
        self.goal()
        self.store.pose(100.1, 100.1, "map", 0, 1, 0)
        self.assertEqual(self.read_csv("tracking.csv")[-1]["error_status"], "no_reference")

    def test_first_and_latest_references_survive_replan(self):
        self.goal()
        first = self.store.path("global_path", 100.1, 100.1, "map", [(0, 0, 0, 0), (10, 0, 0, 10)])
        last = self.store.path("global_path", 100.2, 100.2, "map", [(0, 2, 0, 0), (10, 2, 0, 10)])
        self.store.pose(100.3, 100.3, "map", 5, 0.5, 0)
        row = self.read_csv("tracking.csv")[-1]
        self.assertEqual(row["first_reference_id"], first)
        self.assertEqual(row["latest_reference_id"], last)
        self.assertEqual(float(row["first_error_m"]), 0.5)
        self.assertEqual(float(row["latest_error_m"]), 1.5)

    def test_empty_path_revokes_latest_not_first(self):
        self.goal()
        self.store.path("global_path", 100.1, 100.1, "map", [(0, 0, 0, 0)])
        self.store.path("global_path", 100.2, 100.2, "map", [])
        self.store.pose(100.3, 100.3, "map", 0, 1, 0)
        row = self.read_csv("tracking.csv")[-1]
        self.assertEqual(row["latest_error_m"], "")
        self.assertEqual(float(row["first_error_m"]), 1)

    def test_frame_mismatch_and_old_pose_blank_errors(self):
        self.goal()
        self.store.path("global_path", 100.1, 100.1, "map", [(0, 0, 0, 0)])
        self.store.pose(100.2, 100.2, "odom", 0, 1, 0)
        self.store.pose(100.3, 90, "map", 0, 1, 0)
        rows = self.read_csv("tracking.csv")
        self.assertEqual(rows[0]["error_status"], "frame_mismatch_no_transform")
        self.assertEqual(rows[1]["error_status"], "pose_predates_goal")
        self.assertTrue(all(row["latest_error_m"] == "" for row in rows))

    def test_old_path_is_not_reference(self):
        self.goal()
        self.store.path("global_path", 100.1, 90, "map", [(0, 0, 0, 0)])
        self.assertIsNone(self.store.latest_reference)

    def test_delayed_pose_and_odom_do_not_fake_attainment(self):
        self.goal()
        self.store.latest_odom = (101, 100, 0.0, 0.0)
        self.store.pose(101, 100.1, "map", 10, 0, 0)
        row = self.read_csv("tracking.csv")[-1]
        self.assertEqual(row["error_status"], "pose_source_stamp_not_fresh")
        self.assertEqual(row["measured_speed_mps"], "")
        self.assertFalse(self.store.active["pose_tolerance_observed"])

    def test_dwell_resets_on_pose_gap_no_fake_success(self):
        self.goal()
        for timestamp in (100.1, 100.3, 102.0):
            self.store.pose(timestamp, timestamp, "map", 10, 0, 0)
        self.assertFalse(self.store.active["pose_tolerance_observed"])
        for timestamp in (102.2, 102.4, 102.6):
            self.store.pose(timestamp, timestamp, "map", 10, 0, 0)
        self.assertTrue(self.store.active["pose_tolerance_observed"])
        self.assertIsNone(self.store.active["stationary_at_attainment"])
        self.store.finish(103, "test")
        row = self.read_csv("runs.csv")[-1]
        self.assertEqual(row["physical_success"], "")
        self.assertEqual(row["contact_observed"], "")

    def test_timeout_retains_subsequent_data(self):
        self.goal()
        self.store.tick(102)
        self.store.pose(103, 103, "map", 1, 0, 0)
        self.assertTrue(self.store.active["deadline_exceeded"])
        self.assertEqual(len(self.read_csv("tracking.csv")), 1)

    def test_second_goal_reset_and_summary_denominators(self):
        self.goal()
        self.store.path("global_path", 100.1, 100.1, "map", [(0, 0, 0, 0), (10, 0, 0, 10)])
        self.store.pose(100.2, 100.2, "map", 5, 1, 0)
        self.store.goal(101, 101, "map", 20, 0, 0)
        self.assertIsNone(self.store.first_reference)
        self.store.pose(101.1, 101.1, "map", 10, 0, 0)
        self.store.close(102)
        rows = self.read_csv_after_close("runs.csv")
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["error_samples"], "1")
        self.assertEqual(rows[1]["error_samples"], "0")
        self.assertEqual(rows[1]["latest_error_mean_m"], "")

    def read_csv_after_close(self, name):
        with (self.output / name).open(newline="") as stream:
            return list(csv.DictReader(stream))

    def test_nonfinite_pose_preserved_without_fake_error(self):
        self.goal()
        self.store.pose(100.2, 100.2, "map", float("nan"), 0, 0)
        row = self.read_csv("tracking.csv")[-1]
        self.assertEqual(row["x"], "")
        self.assertEqual(row["error_status"], "nonfinite_pose")

    def test_no_distance_across_localization_gap(self):
        self.goal()
        self.store.pose(100.2, 100.2, "map", 0, 0, 0)
        self.store.pose(110.2, 110.2, "map", 10, 0, 0)
        self.assertEqual(self.store.active["executed_distance_m"], 0)

    def test_stats_schema_and_no_publishers(self):
        fields, source_hash = rec.load_stat_fields()
        self.assertEqual(len(source_hash), 64)
        self.assertIn("isweep_first_incumbent_time_ms", fields)
        self.assertIn("isweep_full_svsdf_calls", fields)
        self.assertNotIn(".Publisher(", SCRIPT.read_text())


class RosSerializationTests(unittest.TestCase):
    def test_actual_ros_messages_and_bag_roundtrip_without_master(self):
        try:
            import rospy
            import rosbag
            from geometry_msgs.msg import PoseStamped, Twist
            from nav_msgs.msg import Odometry, Path as RosPath, OccupancyGrid
            from std_msgs.msg import Float64MultiArray, String, Bool
            from isweep_planner.msg import GlobalReferencePoint, RiskAwareGlobalReference
        except ImportError as error:
            self.skipTest("ROS message environment unavailable: {}".format(error))

        # Patch all network-facing ROS calls. No master and no Publisher exist.
        fake_clock = [100.0]
        patches = [
            mock.patch.object(rospy, "Subscriber"), mock.patch.object(rospy, "Timer"),
            mock.patch.object(rospy, "on_shutdown"), mock.patch.object(rospy, "get_param", return_value={}),
            mock.patch.object(rospy, "get_time", side_effect=lambda: fake_clock[0]),
            mock.patch.object(rospy.Time, "now", side_effect=lambda: rospy.Time.from_sec(fake_clock[0])),
            mock.patch.object(rospy, "loginfo"), mock.patch.object(rospy, "logwarn"),
            mock.patch.object(rospy, "logwarn_throttle"), mock.patch.object(rospy, "logfatal"),
            mock.patch.object(rospy, "signal_shutdown"), mock.patch.object(rospy, "logerr"),
        ]
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "serialized"
            args = rec.arguments(["--output", str(output)])
            recorder = rec.RosRecorder(args)
            try:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = rospy.Time.from_sec(100)
                goal.pose.orientation.w = 1
                goal.pose.position.x = 10
                recorder.callback("goal", goal)

                fake_clock[0] = 100.1
                path = RosPath()
                path.header.frame_id = "map"
                path.header.stamp = rospy.Time.from_sec(100.1)
                path.poses = [PoseStamped(), PoseStamped()]
                for index, pose in enumerate(path.poses):
                    pose.pose.position.x = index * 10
                    pose.pose.orientation.w = 1
                recorder.callback("global_path", path)
                recorder.callback("local_path", path)

                risk = RiskAwareGlobalReference()
                risk.header = path.header
                risk.points = [GlobalReferencePoint(x=5, yaw=0, risk_level=2, risk_score=0.9)]
                recorder.callback("risk", risk)
                recorder.callback("stats", Float64MultiArray(data=[0.0] * len(recorder.stat_fields)))
                recorder.callback("stats", Float64MultiArray(data=[1.0]))
                recorder.callback("cmd", Twist())
                recorder.callback("local_cmd", Twist())
                recorder.callback("status", String(data="OK|LOW_RISK_FOLLOW"))
                recorder.callback("replan", Bool(data=True))
                recorder.callback("replan", Bool(data=True))

                grid = OccupancyGrid()
                grid.header.frame_id = "map"
                grid.info.width = grid.info.height = 2
                grid.info.resolution = 0.1
                grid.info.origin.orientation.w = 1
                grid.data = [0, -1, 100, 0]
                recorder.callback("map", grid)
                recorder.callback("map", grid)  # identical geometry, one snapshot
                self.assertEqual(len(list((output / "maps").glob("*.json"))), 1)

                fake_clock[0] = 100.2
                pose = Odometry()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.from_sec(100.2)
                pose.pose.pose.orientation.w = 1
                pose.pose.pose.position.x = 5
                pose.pose.pose.position.y = 1
                recorder.callback("odom", pose)
                recorder.callback("pose", pose)
                recorder.tick(None)
                self.assertIsNone(recorder.fatal_error)
                self.assertEqual(recorder.store.active["replan_true_edges"], 1)
            finally:
                recorder.shutdown()
            with rosbag.Bag(str(output / "evidence.bag")) as bag:
                messages = list(bag.read_messages())
            self.assertEqual(len(messages), 15)
            self.assertEqual(len(messages), sum(recorder.store.counts.values()))
            self.assertTrue(any(topic == rec.TOPICS["pose"] for topic, _, _ in messages))
            with (output / "tracking.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(float(rows[0]["latest_error_m"]), 1.0)
            metadata = json.loads((output / "metadata.json").read_text())
            self.assertEqual(metadata["missing_topics"], [])
            events = [json.loads(line) for line in (output / "events.jsonl").read_text().splitlines()]
            stats = [event for event in events if event["kind"] == "planning_stats"]
            self.assertTrue(stats[0]["schema_length_matches"])
            self.assertIsNone(stats[1]["named_values"])


if __name__ == "__main__":
    unittest.main()
