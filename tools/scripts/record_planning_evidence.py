#!/usr/bin/env python3
"""Read-only ROS1 evidence recorder. --help and geometry tests need no ROS.

This node NEVER publishes commands, goals, parameters, or stop requests.
CSV errors are geometric path-following errors, not time-indexed tracking errors.
"""

import argparse
import ast
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import sys
import threading
import time
from datetime import datetime, timezone


TOPICS = {
    "goal": "/move_base_simple/goal",
    "pose": "/localization",
    "odom": "/odom",
    "lio_odom": "/Odometry",
    "cmd": "/cmd_vel",
    "local_cmd": "/isweep_planner/local_cmd",
    "global_path": "/isweep_planner/trajectory",
    "coarse_path": "/isweep_planner/trajectory_astar",
    "local_path": "/isweep_planner/local_trajectory",
    "risk": "/isweep_planner/risk_aware_global_reference",
    "stats": "/isweep_planner/planning_stats",
    "planning_time": "/isweep_planner/planning_time",
    "status": "/isweep_planner/local_planner_status",
    "replan": "/isweep_planner/replan_request",
    "map": "/prior_map",
    "planner_map": "/isweep_planner/map",
    "scan": "/scan",
    "tf": "/tf",
    "tf_static": "/tf_static",
    "initialpose": "/initialpose",
}
RISK_FIELDS = (
    "x y yaw risk_level risk_score effective_clearance feasible_yaw_width "
    "feasible_yaw_overlap correction_distance uncertainty_erosion clearance_risk "
    "orientation_width_risk orientation_continuity_risk correction_risk "
    "uncertainty_risk clearance segment_id s preferred_speed"
).split()
TIME_FIELDS = ["run_id", "receipt_ros_s", "source_stamp_s", "wall_unix_s", "elapsed_s"]
ERROR_FIELDS = ["reference_id", "error_m", "lateral_m", "yaw_error_rad", "reference_s_m"]
TRACK_FIELDS = TIME_FIELDS + ["frame_id", "x", "y", "yaw_rad", "error_status"] + [
    prefix + name for prefix in ("first_", "latest_") for name in ERROR_FIELDS
] + ["goal_position_error_m", "goal_yaw_error_rad", "inside_goal_tolerance",
     "pose_receipt_gap_s", "pose_source_age_s", "odom_age_s", "odom_source_age_s",
     "measured_speed_mps", "measured_yaw_rate_radps"]
CMD_FIELDS = TIME_FIELDS + ["kind", "linear_x", "linear_y", "angular_z"]
PATH_FIELDS = TIME_FIELDS + ["kind", "reference_id", "frame_id", "point_index",
                            "pose_stamp_s", "x", "y", "yaw_rad", "s_m"]
RISK_CSV_FIELDS = TIME_FIELDS + ["reference_id", "frame_id", "point_index"] + RISK_FIELDS
RUN_FIELDS = [
    "run_id", "mode", "goal_receipt_ros_s", "goal_source_stamp_s", "goal_frame",
    "goal_x", "goal_y", "goal_yaw_rad", "closed_reason", "observed_duration_s",
    "pose_tolerance_observed", "time_to_pose_tolerance_s", "stationary_at_attainment",
    "deadline_exceeded", "first_path_latency_s", "global_path_messages",
    "reference_update_count", "replan_true_edges", "pose_samples", "error_samples",
    "latest_error_mean_m", "latest_error_p95_m", "latest_error_max_m",
    "latest_abs_yaw_mean_rad", "latest_abs_yaw_p95_rad",
    "last_goal_position_error_m", "last_goal_yaw_error_rad", "executed_distance_m",
    "physical_success", "manual_intervention", "contact_observed",
]


def clean_json(value):
    """Strict JSON: unavailable/non-finite values are null, never fake zeros."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): clean_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_json(item) for item in value]
    return value


def finite(value):
    return value is not None and math.isfinite(value)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(q):
    values = (q.x, q.y, q.z, q.w)
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1e-12:
        return float("nan")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def normalized_frame(frame):
    return str(frame).lstrip("/")


def path_points(poses):
    points = []
    distance = 0.0
    for pose in poses:
        p = pose.pose.position
        yaw = yaw_of(pose.pose.orientation)
        if points:
            distance += math.hypot(p.x - points[-1][0], p.y - points[-1][1])
        points.append((p.x, p.y, yaw, distance))
    return points


def project_to_path(x, y, yaw, points):
    """Nearest segment projection, shortest-angle yaw interpolation, signed e_y.

    At intersections a purely geometric match may be ambiguous; no progress or
    time matching is implied. Duplicate points and singleton paths are allowed.
    """
    if not points or not all(finite(v) for v in (x, y, yaw)):
        return None
    if not all(all(finite(v) for v in point) for point in points):
        return None
    best = None
    segments = zip(points[:-1], points[1:]) if len(points) > 1 else [(points[0], points[0])]
    for left, right in segments:
        dx, dy = right[0] - left[0], right[1] - left[1]
        length_sq = dx * dx + dy * dy
        alpha = (max(0.0, min(1.0, ((x - left[0]) * dx + (y - left[1]) * dy) /
                             length_sq)) if length_sq > 1e-20 else 0.0)
        px, py = left[0] + alpha * dx, left[1] + alpha * dy
        ref_yaw = wrap(left[2] + alpha * wrap(right[2] - left[2]))
        ex, ey = x - px, y - py
        distance_sq = ex * ex + ey * ey
        if best is None or distance_sq < best[0]:
            best = (distance_sq, {
                "error_m": math.sqrt(distance_sq),
                "lateral_m": -math.sin(ref_yaw) * ex + math.cos(ref_yaw) * ey,
                "yaw_error_rad": wrap(yaw - ref_yaw),
                "reference_s_m": left[3] + alpha * (right[3] - left[3]),
            })
    return best[1]


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = fraction * (len(ordered) - 1)
    low, high = int(math.floor(index)), int(math.ceil(index))
    return ordered[low] + (index - low) * (ordered[high] - ordered[low])


def load_stat_fields():
    """Use the existing recorder's schema without importing rospy at test time."""
    source = Path(__file__).with_name("experiment_recorder.py")
    tree = ast.parse(source.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name) and target.id == "PLANNING_STAT_FIELDS"
                for target in node.targets):
            return ast.literal_eval(node.value), hashlib.sha256(source.read_bytes()).hexdigest()
    raise ValueError("PLANNING_STAT_FIELDS not found in existing recorder")


class EvidenceStore:
    """ROS-independent storage and run bookkeeping, used by tests too."""

    def __init__(self, output, mode="isweep_v3", position_tolerance=0.10,
                 yaw_tolerance=0.20, dwell_time=1.0, timeout=120.0, pose_max_gap=0.5):
        self.output = Path(output).expanduser().resolve()
        # Never mix new evidence into a previous session, including empty dirs.
        self.output.mkdir(parents=True, exist_ok=False)
        self.mode = mode
        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.dwell_time = dwell_time
        self.timeout = timeout
        self.pose_max_gap = pose_max_gap
        self.streams = []
        self.writers = {}
        self.active = None
        self.run_number = 0
        self.sequence = 0
        self.closed = False
        self.counts = {}
        self.errors = []
        self.latest_odom = None
        self.first_reference = None
        self.latest_reference = None
        for filename, fields in (("tracking.csv", TRACK_FIELDS), ("commands.csv", CMD_FIELDS),
                                 ("paths.csv", PATH_FIELDS), ("risk.csv", RISK_CSV_FIELDS),
                                 ("runs.csv", RUN_FIELDS)):
            stream = (self.output / filename).open("x", newline="", encoding="utf-8")
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            self.streams.append(stream)
            self.writers[filename] = writer
        self.events = (self.output / "events.jsonl").open("x", encoding="utf-8")
        self.streams.append(self.events)
        (self.output / "maps").mkdir()
        self.map_hashes = {}

    def row(self, filename, data):
        self.writers[filename].writerow({key: "" if value is None else value
                                        for key, value in clean_json(data).items()})

    def timing(self, receipt, source=None):
        return {"run_id": self.active["run_id"] if self.active else "",
                "receipt_ros_s": receipt, "source_stamp_s": source,
                "wall_unix_s": time.time(),
                "elapsed_s": receipt - self.active["goal_receipt_ros_s"] if self.active else None}

    def event(self, kind, receipt, source=None, **payload):
        self.sequence += 1
        self.events.write(json.dumps(clean_json(dict(
            self.timing(receipt, source), sequence=self.sequence, kind=kind, **payload)),
            ensure_ascii=False, allow_nan=False) + "\n")

    def goal(self, receipt, source, frame, x, y, yaw):
        self.finish(receipt, "next_goal")
        self.run_number += 1
        run_id = "run_{:03d}".format(self.run_number)
        self.active = dict.fromkeys(RUN_FIELDS)
        self.active.update(run_id=run_id, mode=self.mode, goal_receipt_ros_s=receipt,
                           goal_source_stamp_s=source, goal_frame=frame, goal_x=x,
                           goal_y=y, goal_yaw_rad=yaw, pose_tolerance_observed=False,
                           deadline_exceeded=False, global_path_messages=0,
                           reference_update_count=0, replan_true_edges=0, pose_samples=0,
                           executed_distance_m=0.0, error_samples=0)
        self.active.update(_errors=[], _yaw_errors=[], _inside_since=None,
                           _previous_pose=None, _last_pose_receipt=None, _replan=False)
        self.first_reference = self.latest_reference = None
        self.event("goal", receipt, source, frame_id=frame, x=x, y=y, yaw=yaw)
        return run_id

    def path(self, kind, receipt, source, frame, points, pose_stamps=None):
        reference_id = "{}_{:07d}".format(kind, self.sequence + 1)
        stale = bool(self.active and source and self.active["goal_source_stamp_s"] and
                     source < self.active["goal_source_stamp_s"] - 1e-6)
        valid = bool(points) and all(all(finite(v) for v in p) for p in points)
        self.event("path", receipt, source, path_kind=kind, reference_id=reference_id,
                   frame_id=frame, point_count=len(points), stale_for_goal=stale,
                   finite_geometry=valid)
        for index, point in enumerate(points):
            self.row("paths.csv", dict(self.timing(receipt, source), kind=kind,
                     reference_id=reference_id, frame_id=frame, point_index=index,
                     pose_stamp_s=pose_stamps[index] if pose_stamps else None,
                     x=point[0], y=point[1], yaw_rad=point[2], s_m=point[3]))
        if kind != "global_path" or not self.active:
            return reference_id
        self.active["global_path_messages"] += 1
        if stale:
            return reference_id
        # Empty/invalid publications invalidate the latest reference, never reuse
        # a previous plan silently when the planner publishes explicit failure.
        self.latest_reference = {"id": reference_id, "frame": frame, "points": points} if valid else None
        if valid:
            self.active["reference_update_count"] += 1
            if self.first_reference is None:
                self.first_reference = self.latest_reference
                self.active["first_path_latency_s"] = receipt - self.active["goal_receipt_ros_s"]
        return reference_id

    def pose(self, receipt, source, frame, x, y, yaw):
        if not self.active:
            return
        run = self.active
        run["pose_samples"] += 1
        row = dict(self.timing(receipt, source), frame_id=frame, x=x, y=y, yaw_rad=yaw,
                   error_status="no_reference")
        valid_pose = all(finite(v) for v in (x, y, yaw))
        stale_pose = bool(source and run["goal_source_stamp_s"] and
                          source < run["goal_source_stamp_s"] - 1e-6)
        source_age = receipt - source if source else None
        row["pose_source_age_s"] = source_age
        delayed_pose = source_age is not None and (source_age > self.pose_max_gap or source_age < -self.pose_max_gap)
        if stale_pose:
            row["error_status"] = "pose_predates_goal"
        elif delayed_pose:
            row["error_status"] = "pose_source_stamp_not_fresh"
        elif not valid_pose:
            row["error_status"] = "nonfinite_pose"
        for prefix, reference in (("first_", self.first_reference), ("latest_", self.latest_reference)):
            if reference is None or not valid_pose or stale_pose or delayed_pose:
                continue
            row[prefix + "reference_id"] = reference["id"]
            if not frame or not reference["frame"] or normalized_frame(frame) != normalized_frame(reference["frame"]):
                if prefix == "latest_":
                    row["error_status"] = "frame_mismatch_no_transform"
                continue
            error = project_to_path(x, y, yaw, reference["points"])
            if error is not None:
                row.update({prefix + key: value for key, value in error.items()})
                if prefix == "latest_":
                    row["error_status"] = "ok_geometric"
                    run["_errors"].append(error["error_m"])
                    run["_yaw_errors"].append(abs(error["yaw_error_rad"]))
        if self.latest_odom:
            age = receipt - self.latest_odom[0]
            odom_source_age = receipt - self.latest_odom[1] if self.latest_odom[1] else None
            row["odom_age_s"] = age
            row["odom_source_age_s"] = odom_source_age
            source_fresh = odom_source_age is None or 0 <= odom_source_age <= self.pose_max_gap
            if 0.0 <= age <= self.pose_max_gap and source_fresh:
                row["measured_speed_mps"], row["measured_yaw_rate_radps"] = self.latest_odom[2:]
        goal_same_frame = (bool(frame) and bool(run["goal_frame"]) and
                           normalized_frame(frame) == normalized_frame(run["goal_frame"]))
        previous_time = run["_last_pose_receipt"]
        row["pose_receipt_gap_s"] = receipt - previous_time if previous_time is not None else None
        if valid_pose and not stale_pose and not delayed_pose and goal_same_frame:
            position_error = math.hypot(x - run["goal_x"], y - run["goal_y"])
            yaw_error = abs(wrap(yaw - run["goal_yaw_rad"]))
            row.update(goal_position_error_m=position_error, goal_yaw_error_rad=yaw_error)
            run["last_goal_position_error_m"] = position_error
            run["last_goal_yaw_error_rad"] = yaw_error
            previous = run["_previous_pose"]
            if (previous and normalized_frame(previous[0]) == normalized_frame(frame) and
                    previous_time is not None and 0 <= receipt - previous_time <= self.pose_max_gap):
                run["executed_distance_m"] += math.hypot(x - previous[1], y - previous[2])
            run["_previous_pose"] = (frame, x, y)
            inside = position_error <= self.position_tolerance and yaw_error <= self.yaw_tolerance
            row["inside_goal_tolerance"] = inside
            gap_ok = previous_time is not None and 0 <= receipt - previous_time <= self.pose_max_gap
            if not inside:
                run["_inside_since"] = None
            elif run["_inside_since"] is None or not gap_ok:
                run["_inside_since"] = receipt
            if inside and receipt - run["_inside_since"] >= self.dwell_time and not run["pose_tolerance_observed"]:
                run["pose_tolerance_observed"] = True
                run["time_to_pose_tolerance_s"] = receipt - run["goal_receipt_ros_s"]
                speed, angular = row.get("measured_speed_mps"), row.get("measured_yaw_rate_radps")
                stationary = (abs(speed) < 0.03 and abs(angular) < 0.03
                              if finite(speed) and finite(angular) else None)
                run["stationary_at_attainment"] = stationary
                self.event("pose_tolerance_observed_NOT_physical_success", receipt, source,
                           stationary_at_attainment=stationary)
        else:
            run["_inside_since"] = None
        run["_last_pose_receipt"] = receipt
        self.row("tracking.csv", row)

    def tick(self, receipt):
        if self.active and self.timeout > 0 and not self.active["deadline_exceeded"]:
            if receipt - self.active["goal_receipt_ros_s"] >= self.timeout:
                self.active["deadline_exceeded"] = True
                self.event("observation_deadline_exceeded_recording_continues", receipt)
        self.flush()

    def finish(self, receipt, reason):
        if self.active is None:
            return
        run = self.active
        run.update(closed_reason=reason, observed_duration_s=receipt - run["goal_receipt_ros_s"],
                   error_samples=len(run["_errors"]))
        errors, yaws = run["_errors"], run["_yaw_errors"]
        run.update(latest_error_mean_m=statistics.mean(errors) if errors else None,
                   latest_error_p95_m=percentile(errors, 0.95), latest_error_max_m=max(errors) if errors else None,
                   latest_abs_yaw_mean_rad=statistics.mean(yaws) if yaws else None,
                   latest_abs_yaw_p95_rad=percentile(yaws, 0.95))
        self.event("run_closed", receipt, reason=reason)
        self.row("runs.csv", {name: run[name] for name in RUN_FIELDS})
        self.active = None
        self.flush()

    def flush(self):
        for stream in self.streams:
            stream.flush()

    def close(self, receipt):
        if self.closed:
            return
        self.finish(receipt, "recorder_stopped")
        for stream in self.streams:
            stream.flush()
            os.fsync(stream.fileno())
            stream.close()
        self.closed = True


def ros_to_plain(message):
    if hasattr(message, "__slots__"):
        return {slot: ros_to_plain(getattr(message, slot)) for slot in message.__slots__}
    if isinstance(message, (list, tuple)):
        return [ros_to_plain(item) for item in message]
    return message


def stamp_of(message):
    return message.header.stamp.to_sec() if hasattr(message, "header") else None


class RosRecorder:
    def __init__(self, args):
        import rospy
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Bool, Float64, Float64MultiArray, String
        from tf2_msgs.msg import TFMessage
        from isweep_planner.msg import RiskAwareGlobalReference

        self.rospy = rospy
        self.args = args
        self.lock = threading.RLock()
        self.fatal_error = None
        self.bag = None
        self.subscribers = []
        self.last_receipt = None
        self.stat_fields, stat_hash = load_stat_fields()
        self.topics = dict(TOPICS, pose=args.pose_topic, odom=args.odom_topic,
                           goal=args.goal_topic, map=args.map_topic)
        if len(set(self.topics.values())) != len(self.topics):
            raise ValueError("Topic overrides must not alias another recorded topic")
        if not args.no_bag:
            import rosbag
        self.store = EvidenceStore(args.output, args.mode, args.position_tolerance,
                                   args.yaw_tolerance, args.dwell_time, args.timeout,
                                   args.pose_max_gap)
        self.closed = False
        self.started_wall = time.time()
        self.metadata = {
            "schema_version": 1, "arguments": vars(args), "topics": self.topics,
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "stat_fields": self.stat_fields, "stat_schema_source_sha256": stat_hash,
            "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "interpretation": {
                "tracking": "Geometric nearest-polyline distance; shortest-angle interpolated yaw. Not time-indexed tracking. Self-intersections may be ambiguous.",
                "pose": "Localization estimate, not external ground truth. No implicit frame or sensor-to-body transform.",
                "latency": "Receipt time relative to goal receipt, not guaranteed planner-start time.",
                "incumbent": "Only reported first-incumbent duration and counters are available. This recorder does not invent intermediate incumbent events.",
                "outcome": "Pose tolerance observation is not collision-free physical success. Contact/intervention require author annotation.",
                "commands": "cmd_vel may be scaled/sign-flipped by the driver; not measured actuator feedback. Inspect parameter snapshots.",
                "sampling": "All received pose/path messages retained; no decimation. ROS subscriber queue or disk overload may still lose messages.",
                "timeout": "Observation annotation only; never sends stop and never discards later evidence.",
            },
        }
        self.snapshot_parameters("parameters_start.json")
        if not args.no_bag:
            self.bag = rosbag.Bag(str(self.store.output / "evidence.bag"), "w")
        self.write_metadata()
        kinds = {
            "goal": PoseStamped, "pose": Odometry, "odom": Odometry, "lio_odom": Odometry, "cmd": Twist,
            "local_cmd": Twist, "global_path": RosPath, "coarse_path": RosPath,
            "local_path": RosPath, "risk": RiskAwareGlobalReference,
            "stats": Float64MultiArray, "planning_time": Float64, "status": String,
            "replan": Bool, "map": OccupancyGrid, "planner_map": OccupancyGrid,
            "scan": LaserScan, "tf": TFMessage, "tf_static": TFMessage,
            "initialpose": PoseWithCovarianceStamped,
        }
        rospy.on_shutdown(self.shutdown)
        for kind, msg_type in kinds.items():
            self.subscribers.append(rospy.Subscriber(
                self.topics[kind], msg_type, lambda message, k=kind: self.callback(k, message),
                queue_size=200 if kind in ("pose", "odom", "lio_odom", "tf", "cmd", "local_cmd") else 30,
                buff_size=2 ** 24))
        self.timer = rospy.Timer(rospy.Duration(1.0), self.tick)
        rospy.logwarn("Evidence recorder is PASSIVE: it never stops or controls the robot. Ctrl+C only stops recording.")
        rospy.loginfo("Evidence directory: %s ; waiting for a NEW goal", self.store.output)

    def write_json(self, filename, value):
        with (self.store.output / filename).open("w", encoding="utf-8") as stream:
            json.dump(clean_json(value), stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")

    def write_metadata(self):
        self.metadata.update(message_counts=self.store.counts, errors=self.store.errors,
                             fatal_error=self.fatal_error)
        self.write_json("metadata.json", self.metadata)

    def snapshot_parameters(self, filename):
        params = {}
        for namespace in ("/isweep_planner", "/planner_bridge", "/navigation_adapter",
                          "/wheeltec_robot", "/use_sim_time", "/rosdistro"):
            try:
                params[namespace] = self.rospy.get_param(namespace, None)
            except Exception as error:
                params[namespace] = {"read_error": str(error)}
        params["captured_receipt_ros_s"] = self.rospy.get_time()
        self.write_json(filename, params)

    def callback(self, kind, message):
        with self.lock:
            if self.closed:
                return
            try:
                now = self.rospy.Time.now()
                receipt, source = now.to_sec(), stamp_of(message)
                if self.last_receipt is not None and receipt < self.last_receipt - 1e-6:
                    self.store.event("ros_clock_moved_backwards", receipt, previous=self.last_receipt)
                    self.store.finish(self.last_receipt, "ros_clock_reset")
                    self.store.first_reference = self.store.latest_reference = None
                self.last_receipt = receipt
                self.store.counts[kind] = self.store.counts.get(kind, 0) + 1
                if self.bag is not None:
                    header = getattr(message, "_connection_header", None)
                    self.bag.write(self.topics[kind], message, t=now, connection_header=header)
                self.dispatch(kind, message, receipt, source)
            except Exception as error:
                self.fail(error)

    def fail(self, error):
        self.fatal_error = "{}: {}".format(type(error).__name__, error)
        self.rospy.logfatal("Evidence recording failed: %s. This does NOT stop the chassis!", self.fatal_error)
        self.rospy.signal_shutdown("evidence recording failed")

    def dispatch(self, kind, message, receipt, source):
        store = self.store
        if kind == "goal":
            # Ignore a previous latched goal delivered on recorder connection.
            if (getattr(message, "_connection_header", None) or {}).get("latching") == "1" and source and source < receipt - 1.0:
                store.event("old_latched_goal_ignored", receipt, source)
                return
            p = message.pose.position
            yaw = yaw_of(message.pose.orientation)
            if not all(finite(v) for v in (p.x, p.y, yaw)) or not message.header.frame_id:
                store.event("invalid_goal_ignored", receipt, source, message=ros_to_plain(message))
                return
            run_id = store.goal(receipt, source, message.header.frame_id, p.x, p.y, yaw)
            self.snapshot_parameters("parameters_{}.json".format(run_id))
            self.rospy.loginfo("Recording %s", run_id)
        elif kind == "pose":
            p = message.pose.pose.position
            store.pose(receipt, source, message.header.frame_id, p.x, p.y,
                       yaw_of(message.pose.pose.orientation))
        elif kind in ("odom", "lio_odom"):
            twist = message.twist.twist
            if kind == "odom":
                store.latest_odom = (receipt, source, math.hypot(twist.linear.x, twist.linear.y), twist.angular.z)
            store.event(kind, receipt, source, frame_id=message.header.frame_id,
                        child_frame_id=message.child_frame_id, twist=ros_to_plain(message.twist))
        elif kind in ("cmd", "local_cmd"):
            store.row("commands.csv", dict(store.timing(receipt, source), kind=kind,
                      linear_x=message.linear.x, linear_y=message.linear.y, angular_z=message.angular.z))
        elif kind in ("global_path", "coarse_path", "local_path"):
            store.path(kind, receipt, source, message.header.frame_id, path_points(message.poses),
                       [pose.header.stamp.to_sec() for pose in message.poses])
        elif kind == "risk":
            ref_id = "risk_{:07d}".format(store.sequence + 1)
            store.event("risk_reference", receipt, source, reference_id=ref_id,
                        frame_id=message.header.frame_id, total_length=message.total_length,
                        num_segments=message.num_segments,
                        num_medium_risk_segments=message.num_medium_risk_segments,
                        num_high_risk_segments=message.num_high_risk_segments)
            for index, point in enumerate(message.points):
                values = {field: getattr(point, field, None) for field in RISK_FIELDS}
                store.row("risk.csv", dict(store.timing(receipt, source), reference_id=ref_id,
                          frame_id=message.header.frame_id, point_index=index, **values))
        elif kind == "stats":
            # Preserve the raw positional vector even on a future schema change.
            matching = len(message.data) == len(self.stat_fields)
            values = dict(zip(self.stat_fields, message.data)) if matching else None
            store.event("planning_stats", receipt, source, raw_values=list(message.data),
                        schema_length_matches=matching, named_values=values)
            if not matching:
                self.rospy.logwarn_throttle(5, "planning_stats schema length mismatch; raw values retained, no named interpretation")
        elif kind in ("status", "planning_time", "replan"):
            store.event(kind, receipt, source, value=message.data)
            if kind == "replan" and store.active:
                if message.data and not store.active["_replan"]:
                    store.active["replan_true_edges"] += 1
                store.active["_replan"] = bool(message.data)
        elif kind in ("map", "planner_map"):
            # Save every distinct map version without duplicating identical updates.
            plain = ros_to_plain(message)
            geometry = {"frame_id": message.header.frame_id,
                        "info": dict(plain["info"]), "data": plain["data"]}
            geometry["info"].pop("map_load_time", None)
            digest = hashlib.sha256(json.dumps(clean_json(geometry), sort_keys=True).encode()).hexdigest()
            if store.map_hashes.get(kind) != digest:
                filename = "maps/{}_{}.json".format(kind, digest[:16])
                self.write_json(filename, plain)
                store.map_hashes[kind] = digest
                store.event("map_snapshot", receipt, source, map_kind=kind, path=filename, sha256=digest)
        elif kind == "initialpose":
            store.event(kind, receipt, source, message=ros_to_plain(message))
        # TF and scan remain in bag; no high-volume duplicate JSON conversion.

    def tick(self, _event):
        with self.lock:
            if self.closed:
                return
            try:
                self.store.tick(self.rospy.get_time())
                self.write_metadata()
            except Exception as error:
                self.fail(error)

    def shutdown(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            errors = []
            for operation in (lambda: self.store.close(self.rospy.get_time()),
                              lambda: self.bag.close() if self.bag is not None else None,
                              lambda: self.snapshot_parameters("parameters_end.json")):
                try:
                    operation()
                except Exception as error:
                    errors.append(str(error))
            required = ("goal", "pose", "odom", "global_path", "risk", "stats", "cmd")
            self.metadata.update(ended_utc=datetime.now(timezone.utc).isoformat(),
                                 wall_duration_s=time.time() - self.started_wall,
                                 missing_topics=[self.topics[k] for k in required if not self.store.counts.get(k)],
                                 shutdown_errors=errors)
            try:
                self.write_metadata()
            except Exception as error:
                errors.append(str(error))
            if errors:
                self.fatal_error = self.fatal_error or "; ".join(errors)
                self.rospy.logerr("Evidence close errors: %s", errors)


def arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="NEW session directory; existing directories are refused")
    parser.add_argument("--mode", default="isweep_v3", help="Recorded label only, does not switch the planner")
    parser.add_argument("--position-tolerance", type=float, default=0.10)
    parser.add_argument("--yaw-tolerance", type=float, default=0.20)
    parser.add_argument("--dwell-time", type=float, default=1.0)
    parser.add_argument("--pose-max-gap", type=float, default=0.5,
                        help="Reset dwell across pose gaps; odometry freshness limit (s)")
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="Observation deadline annotation; 0 disables. NEVER stops chassis or recording")
    parser.add_argument("--no-bag", action="store_true", help="Disable supplemental evidence.bag; CSV/JSON still saved")
    for kind in ("pose", "odom", "goal", "map"):
        parser.add_argument("--{}-topic".format(kind), default=TOPICS[kind])
    args = parser.parse_args(argv)
    for field in ("position_tolerance", "yaw_tolerance", "dwell_time", "timeout", "pose_max_gap"):
        value = getattr(args, field)
        if not math.isfinite(value) or value < 0 or (field == "pose_max_gap" and value == 0):
            parser.error("--{} must be finite and nonnegative (pose-max-gap > 0)".format(field.replace("_", "-")))
    return args


def main(argv=None):
    # rospy remappings are permitted; no ROS imports until after --help handling.
    argv = sys.argv[1:] if argv is None else argv
    args = arguments([arg for arg in argv if ":=" not in arg])
    import rospy
    rospy.init_node("planning_evidence_recorder", anonymous=True)
    try:
        recorder = RosRecorder(args)
    except Exception as error:
        rospy.logfatal("Cannot start evidence recorder: %s", error)
        return 2
    try:
        rospy.spin()
    finally:
        recorder.shutdown()
    return 2 if recorder.fatal_error else 0


if __name__ == "__main__":
    sys.exit(main())
