#!/usr/bin/env python3
"""Run paper-facing planners sequentially in one reproducible IR-SIM case."""

import argparse
import csv
import json
import math
import os
import pathlib
import signal
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

import rospy
from isweep_planner.msg import RiskAwareGlobalReference
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64, Float64MultiArray, String


METHODS = ("current_full", "full_r2", "full_strict", "hybrid_astar")
BAG_TOPICS = (
    "/prior_map",
    "/isweep_planner/map",
    "/tf",
    "/tf_static",
    "/initialpose",
    "/move_base_simple/goal",
    "/isweep_planner/planner_method",
    "/isweep_planner/trajectory",
    "/isweep_planner/trajectory_astar",
    "/isweep_planner/local_trajectory",
    "/isweep_planner/local_cmd",
    "/isweep_planner/local_planner_status",
    "/isweep_planner/replan_request",
    "/isweep_planner/markers",
    "/isweep_planner/planning_time",
    "/isweep_planner/planning_stats",
    "/isweep_planner/risk_aware_global_reference",
    "/irsim/odom",
    "/irsim/localization",
    "/irsim/cmd_vel",
    "/irsim/scan",
    "/irsim/actual_path",
    "/irsim/status",
    "/irsim/collision",
    "/irsim/arrived",
    "/irsim/metadata",
)


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def path_rows(message):
    rows = []
    for index, stamped in enumerate(message.poses):
        rows.append(
            {
                "index": index,
                "stamp": stamped.header.stamp.to_sec(),
                "x": stamped.pose.position.x,
                "y": stamped.pose.position.y,
                "yaw": yaw_from_quaternion(stamped.pose.orientation),
            }
        )
    return rows


def path_length(rows):
    return sum(
        math.hypot(current["x"] - previous["x"],
                   current["y"] - previous["y"])
        for previous, current in zip(rows, rows[1:])
    )


def write_path(path, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=("index", "stamp", "x", "y", "yaw")
        )
        writer.writeheader()
        writer.writerows(rows)


def risk_reference_rows(message):
    return [
        {
            "index": index,
            "x": point.x,
            "y": point.y,
            "yaw": point.yaw,
            "risk_level": int(point.risk_level),
            "risk_score": point.risk_score,
            "segment_id": int(point.segment_id),
            "s": point.s,
        }
        for index, point in enumerate(message.points)
    ]


def write_risk_reference(path, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "index", "x", "y", "yaw", "risk_level", "risk_score",
                "segment_id", "s",
            ),
        )
        writer.writeheader()
        writer.writerows(rows)


def terminate_process(process, timeout=8.0):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=timeout)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=3.0)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        pass


def unused_tcp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as candidate:
        candidate.bind(("127.0.0.1", 0))
        return candidate.getsockname()[1]


def wait_for_master(port, process, timeout=15.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("roscore exited before becoming ready")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.settimeout(0.2)
            if probe.connect_ex(("127.0.0.1", port)) == 0:
                return
        time.sleep(0.1)
    raise RuntimeError("timed out waiting for isolated roscore")


class RunState:
    def __init__(self, method):
        self.method = method
        self.started_monotonic = time.monotonic()
        self.first_plan = None
        self.latest_plan = None
        self.coarse_path = None
        self.risk_reference = None
        self.planned_message_received = False
        self.actual_path = []
        self.planning_time_s = None
        self.planning_stats = []
        self.latest_local_status = ""
        self.replan_request_count = 0
        self.last_replan_request = False
        self.status = {}
        self.metadata = {}
        self.observed_method = ""
        self.arrived = False
        self.collision = False
        self.planning_completed_monotonic = None


class ComparisonCollector:
    def __init__(self):
        self.lock = threading.RLock()
        self.active = None
        rospy.Subscriber(
            "/isweep_planner/trajectory", Path, self._planned_path,
            queue_size=2,
        )
        rospy.Subscriber(
            "/isweep_planner/trajectory_astar", Path, self._coarse_path,
            queue_size=2,
        )
        rospy.Subscriber(
            "/irsim/actual_path", Path, self._actual_path, queue_size=2
        )
        rospy.Subscriber(
            "/isweep_planner/planning_time", Float64,
            self._planning_time, queue_size=2,
        )
        rospy.Subscriber(
            "/isweep_planner/planning_stats", Float64MultiArray,
            self._planning_stats, queue_size=2,
        )
        rospy.Subscriber(
            "/isweep_planner/risk_aware_global_reference",
            RiskAwareGlobalReference, self._risk_reference, queue_size=2,
        )
        rospy.Subscriber(
            "/isweep_planner/local_planner_status", String,
            self._local_status, queue_size=5,
        )
        rospy.Subscriber(
            "/isweep_planner/replan_request", Bool,
            self._replan_request, queue_size=5,
        )
        rospy.Subscriber("/irsim/status", String, self._status, queue_size=5)
        rospy.Subscriber(
            "/irsim/metadata", String, self._metadata, queue_size=2
        )
        rospy.Subscriber(
            "/isweep_planner/planner_method", String, self._method,
            queue_size=2,
        )
        rospy.Subscriber("/irsim/arrived", Bool, self._arrived, queue_size=2)
        rospy.Subscriber(
            "/irsim/collision", Bool, self._collision, queue_size=2
        )

    def begin(self, method):
        with self.lock:
            self.active = RunState(method)
            return self.active

    def snapshot(self):
        with self.lock:
            return self.active

    def _planned_path(self, message):
        with self.lock:
            if self.active is None:
                return
            rows = path_rows(message)
            self.active.planned_message_received = True
            self.active.latest_plan = rows
            if rows and self.active.first_plan is None:
                self.active.first_plan = rows

    def _coarse_path(self, message):
        with self.lock:
            if self.active is None:
                return
            rows = path_rows(message)
            if rows:
                self.active.coarse_path = rows

    def _actual_path(self, message):
        with self.lock:
            if self.active is not None:
                self.active.actual_path = path_rows(message)

    def _planning_time(self, message):
        with self.lock:
            if self.active is not None:
                self.active.planning_time_s = float(message.data)
                if self.active.planning_completed_monotonic is None:
                    self.active.planning_completed_monotonic = time.monotonic()

    def _planning_stats(self, message):
        with self.lock:
            if self.active is not None:
                self.active.planning_stats = list(message.data)

    def _risk_reference(self, message):
        with self.lock:
            if self.active is not None and message.points:
                self.active.risk_reference = risk_reference_rows(message)

    def _local_status(self, message):
        with self.lock:
            if self.active is not None:
                self.active.latest_local_status = message.data

    def _replan_request(self, message):
        with self.lock:
            if self.active is None:
                return
            requested = bool(message.data)
            if requested and not self.active.last_replan_request:
                self.active.replan_request_count += 1
            self.active.last_replan_request = requested

    def _status(self, message):
        with self.lock:
            if self.active is None:
                return
            try:
                self.active.status = json.loads(message.data)
            except (TypeError, ValueError):
                self.active.status = {"raw": message.data}

    def _metadata(self, message):
        with self.lock:
            if self.active is None:
                return
            try:
                self.active.metadata = json.loads(message.data)
            except (TypeError, ValueError):
                self.active.metadata = {"raw": message.data}

    def _method(self, message):
        with self.lock:
            if self.active is not None:
                self.active.observed_method = message.data

    def _arrived(self, message):
        with self.lock:
            if self.active is not None:
                self.active.arrived = self.active.arrived or bool(message.data)

    def _collision(self, message):
        with self.lock:
            if self.active is not None:
                self.active.collision = self.active.collision or bool(message.data)


def launch_arguments(arguments, method):
    start = arguments.start
    goal = arguments.goal
    return [
        "roslaunch",
        "irsim_bridge",
        "isweep_v3_irsim_rviz.launch",
        "rviz:=false",
        "display_irsim:=false",
        "planner_method:={}".format(method),
        "map_file:={}".format(arguments.map_file),
        "seed:={}".format(arguments.seed),
        "step_time:={}".format(arguments.step_time),
        "start_x:={}".format(start[0]),
        "start_y:={}".format(start[1]),
        "start_yaw:={}".format(start[2]),
        "goal_x:={}".format(goal[0]),
        "goal_y:={}".format(goal[1]),
        "goal_yaw:={}".format(goal[2]),
        "robot_length:={}".format(arguments.robot_length),
        "robot_width:={}".format(arguments.robot_width),
        "wheelbase:={}".format(arguments.wheelbase),
        "min_turn_radius:={}".format(arguments.min_turn_radius),
        "max_linear_speed:={}".format(arguments.max_linear_speed),
        "max_yaw_rate:={}".format(arguments.max_yaw_rate),
        "hybrid_max_expansions:={}".format(
            arguments.hybrid_max_expansions
        ),
        "hybrid_fast_pass_max_expansions:={}".format(
            arguments.hybrid_fast_pass_max_expansions
        ),
        "hybrid_steer_samples:={}".format(arguments.hybrid_steer_samples),
        "hybrid_heuristic_weight:={}".format(
            arguments.hybrid_heuristic_weight
        ),
        "goal_position_tolerance:={}".format(
            arguments.position_tolerance
        ),
        "goal_yaw_tolerance:={}".format(arguments.yaw_tolerance),
    ]


def run_method(arguments, environment, collector, method, output_root):
    method_directory = output_root / method
    method_directory.mkdir(parents=True, exist_ok=True)
    state = collector.begin(method)
    launch_log_path = method_directory / "roslaunch.log"
    bag_log_path = method_directory / "rosbag.log"
    launch_log = launch_log_path.open("w", encoding="utf-8")
    launch_process = subprocess.Popen(
        launch_arguments(arguments, method),
        env=environment,
        stdout=launch_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )
    bag_process = None
    bag_log = None
    if not arguments.no_bag:
        bag_log = bag_log_path.open("w", encoding="utf-8")
        bag_process = subprocess.Popen(
            ["rosbag", "record", "--lz4", "-O",
             str(method_directory / "evidence.bag")] + list(BAG_TOPICS),
            env=environment,
            stdout=bag_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )

    outcome = "timeout"
    detail = "execution timeout"
    deadline = time.monotonic() + arguments.timeout
    try:
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if launch_process.poll() is not None:
                outcome = "infrastructure_failure"
                detail = "roslaunch exited with code {}".format(
                    launch_process.returncode
                )
                break
            current = collector.snapshot()
            if current.collision:
                outcome = "collision"
                detail = "IR-SIM collision truth became true"
                break
            if current.arrived:
                outcome = "success"
                detail = "IR-SIM position and yaw gates passed"
                break
            if (
                current.planning_completed_monotonic is not None
                and current.planned_message_received
                and not current.first_plan
                and time.monotonic() - current.planning_completed_monotonic
                >= arguments.planning_failure_grace
            ):
                outcome = "planning_failure"
                detail = "planner completed without a non-empty trajectory"
                break
            time.sleep(0.05)
    finally:
        terminate_process(bag_process)
        terminate_process(launch_process)
        launch_log.close()
        if bag_log is not None:
            bag_log.close()

    final_state = collector.snapshot()
    planned = final_state.first_plan or []
    latest_planned = final_state.latest_plan or []
    actual = final_state.actual_path or []
    coarse = final_state.coarse_path or []
    risk_reference = final_state.risk_reference or []
    write_path(method_directory / "planned.csv", planned)
    write_path(method_directory / "planned_latest.csv", latest_planned)
    write_path(method_directory / "coarse.csv", coarse)
    write_path(method_directory / "actual.csv", actual)
    write_risk_reference(
        method_directory / "risk_reference.csv", risk_reference
    )
    (method_directory / "status.json").write_text(
        json.dumps(final_state.status, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (method_directory / "metadata.json").write_text(
        json.dumps(final_state.metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (method_directory / "planning_stats.json").write_text(
        json.dumps(final_state.planning_stats, indent=2) + "\n",
        encoding="utf-8",
    )
    (method_directory / "local_status.json").write_text(
        json.dumps(
            {
                "final": final_state.latest_local_status,
                "replan_request_count": final_state.replan_request_count,
            },
            indent=2,
            sort_keys=True,
        ) + "\n",
        encoding="utf-8",
    )

    status = final_state.status
    metadata = final_state.metadata
    simulation_time = None
    if status.get("step") is not None:
        simulation_time = float(status["step"]) * float(
            metadata.get("step_time_s", arguments.step_time)
        )
    return {
        "method": method,
        "outcome": outcome,
        "detail": detail,
        "planner_method_observed": final_state.observed_method,
        "method_match": final_state.observed_method == method,
        "planning_time_s": final_state.planning_time_s,
        "simulation_time_s": simulation_time,
        "planned_points": len(planned),
        "planned_length_m": path_length(planned),
        "coarse_points": len(coarse),
        "coarse_length_m": path_length(coarse),
        "risk_reference_points": len(risk_reference),
        "actual_points": len(actual),
        "actual_length_m": path_length(actual),
        "position_error_m": status.get("position_error_m"),
        "yaw_error_rad": status.get("yaw_error_rad"),
        "collision": bool(final_state.collision),
        "arrived": bool(final_state.arrived),
        "final_local_status": final_state.latest_local_status,
        "replan_request_count": final_state.replan_request_count,
        "map_checksum": metadata.get("map_checksum", ""),
        "seed": metadata.get("seed", arguments.seed),
        "planned_csv": str(method_directory / "planned.csv"),
        "coarse_csv": str(method_directory / "coarse.csv"),
        "actual_csv": str(method_directory / "actual.csv"),
        "bag": "" if arguments.no_bag else str(method_directory / "evidence.bag"),
        "roslaunch_log": str(launch_log_path),
    }


def write_summary(output_root, rows):
    fields = list(rows[0].keys()) if rows else []
    with (output_root / "summary.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (output_root / "summary.json").write_text(
        json.dumps(rows, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description="Run four planner methods sequentially in an isolated IR-SIM case."
    )
    parser.add_argument("--map-file", required=True)
    parser.add_argument("--start", type=float, nargs=3, required=True,
                        metavar=("X", "Y", "YAW"))
    parser.add_argument("--goal", type=float, nargs=3, required=True,
                        metavar=("X", "Y", "YAW"))
    parser.add_argument("--output", required=True)
    parser.add_argument("--methods", nargs="+", choices=METHODS,
                        default=list(METHODS))
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--step-time", type=float, default=0.10)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--planning-failure-grace", type=float, default=2.0)
    parser.add_argument("--position-tolerance", type=float, default=0.10)
    parser.add_argument("--yaw-tolerance", type=float, default=0.20)
    parser.add_argument("--robot-length", type=float, default=0.45)
    parser.add_argument("--robot-width", type=float, default=0.36)
    parser.add_argument("--wheelbase", type=float, default=0.30)
    parser.add_argument("--min-turn-radius", type=float, default=0.30)
    parser.add_argument("--max-linear-speed", type=float, default=0.45)
    parser.add_argument("--max-yaw-rate", type=float, default=1.0)
    parser.add_argument("--hybrid-max-expansions", type=int, default=300000)
    parser.add_argument(
        "--hybrid-fast-pass-max-expansions", type=int, default=30000
    )
    parser.add_argument("--hybrid-steer-samples", type=int, default=7)
    parser.add_argument("--hybrid-heuristic-weight", type=float, default=2.0)
    parser.add_argument("--master-port", type=int, default=0)
    parser.add_argument("--no-bag", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args(argv)


def main():
    arguments = parse_arguments(sys.argv[1:])
    arguments.map_file = str(pathlib.Path(arguments.map_file).expanduser().resolve())
    if not pathlib.Path(arguments.map_file).exists():
        raise SystemExit("map YAML does not exist: {}".format(arguments.map_file))
    output_root = pathlib.Path(arguments.output).expanduser().resolve()
    if output_root.exists() and any(output_root.iterdir()) and not arguments.overwrite:
        raise SystemExit(
            "output directory is not empty; use --overwrite or a new directory: {}".format(
                output_root
            )
        )
    output_root.mkdir(parents=True, exist_ok=True)

    port = arguments.master_port or unused_tcp_port()
    environment = os.environ.copy()
    environment["ROS_MASTER_URI"] = "http://127.0.0.1:{}".format(port)
    environment["ROS_HOSTNAME"] = "127.0.0.1"
    os.environ.update(
        ROS_MASTER_URI=environment["ROS_MASTER_URI"],
        ROS_HOSTNAME=environment["ROS_HOSTNAME"],
    )

    roscore_log = (output_root / "roscore.log").open("w", encoding="utf-8")
    roscore = subprocess.Popen(
        ["roscore", "-p", str(port)], env=environment,
        stdout=roscore_log, stderr=subprocess.STDOUT,
        start_new_session=True, text=True,
    )
    rows = []
    try:
        wait_for_master(port, roscore)
        rospy.init_node(
            "irsim_planner_comparison_runner", anonymous=True,
            disable_signals=True,
        )
        collector = ComparisonCollector()
        manifest = {
            "schema_version": 1,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "map_file": arguments.map_file,
            "start": list(arguments.start),
            "goal": list(arguments.goal),
            "methods": list(arguments.methods),
            "seed": arguments.seed,
            "step_time_s": arguments.step_time,
            "timeout_s": arguments.timeout,
            "position_tolerance_m": arguments.position_tolerance,
            "yaw_tolerance_rad": arguments.yaw_tolerance,
            "hybrid_astar": {
                "max_expansions": arguments.hybrid_max_expansions,
                "fast_pass_max_expansions":
                    arguments.hybrid_fast_pass_max_expansions,
                "steer_samples": arguments.hybrid_steer_samples,
                "heuristic_weight": arguments.hybrid_heuristic_weight,
            },
            "robot": {
                "length_m": arguments.robot_length,
                "width_m": arguments.robot_width,
                "wheelbase_m": arguments.wheelbase,
                "min_turn_radius_m": arguments.min_turn_radius,
                "max_linear_speed_mps": arguments.max_linear_speed,
                "max_yaw_rate_radps": arguments.max_yaw_rate,
            },
            "bags_enabled": not arguments.no_bag,
        }
        (output_root / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        for method in arguments.methods:
            print("[comparison] starting {}".format(method), flush=True)
            row = run_method(
                arguments, environment, collector, method, output_root
            )
            rows.append(row)
            write_summary(output_root, rows)
            print(
                "[comparison] {}: {} planned={:.3f}m actual={:.3f}m".format(
                    method, row["outcome"], row["planned_length_m"],
                    row["actual_length_m"],
                ),
                flush=True,
            )
            time.sleep(1.0)
    finally:
        write_summary(output_root, rows)
        terminate_process(roscore)
        roscore_log.close()
    return 0 if rows and all(row["method_match"] for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
