#!/usr/bin/env python3
"""Replay saved experiment artifacts in RViz without replanning."""

import argparse
import csv
import json
import math
import pathlib

from PIL import Image
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def color(r, g, b, a=1.0):
    return ColorRGBA(r=r, g=g, b=b, a=a)


def point(x, y, z=0.0):
    return Point(x=float(x), y=float(y), z=float(z))


def resolve_path(value, relative_to):
    if value.startswith("package://"):
        import rospkg
        remainder = value[len("package://"):]
        package, separator, suffix = remainder.partition("/")
        if not separator:
            raise ValueError(f"invalid package URI: {value}")
        return (pathlib.Path(rospkg.RosPack().get_path(package)) / suffix).resolve()
    path = pathlib.Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (relative_to / path).resolve()


def find_map_yaml(root, manifest, result):
    maps = manifest.get("map_list", [])
    matching = [
        entry for entry in maps
        if entry.get("hash") == result.get("map_hash")
    ]
    if not matching:
        matching = [
            entry for entry in maps
            if entry.get("name") == result.get("map_name")
        ]
    if matching and matching[0].get("yaml"):
        return resolve_path(str(matching[0]["yaml"]), root)

    tasks_value = result.get("tasks_file")
    if tasks_value:
        tasks_path = resolve_path(str(tasks_value), root)
        tasks = yaml.safe_load(tasks_path.read_text(encoding="utf-8")) or {}
        if tasks.get("map_yaml"):
            return resolve_path(str(tasks["map_yaml"]), tasks_path.parent)
    raise ValueError(
        "the saved run does not contain a resolvable map YAML reference")


def load_occupancy_grid(map_yaml, frame_id):
    configuration = yaml.safe_load(
        map_yaml.read_text(encoding="utf-8")) or {}
    image_path = resolve_path(str(configuration["image"]), map_yaml.parent)
    image = Image.open(image_path).convert("L")
    width, height = image.size
    pixels = list(image.getdata())
    resolution = float(configuration["resolution"])
    negate = bool(int(configuration.get("negate", 0)))
    occupied_threshold = float(configuration.get("occupied_thresh", 0.65))
    free_threshold = float(configuration.get("free_thresh", 0.196))
    mode = str(configuration.get("mode", "trinary")).lower()
    if mode not in {"trinary", "scale"}:
        raise ValueError(
            f"saved-map replay supports trinary/scale mode, not {mode!r}")

    message = OccupancyGrid()
    message.header.frame_id = frame_id
    message.info.width = width
    message.info.height = height
    message.info.resolution = resolution
    origin = configuration.get("origin", [0.0, 0.0, 0.0])
    message.info.origin.position.x = float(origin[0])
    message.info.origin.position.y = float(origin[1])
    yaw = float(origin[2]) if len(origin) > 2 else 0.0
    message.info.origin.orientation.z = math.sin(0.5 * yaw)
    message.info.origin.orientation.w = math.cos(0.5 * yaw)
    data = []
    for map_y in range(height):
        image_y = height - map_y - 1
        for map_x in range(width):
            grayscale = pixels[image_y * width + map_x] / 255.0
            probability = grayscale if negate else 1.0 - grayscale
            if probability > occupied_threshold:
                value = 100
            elif probability < free_threshold:
                value = 0
            elif mode == "scale":
                span = max(1e-12, occupied_threshold - free_threshold)
                value = int(round(
                    99.0 * (probability - free_threshold) / span))
                value = max(1, min(99, value))
            else:
                value = -1
            data.append(value)
    message.data = data
    return message, image_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_directory")
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--frame-id", default="map")
    args = parser.parse_args(rospy.myargv()[1:])
    root = pathlib.Path(args.result_directory).resolve()
    if not root.joinpath("manifest.json").exists():
        candidates = sorted(p for p in root.iterdir()
                            if p.is_dir() and p.joinpath("manifest.json").exists())
        if not candidates:
            raise SystemExit("manifest not found")
        root = candidates[-1]
    manifest = json.loads(
        root.joinpath("manifest.json").read_text(encoding="utf-8"))
    trajectory_path = root / "trajectories" / f"{args.run_id}.csv"
    if not trajectory_path.exists():
        raise SystemExit(f"trajectory not found: {trajectory_path}")
    with trajectory_path.open("r", encoding="utf-8") as stream:
        trajectory = list(csv.DictReader(stream))
    state_metrics = []
    with root.joinpath("state_metrics.jsonl").open("r", encoding="utf-8") as stream:
        for line in stream:
            value = json.loads(line)
            if value.get("run_id") == args.run_id:
                state_metrics.append(value)
    result_path = root / "runs" / args.run_id / "result.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    try:
        map_yaml = find_map_yaml(root, manifest, result)
        occupancy, map_image = load_occupancy_grid(map_yaml, args.frame_id)
    except (OSError, KeyError, ValueError) as exception:
        raise SystemExit(f"could not load saved map: {exception}")

    rospy.init_node("planner_experiment_replay")
    map_publisher = rospy.Publisher(
        "/planner_experiments/replay_map", OccupancyGrid,
        queue_size=1, latch=True)
    publisher = rospy.Publisher("/planner_experiments/replay_markers", MarkerArray,
                                queue_size=1, latch=True)
    markers = MarkerArray()
    line = Marker(header=Header(frame_id=args.frame_id), ns="final_trajectory",
                  id=0, type=Marker.LINE_STRIP, action=Marker.ADD)
    line.scale.x = 0.035
    line.color = color(0.0, 1.0, 1.0, 1.0)
    line.points = [point(row["x"], row["y"], 0.03) for row in trajectory]
    markers.markers.append(line)
    for index, state in enumerate(state_metrics):
        marker = Marker(header=Header(frame_id=args.frame_id), ns="risk_states",
                        id=index, type=Marker.SPHERE, action=Marker.ADD)
        marker.pose.position = point(state["x"], state["y"], 0.05)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.07
        risk_label = state.get("risk_label")
        marker.color = color(1.0, 0.0, 0.0) if risk_label == "HIGH" \
            else (color(1.0, 0.7, 0.0) if risk_label == "MEDIUM" \
                  else color(0.0, 1.0, 0.0))
        markers.markers.append(marker)
        arrow = Marker(header=Header(frame_id=args.frame_id), ns="safe_yaw",
                       id=index, type=Marker.ARROW, action=Marker.ADD)
        arrow.points = [point(state["x"], state["y"], 0.08),
                        point(state["x"] + 0.22 * math.cos(state["selected_yaw"]),
                              state["y"] + 0.22 * math.sin(state["selected_yaw"]), 0.08)]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.015, 0.03, 0.03
        arrow.color = color(0.4, 0.8, 0.4, 0.7)
        markers.markers.append(arrow)
    validation = result.get("validation", {})
    if validation.get("collision_time_sec") is not None:
        marker = Marker(header=Header(frame_id=args.frame_id), ns="collision",
                        id=0, type=Marker.SPHERE, action=Marker.ADD)
        marker.pose.position = point(validation["collision_x"], validation["collision_y"], 0.1)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.25
        marker.color = color(0.05, 0.05, 0.05)
        markers.markers.append(marker)
    occupancy.header.stamp = rospy.Time.now()
    occupancy.info.map_load_time = occupancy.header.stamp
    map_publisher.publish(occupancy)
    rospy.sleep(0.5)
    publisher.publish(markers)
    rospy.loginfo(
        "Published saved map %s (%dx%d at %.3f m/cell) and %d markers for run %s",
        map_image, occupancy.info.width, occupancy.info.height,
        occupancy.info.resolution, len(markers.markers), args.run_id)
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
