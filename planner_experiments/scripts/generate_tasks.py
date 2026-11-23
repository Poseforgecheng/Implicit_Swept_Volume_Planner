#!/usr/bin/env python3
"""Generate deterministic, footprint-valid task files."""

import argparse
import math
import os
import pathlib
import tempfile
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np
from PIL import Image
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True)
    parser.add_argument("--robot", required=True)
    parser.add_argument("--num-tasks", type=int, default=30)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--output", required=True)
    parser.add_argument("--min-distance", type=float, default=2.0)
    parser.add_argument("--yaw-mode", choices=["random", "cardinal", "zero"],
                        default="random")
    parser.add_argument("--different-topology-direction", action="store_true")
    return parser.parse_args()


def point_in_polygon(x: float, y: float,
                     polygon: Sequence[Tuple[float, float]]) -> bool:
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def body_samples(polygon: Sequence[Tuple[float, float]], step: float):
    samples = []
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        length = math.hypot(second[0] - first[0], second[1] - first[1])
        count = max(1, math.ceil(length / step))
        for offset in range(count):
            ratio = offset / count
            samples.append((first[0] + ratio * (second[0] - first[0]),
                            first[1] + ratio * (second[1] - first[1])))
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    for x in np.arange(min(xs), max(xs) + step, step):
        for y in np.arange(min(ys), max(ys) + step, step):
            if point_in_polygon(float(x), float(y), polygon):
                samples.append((float(x), float(y)))
    return samples


def generate_task_document(map_path: pathlib.Path, robot_path: pathlib.Path,
                           num_tasks: int = 30, seed: int = 1,
                           min_distance: float = 2.0,
                           yaw_mode: str = "random",
                           different_topology_direction: bool = False,
                           map_name: str = "") -> Dict[str, Any]:
    """Generate deterministic task data without writing it.

    The caller supplies the final map/robot references when serializing.  This
    lets the batch matrix generator create tasks inside its own atomic batch
    directory while reusing exactly the same validity checks as this CLI.
    """
    map_path = pathlib.Path(map_path).resolve()
    robot_path = pathlib.Path(robot_path).resolve()
    if num_tasks <= 0:
        raise ValueError("num_tasks must be positive")
    if min_distance < 0.0:
        raise ValueError("min_distance must be non-negative")
    if yaw_mode not in {"random", "cardinal", "zero"}:
        raise ValueError(f"unsupported yaw_mode: {yaw_mode}")
    map_data = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    robot_data = yaml.safe_load(robot_path.read_text(encoding="utf-8"))
    image_path = (map_path.parent / map_data["image"]).resolve()
    pixels = np.asarray(Image.open(image_path).convert("L"))
    normalized = np.flipud(pixels).astype(float) / 255.0
    occupancy_probability = (
        normalized if int(map_data.get("negate", 0))
        else 1.0 - normalized)
    # Formal task endpoints must be known free.  Treat the map_server
    # indeterminate band as blocked instead of silently sampling starts/goals
    # in unknown space.
    blocked = occupancy_probability >= float(
        map_data.get("free_thresh", 0.196))
    resolution = float(map_data["resolution"])
    origin = map_data.get("origin", [0.0, 0.0, 0.0])
    polygon = [tuple(map(float, point)) for point in robot_data["footprint"]]
    samples = body_samples(polygon, min(0.01, 0.5 * resolution))
    rng = np.random.default_rng(seed)

    def valid(pose: Tuple[float, float, float]) -> bool:
        c, s = math.cos(pose[2]), math.sin(pose[2])
        for bx, by in samples:
            x = pose[0] + c * bx - s * by
            y = pose[1] + s * bx + c * by
            gx = math.floor((x - origin[0]) / resolution)
            gy = math.floor((y - origin[1]) / resolution)
            if gx < 0 or gy < 0 or gx >= blocked.shape[1] or gy >= blocked.shape[0]:
                return False
            if blocked[gy, gx]:
                return False
        return True

    def yaw() -> float:
        if yaw_mode == "zero":
            return 0.0
        if yaw_mode == "cardinal":
            return float(rng.choice([0.0, math.pi / 2, math.pi, -math.pi / 2]))
        return float(rng.uniform(-math.pi, math.pi))

    tasks: List[dict] = []
    task_keys = set()
    attempts = 0
    max_attempts = max(10000, num_tasks * 2000)
    while len(tasks) < num_tasks and attempts < max_attempts:
        attempts += 1
        start = (float(rng.uniform(origin[0], origin[0] + blocked.shape[1] * resolution)),
                 float(rng.uniform(origin[1], origin[1] + blocked.shape[0] * resolution)), yaw())
        goal = (float(rng.uniform(origin[0], origin[0] + blocked.shape[1] * resolution)),
                float(rng.uniform(origin[1], origin[1] + blocked.shape[0] * resolution)), yaw())
        if math.hypot(goal[0] - start[0], goal[1] - start[1]) < min_distance:
            continue
        if different_topology_direction and \
                (goal[0] - start[0]) * math.cos(start[2]) >= 0:
            continue
        if not valid(start) or not valid(goal):
            continue
        key = tuple(round(value, 12) for value in start + goal)
        if key in task_keys:
            continue
        task_keys.add(key)
        tasks.append({"id": f"task_{len(tasks) + 1:03d}",
                      "start": list(start), "goal": list(goal)})
    if len(tasks) != num_tasks:
        raise RuntimeError(f"generated only {len(tasks)}/{num_tasks} valid tasks")
    inferred_map_name = map_path.parent.name if map_path.stem == "map" else map_path.stem
    return {
        "map": map_name or inferred_map_name,
        "robot_shape": robot_data["name"],
        "generation": {"seed": seed, "num_tasks": num_tasks,
                       "min_distance": min_distance,
                       "yaw_mode": yaw_mode,
                       "different_topology_direction":
                           different_topology_direction},
        "tasks": tasks,
    }


def write_task_document(document: Dict[str, Any], output_path: pathlib.Path,
                        map_path: pathlib.Path,
                        robot_path: pathlib.Path) -> pathlib.Path:
    output_path = pathlib.Path(output_path).resolve()
    output = dict(document)
    output["map_yaml"] = os.path.relpath(pathlib.Path(map_path).resolve(),
                                         output_path.parent)
    output["robot_yaml"] = os.path.relpath(pathlib.Path(robot_path).resolve(),
                                           output_path.parent)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    serialized = yaml.safe_dump(output, sort_keys=False)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=output_path.name + ".", suffix=".tmp",
        dir=str(output_path.parent))
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(serialized)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, output_path)
    except Exception:
        try:
            os.unlink(temporary_name)
        except OSError:
            pass
        raise
    return output_path


def main() -> int:
    args = parse_args()
    map_path = pathlib.Path(args.map).resolve()
    robot_path = pathlib.Path(args.robot).resolve()
    try:
        document = generate_task_document(
            map_path, robot_path, num_tasks=args.num_tasks, seed=args.seed,
            min_distance=args.min_distance, yaw_mode=args.yaw_mode,
            different_topology_direction=args.different_topology_direction)
    except (OSError, ValueError, RuntimeError) as exception:
        raise SystemExit(str(exception))
    output_path = write_task_document(
        document, pathlib.Path(args.output), map_path, robot_path)
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
