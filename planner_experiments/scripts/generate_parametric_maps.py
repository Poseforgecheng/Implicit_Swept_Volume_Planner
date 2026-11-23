#!/usr/bin/env python3
"""Generate deterministic ROS map_server PGM/YAML maps."""

import argparse
import hashlib
import json
import math
import pathlib
from typing import Dict

import numpy as np
from PIL import Image, ImageDraw
import yaml


GENERATOR_VERSION = "1.0.0"


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser()
    result.add_argument("--type", required=True,
                        choices=["narrow_door", "l_corner", "s_channel",
                                 "rotation_channel", "dual_route", "random_obstacles"])
    result.add_argument("--resolution", type=float, default=0.05)
    result.add_argument("--width", type=float, default=10.0)
    result.add_argument("--height", type=float, default=8.0)
    result.add_argument("--door-width", type=float, default=0.9)
    result.add_argument("--channel-width", type=float, default=1.2)
    result.add_argument("--obstacle-count", type=int, default=20)
    result.add_argument("--seed", type=int, default=1)
    result.add_argument("--output-dir", required=True)
    return result


def cells(metres: float, resolution: float) -> int:
    return max(1, int(round(metres / resolution)))


def make_grid(args: argparse.Namespace) -> np.ndarray:
    width = cells(args.width, args.resolution)
    height = cells(args.height, args.resolution)
    occupied = np.zeros((height, width), dtype=np.bool_)
    border = max(1, cells(0.15, args.resolution))
    occupied[:border, :] = True
    occupied[-border:, :] = True
    occupied[:, :border] = True
    occupied[:, -border:] = True
    wall = max(1, cells(0.20, args.resolution))

    if args.type == "narrow_door":
        x = width // 2
        occupied[:, x:x + wall] = True
        opening = cells(args.door_width, args.resolution)
        center = height // 2
        occupied[center - opening // 2:center + (opening + 1) // 2,
                 x:x + wall] = False
    elif args.type == "l_corner":
        occupied[:, :] = True
        corridor = cells(args.channel_width, args.resolution)
        y = height // 3
        x = 2 * width // 3
        occupied[y - corridor // 2:y + (corridor + 1) // 2,
                 border:x + corridor // 2] = False
        occupied[y - corridor // 2:height - border,
                 x - corridor // 2:x + (corridor + 1) // 2] = False
    elif args.type == "s_channel":
        for fraction, top_gap in [(0.33, True), (0.66, False)]:
            x = int(width * fraction)
            occupied[:, x:x + wall] = True
            gap = cells(args.channel_width, args.resolution)
            if top_gap:
                occupied[height - border - gap:height - border, x:x + wall] = False
            else:
                occupied[border:border + gap, x:x + wall] = False
    elif args.type == "rotation_channel":
        occupied[:, :] = True
        image = Image.fromarray(np.where(occupied, 0, 255).astype(np.uint8), mode="L")
        draw = ImageDraw.Draw(image)
        points = []
        for i in range(80):
            u = i / 79.0
            x = border + u * (width - 2 * border)
            y = height * (0.5 + 0.28 * math.sin(2.0 * math.pi * u))
            points.append((int(x), int(y)))
        draw.line(points, fill=255, width=cells(args.channel_width, args.resolution),
                  joint="curve")
        occupied = np.asarray(image) < 128
    elif args.type == "dual_route":
        obstacle_width = cells(2.5, args.resolution)
        obstacle_height = cells(3.0, args.resolution)
        x0 = width // 2 - obstacle_width // 2
        y0 = height // 2 - obstacle_height // 2
        occupied[y0:y0 + obstacle_height, x0:x0 + obstacle_width] = True
    elif args.type == "random_obstacles":
        rng = np.random.default_rng(args.seed)
        for _ in range(args.obstacle_count):
            ow = int(rng.integers(max(2, cells(0.25, args.resolution)),
                                  max(3, cells(0.8, args.resolution))))
            oh = int(rng.integers(max(2, cells(0.25, args.resolution)),
                                  max(3, cells(0.8, args.resolution))))
            x0 = int(rng.integers(border, max(border + 1, width - border - ow)))
            y0 = int(rng.integers(border, max(border + 1, height - border - oh)))
            occupied[y0:y0 + oh, x0:x0 + ow] = True
    return occupied


def main() -> int:
    args = parser().parse_args()
    if args.resolution <= 0 or args.width <= 0 or args.height <= 0:
        raise SystemExit("resolution, width, and height must be positive")
    output = pathlib.Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    occupied = make_grid(args)
    # ROS image origin is lower-left; PGM row zero is upper-left.
    pixels = np.where(np.flipud(occupied), 0, 254).astype(np.uint8)
    pgm_path = output / "map.pgm"
    Image.fromarray(pixels, mode="L").save(pgm_path)
    map_hash = hashlib.sha256(pgm_path.read_bytes()).hexdigest()
    geometry: Dict[str, object] = {
        "door_width": args.door_width,
        "channel_width": args.channel_width,
        "obstacle_count": args.obstacle_count,
    }
    metadata = {
        "generator_type": args.type,
        "generator_version": GENERATOR_VERSION,
        "resolution": args.resolution,
        "width": int(occupied.shape[1]),
        "height": int(occupied.shape[0]),
        "all_geometry_parameters": geometry,
        "random_seed": args.seed,
        "map_hash": map_hash,
    }
    map_yaml = {
        "image": "map.pgm", "resolution": args.resolution,
        "origin": [0.0, 0.0, 0.0], "negate": 0,
        "occupied_thresh": 0.65, "free_thresh": 0.196,
        **metadata,
    }
    (output / "map.yaml").write_text(yaml.safe_dump(map_yaml, sort_keys=False),
                                     encoding="utf-8")
    (output / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output / "map.yaml")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
