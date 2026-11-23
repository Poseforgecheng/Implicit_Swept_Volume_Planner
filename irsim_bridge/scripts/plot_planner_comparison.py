#!/usr/bin/env python3
"""Create paper-ready planned/executed trajectory comparison figures."""

import argparse
import csv
import json
import math
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from PIL import Image
import yaml


METHOD_STYLE = {
    "current_full": ("Adaptive", "#17365D", "-", 2.2),
    "full_r2": ("Full R²", "#7A6A8A", "-.", 1.6),
    "full_strict": ("Full SVSDF", "#B33A3A", "--", 1.8),
    "hybrid_astar": ("Hybrid A*", "#2F7D5A", ":", 1.9),
}

RISK_STYLE = {
    0: ("Low risk", "#2C6DB2"),
    1: ("Medium risk", "#D99A20"),
    2: ("High risk", "#C4473A"),
}


def resolve_path(value, relative_to):
    path = pathlib.Path(str(value)).expanduser()
    return path.resolve() if path.is_absolute() else (relative_to / path).resolve()


def load_map(map_yaml):
    configuration = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
    image_path = resolve_path(configuration["image"], map_yaml.parent)
    image = Image.open(image_path).convert("L")
    if int(configuration.get("negate", 0)):
        image = Image.eval(image, lambda value: 255 - value)
    resolution = float(configuration["resolution"])
    origin = configuration.get("origin", [0.0, 0.0, 0.0])
    if len(origin) > 2 and abs(float(origin[2])) > 1e-9:
        raise ValueError("comparison plotting currently requires zero map-origin yaw")
    extent = (
        float(origin[0]),
        float(origin[0]) + image.width * resolution,
        float(origin[1]),
        float(origin[1]) + image.height * resolution,
    )
    return image, extent


def load_path(path):
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8") as stream:
        return [
            (float(row["x"]), float(row["y"]), float(row["yaw"]))
            for row in csv.DictReader(stream)
        ]


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def load_last_nonempty_bag_path(path, topic):
    """Recover a diagnostic path from an older result that lacks coarse.csv."""
    if not path.exists():
        return []
    try:
        import rosbag
    except ImportError:
        print(
            "Warning: cannot recover {} from {}; Python rosbag is unavailable"
            .format(topic, path),
            file=sys.stderr,
        )
        return []

    latest = []
    try:
        with rosbag.Bag(str(path), "r") as bag:
            for _, message, _ in bag.read_messages(topics=[topic]):
                points = [
                    (
                        stamped.pose.position.x,
                        stamped.pose.position.y,
                        yaw_from_quaternion(stamped.pose.orientation),
                    )
                    for stamped in message.poses
                ]
                if points:
                    latest = points
    except Exception as error:  # A damaged/incomplete bag should not block plots.
        print(
            "Warning: failed to recover {} from {}: {}"
            .format(topic, path, error),
            file=sys.stderr,
        )
    return latest


def load_recorded_path(root, method, filename, bag_topic):
    points = load_path(root / method / filename)
    if points:
        return points
    return load_last_nonempty_bag_path(
        root / method / "evidence.bag", bag_topic
    )


def load_risk_reference_csv(path):
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8") as stream:
        return [
            (
                float(row["x"]), float(row["y"]),
                int(float(row["risk_level"])),
            )
            for row in csv.DictReader(stream)
        ]


def load_last_bag_risk_reference(path):
    if not path.exists():
        return []
    try:
        import rosbag
    except ImportError:
        print(
            "Warning: cannot recover risk reference from {}; Python rosbag "
            "is unavailable".format(path), file=sys.stderr,
        )
        return []

    latest = []
    try:
        with rosbag.Bag(str(path), "r") as bag:
            for _, message, _ in bag.read_messages(
                    topics=["/isweep_planner/risk_aware_global_reference"]):
                points = [
                    (point.x, point.y, int(point.risk_level))
                    for point in message.points
                ]
                if points:
                    latest = points
    except Exception as error:
        print(
            "Warning: failed to recover risk reference from {}: {}"
            .format(path, error), file=sys.stderr,
        )
    return latest


def load_recorded_risk_reference(root, method):
    points = load_risk_reference_csv(
        root / method / "risk_reference.csv"
    )
    if points:
        return points
    return load_last_bag_risk_reference(root / method / "evidence.bag")


def load_summary(root):
    path = root / "summary.csv"
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as stream:
        return {row["method"]: row for row in csv.DictReader(stream)}


def bounds_for_case(paths, start, goal, map_extent, margin):
    coordinates = [(start[0], start[1]), (goal[0], goal[1])]
    for points in paths:
        coordinates.extend((point[0], point[1]) for point in points)
    minimum_x = min(point[0] for point in coordinates) - margin
    maximum_x = max(point[0] for point in coordinates) + margin
    minimum_y = min(point[1] for point in coordinates) - margin
    maximum_y = max(point[1] for point in coordinates) + margin
    minimum_x = max(map_extent[0], minimum_x)
    maximum_x = min(map_extent[1], maximum_x)
    minimum_y = max(map_extent[2], minimum_y)
    maximum_y = min(map_extent[3], maximum_y)
    if maximum_x - minimum_x < 1.0:
        center = 0.5 * (minimum_x + maximum_x)
        minimum_x, maximum_x = center - 0.5, center + 0.5
    if maximum_y - minimum_y < 1.0:
        center = 0.5 * (minimum_y + maximum_y)
        minimum_y, maximum_y = center - 0.5, center + 0.5
    return minimum_x, maximum_x, minimum_y, maximum_y


def draw_background(axis, image, extent, limits, start, goal):
    axis.imshow(
        image, cmap="gray", origin="upper", extent=extent,
        interpolation="nearest", vmin=0, vmax=255, zorder=0,
    )
    axis.scatter(
        [start[0]], [start[1]], marker="o", s=32,
        facecolor="white", edgecolor="#111111", linewidth=1.1,
        zorder=8,
    )
    axis.scatter(
        [goal[0]], [goal[1]], marker="*", s=74,
        facecolor="#111111", edgecolor="white", linewidth=0.6,
        zorder=8,
    )
    arrow_length = 0.35
    for pose, color in ((start, "#111111"), (goal, "#111111")):
        axis.arrow(
            pose[0], pose[1],
            arrow_length * math.cos(pose[2]),
            arrow_length * math.sin(pose[2]),
            width=0.008, head_width=0.09, head_length=0.12,
            length_includes_head=True, color=color, zorder=7,
        )
    axis.set_xlim(limits[0], limits[1])
    axis.set_ylim(limits[2], limits[3])
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("$x$ (m)")
    axis.set_ylabel("$y$ (m)")
    axis.tick_params(direction="out", length=2.5, width=0.7)
    for spine in axis.spines.values():
        spine.set_linewidth(0.8)


def draw_risk_reference(axis, risk_reference):
    if len(risk_reference) < 2:
        return False
    segments = [
        ((first[0], first[1]), (second[0], second[1]))
        for first, second in zip(risk_reference, risk_reference[1:])
    ]
    colors = [
        RISK_STYLE.get(first[2], RISK_STYLE[2])[1]
        for first in risk_reference[:-1]
    ]
    # A narrow white underlay separates the colored reference from black map
    # boundaries without altering the underlying trajectory geometry.
    axis.add_collection(LineCollection(
        segments, colors="#FFFFFF", linewidths=3.2, alpha=0.92, zorder=3.8,
        capstyle="round", joinstyle="round",
    ))
    axis.add_collection(LineCollection(
        segments, colors=colors, linewidths=2.25, alpha=1.0, zorder=4,
        capstyle="round", joinstyle="round",
    ))
    return True


def plot_method_overlay(
        axis, method, planned, actual, coarse, risk_reference, summary):
    label, color, _, linewidth = METHOD_STYLE[method]
    has_risk_reference = method == "current_full" and len(risk_reference) >= 2
    risk_drawn = has_risk_reference and draw_risk_reference(
        axis, risk_reference
    )
    if planned:
        if not risk_drawn:
            axis.plot(
                [point[0] for point in planned],
                [point[1] for point in planned],
                color=color, linestyle="--",
                linewidth=max(1.2, linewidth - 0.25),
                alpha=0.72, zorder=4,
            )
    elif coarse:
        axis.plot(
            [point[0] for point in coarse],
            [point[1] for point in coarse],
            color="#686F78", linestyle=(0, (5, 2, 1.2, 2)),
            linewidth=1.35, alpha=0.95, zorder=4,
        )
    if actual and not risk_drawn:
        axis.plot(
            [point[0] for point in actual],
            [point[1] for point in actual],
            color=color, linestyle="-",
            linewidth=linewidth, alpha=0.98, zorder=5,
        )
    outcome = summary.get(method, {}).get("outcome", "")
    if not planned:
        if coarse:
            note = "Unvalidated candidate retained"
        else:
            note = "No path candidate available"
        if outcome:
            note += " ({})".format(outcome.replace("_", " "))
        axis.text(
            0.015, 0.025, note, transform=axis.transAxes,
            fontsize=6.5, color="#6B2525", ha="left", va="bottom",
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.80,
                  "pad": 1.5},
            zorder=10,
        )
    axis.set_title(label)


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("result_directory")
    parser.add_argument("--output-prefix", default="trajectory_comparison")
    parser.add_argument(
        "--methods", nargs="+", choices=tuple(METHOD_STYLE),
        help="Optional subset/order of methods to draw",
    )
    parser.add_argument(
        "--columns", type=int, choices=(1, 2), default=None,
        help="Panel columns; default is 1 for one method and 2 otherwise",
    )
    parser.add_argument(
        "--compact", action="store_true",
        help="Reduce repeated labels and vertical whitespace for manuscript use",
    )
    parser.add_argument("--margin", type=float, default=1.0)
    parser.add_argument("--dpi", type=int, default=300)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    root = pathlib.Path(arguments.result_directory).expanduser().resolve()
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    requested_methods = arguments.methods or manifest["methods"]
    methods = [
        method for method in requested_methods
        if method in manifest["methods"] and method in METHOD_STYLE
    ]
    if not methods:
        raise ValueError("no requested method is present in the result directory")
    summary = load_summary(root)
    start = manifest["start"]
    goal = manifest["goal"]
    image, map_extent = load_map(pathlib.Path(manifest["map_file"]))
    planned = {
        method: load_recorded_path(
            root, method, "planned.csv", "/isweep_planner/trajectory"
        )
        for method in methods
    }
    actual = {method: load_path(root / method / "actual.csv") for method in methods}
    coarse = {
        method: load_recorded_path(
            root, method, "coarse.csv", "/isweep_planner/trajectory_astar"
        )
        for method in methods
    }
    risk_reference = {
        method: load_recorded_risk_reference(root, method)
        if method == "current_full" else []
        for method in methods
    }
    all_paths = (
        list(planned.values()) + list(actual.values()) + list(coarse.values())
    )
    limits = bounds_for_case(
        all_paths, start, goal, map_extent, arguments.margin
    )

    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.size": 7.5,
        "axes.titlesize": 8.5,
        "axes.labelsize": 7.5,
        "xtick.labelsize": 6.8,
        "ytick.labelsize": 6.8,
        "legend.fontsize": 7.0,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "svg.fonttype": "none",
    })
    column_count = (
        arguments.columns
        if arguments.columns is not None
        else (2 if len(methods) > 1 else 1)
    )
    row_count = int(math.ceil(len(methods) / float(column_count)))
    crop_aspect = max(1.0, (limits[1] - limits[0]) / (limits[3] - limits[2]))
    panel_height = min(2.55, max(1.05, 3.15 / crop_aspect))
    figure_width = 7.2 if column_count == 2 else 3.45
    figure_height = row_count * (panel_height + 0.42) + 0.38
    if row_count == 1:
        figure_height += 0.48
    elif column_count == 1:
        figure_height += 0.78
    if arguments.compact and column_count == 1:
        panel_height = min(2.25, max(0.90, 3.0 / crop_aspect))
        figure_width = 3.15
        figure_height = row_count * (panel_height + 0.24) + 0.72
    figure, axes = plt.subplots(
        row_count, column_count,
        figsize=(figure_width, figure_height), squeeze=False,
    )
    flat_axes = list(axes.flat)
    for axis_index, (axis, method) in enumerate(zip(flat_axes, methods)):
        draw_background(axis, image, map_extent, limits, start, goal)
        plot_method_overlay(
            axis, method, planned.get(method, []), actual.get(method, []),
            coarse.get(method, []), risk_reference.get(method, []), summary
        )
        if arguments.compact and column_count == 1 and axis_index < len(methods) - 1:
            axis.set_xlabel("")
    for axis in flat_axes[len(methods):]:
        axis.set_visible(False)

    low_handle = Line2D(
        [0], [0], color=RISK_STYLE[0][1], linewidth=2.25,
        label=RISK_STYLE[0][0],
    )
    medium_handle = Line2D(
        [0], [0], color=RISK_STYLE[1][1], linewidth=2.25,
        label=RISK_STYLE[1][0],
    )
    high_handle = Line2D(
        [0], [0], color=RISK_STYLE[2][1], linewidth=2.25,
        label=RISK_STYLE[2][0],
    )
    planned_handle = Line2D(
        [0], [0], color="#17365D", linestyle="--", linewidth=1.5,
        alpha=0.72, label="Planned",
    )
    executed_handle = Line2D(
        [0], [0], color="#555C64", linewidth=1.6, label="Executed",
    )
    candidate_handle = Line2D(
        [0], [0], color="#686F78", linestyle=(0, (5, 2, 1.2, 2)),
        linewidth=1.35, label="Unvalidated candidate",
    )
    start_handle = Line2D(
        [0], [0], marker="o", color="none", markerfacecolor="white",
        markeredgecolor="#111111", markersize=5, label="Start",
    )
    goal_handle = Line2D(
        [0], [0], marker="*", color="none", markerfacecolor="#111111",
        markeredgecolor="#111111", markersize=7, label="Goal",
    )
    if methods == ["current_full"]:
        # Matplotlib fills multi-column legends column-wise. This order yields
        # risk levels on the first row and start/goal on the second.
        legend_handles = [
            low_handle, start_handle, medium_handle, goal_handle, high_handle,
        ]
        legend_columns = 3
    else:
        legend_handles = [
            low_handle, executed_handle, medium_handle, candidate_handle,
            high_handle, start_handle, planned_handle, goal_handle,
        ]
        legend_columns = 2 if column_count == 1 else 4
    if arguments.compact and column_count == 1:
        figure.subplots_adjust(
            left=0.145, right=0.985, top=0.98, bottom=0.18,
            hspace=0.42, wspace=0.18,
        )
    else:
        figure.subplots_adjust(
            left=0.15 if column_count == 1 else 0.075,
            right=0.985, top=0.96 if row_count > 1 else 0.92,
            bottom=(0.34 if row_count == 1 else
                    (0.16 if column_count == 1 else 0.22)),
            hspace=0.60 if column_count == 1 else 0.52,
            wspace=0.18,
        )
    figure.legend(
        handles=legend_handles, loc="lower center", bbox_to_anchor=(0.5, 0.015),
        ncol=legend_columns, frameon=False, handlelength=2.5,
        columnspacing=1.1 if arguments.compact else 1.4,
        fontsize=6.1 if arguments.compact else None,
    )

    prefix = root / arguments.output_prefix
    figure.savefig(str(prefix) + ".png", dpi=arguments.dpi, bbox_inches="tight")
    figure.savefig(str(prefix) + ".pdf", bbox_inches="tight")
    figure.savefig(str(prefix) + ".svg", bbox_inches="tight")
    plt.close(figure)
    print("Wrote {}.[png|pdf|svg]".format(prefix))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
