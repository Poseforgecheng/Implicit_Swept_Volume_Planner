#!/usr/bin/env python3
"""Create traceable statistical summaries and one-figure-per-file plots."""

import argparse
import json
import math
import os
import pathlib
import shlex
import sys
from typing import Dict, Iterable, List, Sequence, Tuple

os.environ.setdefault("MPLCONFIGDIR", "/tmp/planner_experiments_matplotlib")

try:
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd
    from scipy.stats import bootstrap
except ImportError as exception:
    raise SystemExit(
        "analysis dependencies missing; install pandas, numpy, scipy, matplotlib: "
        + str(exception))


def locate(path: pathlib.Path) -> pathlib.Path:
    if path.joinpath("manifest.json").exists():
        return path
    candidates = sorted(p for p in path.iterdir()
                        if p.is_dir() and p.joinpath("manifest.json").exists())
    if not candidates:
        raise FileNotFoundError(f"no batch manifest in {path}")
    return candidates[-1]


def wilson(successes: int, count: int) -> Tuple[float, float]:
    if count == 0:
        return math.nan, math.nan
    z = 1.959963984540054
    p = successes / count
    denominator = 1.0 + z * z / count
    center = (p + z * z / (2.0 * count)) / denominator
    half = z * math.sqrt(p * (1.0 - p) / count + z * z / (4.0 * count * count)) / denominator
    return center - half, center + half


def continuous(values: pd.Series, seed: int = 1) -> Dict[str, float]:
    array = pd.to_numeric(values, errors="coerce").dropna().to_numpy(dtype=float)
    result = {"n": len(array), "mean": math.nan, "std": math.nan,
              "median": math.nan, "p90": math.nan, "p99": math.nan,
              "bootstrap_ci_low": math.nan, "bootstrap_ci_high": math.nan}
    if len(array) == 0:
        return result
    result.update(mean=float(np.mean(array)),
                  std=float(np.std(array, ddof=1)) if len(array) > 1 else 0.0,
                  median=float(np.median(array)), p90=float(np.quantile(array, 0.90)),
                  p99=float(np.quantile(array, 0.99)))
    if len(array) > 1:
        ci = bootstrap((array,), np.mean, confidence_level=0.95,
                       n_resamples=2000, random_state=np.random.default_rng(seed),
                       method="percentile").confidence_interval
        result["bootstrap_ci_low"] = float(ci.low)
        result["bootstrap_ci_high"] = float(ci.high)
    else:
        result["bootstrap_ci_low"] = result["bootstrap_ci_high"] = float(array[0])
    return result


def rate_row(group: pd.DataFrame, field: str) -> Dict[str, float]:
    count = len(group)
    successes = int(group[field].fillna(False).astype(bool).sum())
    low, high = wilson(successes, count)
    return {field + "_successes": successes, field + "_n": count,
            field + "_rate": successes / count if count else math.nan,
            field + "_ci_low": low, field + "_ci_high": high}


def numeric_column(group: pd.DataFrame, field: str, default: float = 0.0) -> pd.Series:
    if field in group:
        return pd.to_numeric(group[field], errors="coerce").fillna(default)
    return pd.Series(default, index=group.index, dtype=float)


def boolean_column(group: pd.DataFrame, field: str) -> pd.Series:
    if field in group:
        return group[field].fillna(False).astype(bool)
    return pd.Series(False, index=group.index, dtype=bool)


def add_validation_time_columns(frame: pd.DataFrame) -> None:
    """Add honest, analysis-only aliases for the two validation clocks.

    Runtime incumbent events are emitted inside the planner.  The experiment
    layer independently validates only the final returned trajectory, so old
    result files cannot reveal when an intermediate incumbent would first have
    passed that validator.  For executable runs, t_total_ms is therefore the
    available completion time of the final externally validated result.
    """
    executable = boolean_column(frame, "executable")
    frame["t_runtime_first_incumbent_ms"] = pd.to_numeric(
        frame.get("t_first_feasible_ms", pd.Series(index=frame.index, dtype=float)),
        errors="coerce").where(executable)
    frame["t_final_external_validation_ms"] = pd.to_numeric(
        frame.get("t_total_ms", pd.Series(index=frame.index, dtype=float)),
        errors="coerce").where(executable)


def recovery_funnel_row(group: pd.DataFrame) -> Dict[str, float]:
    attempts = numeric_column(group, "validation_recovery_attempt_count")
    local_successes = numeric_column(
        group, "validation_recovery_success_count")
    triggered = attempts > 0
    local_success = triggered & (local_successes > 0)
    planner_returned = boolean_column(group, "planner_returned")
    executable = boolean_column(group, "executable")
    returned_after_local_success = local_success & planner_returned
    externally_executable = returned_after_local_success & executable
    search_patch_success = triggered & boolean_column(group, "recovery_success")

    def conditional_rate(numerator: pd.Series,
                         denominator: pd.Series) -> float:
        count = int(denominator.sum())
        return float((numerator & denominator).sum() / count) if count else math.nan

    return {
        "triggered_run_count": int(triggered.sum()),
        "triggered_run_rate": float(triggered.mean()) if len(group) else math.nan,
        "recovery_attempt_count": int(attempts.sum()),
        "local_success_count": int(local_successes.sum()),
        "local_success_run_count": int(local_success.sum()),
        "local_success_given_trigger_rate": conditional_rate(
            local_success, triggered),
        "final_planner_return_after_local_success_count": int(
            returned_after_local_success.sum()),
        "final_planner_return_given_local_success_rate": conditional_rate(
            returned_after_local_success, local_success),
        "final_external_executable_after_local_success_count": int(
            externally_executable.sum()),
        "final_external_executable_given_planner_return_rate": conditional_rate(
            externally_executable, returned_after_local_success),
        "search_patch_success_run_count": int(search_patch_success.sum()),
        "search_patch_final_executable_run_count": int(
            (search_patch_success & executable).sum()),
        "triggered_final_executable_run_count": int((triggered & executable).sum()),
        "triggered_final_executable_rate": conditional_rate(
            executable, triggered),
    }


def save_figure(fig, summary: pathlib.Path, name: str) -> None:
    fig.tight_layout()
    fig.savefig(summary / f"{name}.pdf", bbox_inches="tight")
    fig.savefig(summary / f"{name}.png", dpi=240, bbox_inches="tight")
    plt.close(fig)


def no_data_figure(title: str, message: str):
    fig, axis = plt.subplots()
    axis.set_title(title)
    axis.text(0.5, 0.5, message, ha="center", va="center",
              transform=axis.transAxes)
    axis.set_axis_off()
    return fig


def read_jsonl(path: pathlib.Path) -> pd.DataFrame:
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame()
    return pd.read_json(path, lines=True)


def precision_recall_curve(labels: np.ndarray, scores: np.ndarray):
    if len(labels) == 0:
        return np.array([]), np.array([]), math.nan
    order = np.argsort(-scores, kind="stable")
    ordered = labels[order].astype(bool)
    true_positive = np.cumsum(ordered)
    false_positive = np.cumsum(~ordered)
    positive_count = int(ordered.sum())
    precision = true_positive / np.maximum(1, true_positive + false_positive)
    recall = (true_positive / positive_count if positive_count > 0
              else np.zeros_like(true_positive, dtype=float))
    average_precision = (float(np.sum(
        np.diff(np.concatenate(([0.0], recall))) * precision))
        if positive_count > 0 else math.nan)
    return precision, recall, average_precision


def summary_row(group: pd.DataFrame) -> Dict[str, float]:
    row: Dict[str, float] = {
        "n": len(group),
        "timeout_count": int(group["timeout"].fillna(False).astype(bool).sum()),
    }
    for field in ["planner_returned", "external_footprint_valid",
                  "dynamics_valid", "executable", "timeout"]:
        row.update(rate_row(group, field))
    for field in ["t_total_ms", "t_first_feasible_ms",
                  "t_runtime_first_incumbent_ms",
                  "t_final_external_validation_ms",
                  "minimum_external_clearance_m", "trajectory_length_m",
                  "high_state_ratio", "trajectory_reuse_ratio"]:
        if field in group:
            row.update({field + "_" + key: value
                        for key, value in continuous(group[field]).items()})
    row["strict_solver_calls"] = float(
        numeric_column(group, "strict_solver_call_count").sum())
    row["full_svsdf_calls"] = float(
        numeric_column(group, "full_svsdf_call_count").sum())
    for field in [
            "full_svsdf_initial_assigned_call_count",
            "full_svsdf_initial_escalation_call_count",
            "full_svsdf_frontend_fallback_call_count",
            "full_svsdf_validation_recovery_call_count",
            "full_svsdf_tail_refinement_call_count",
            "full_svsdf_other_call_count",
            "initial_level_upgrade_attempt_count",
            "initial_level_upgrade_success_count",
            "frontend_fallback_attempt_count",
            "frontend_fallback_success_count",
            "validation_recovery_attempt_count",
            "validation_recovery_success_count",
            "post_incumbent_svsdf_call_count",
            "post_incumbent_svsdf_budget_reject_count",
            "tail_refinement_svsdf_budget_reject_count",
            "external_equivalent_gate_attempt_count",
            "external_equivalent_gate_reject_count"]:
        row[field] = float(numeric_column(group, field).sum())
    funnel = recovery_funnel_row(group)
    row.update({"recovery_" + key: value for key, value in funnel.items()})
    return row


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_directory")
    args = parser.parse_args()
    batch = locate(pathlib.Path(args.result_directory).resolve())
    manifest = json.loads(batch.joinpath("manifest.json").read_text(encoding="utf-8"))
    frame = pd.read_json(batch / "runs.jsonl", lines=True)
    add_validation_time_columns(frame)
    incumbent_frame = read_jsonl(batch / "incumbent_events.jsonl")
    risk_ground_truth = read_jsonl(batch / "risk_ground_truth.jsonl")
    summary = batch / "summary"
    summary.mkdir(exist_ok=True)

    index_columns = [
        "run_id", "map_name", "robot_shape", "method_name", "task_id",
        "random_seed", "planner_status", "planner_returned",
        "external_footprint_valid", "dynamics_valid", "boundary_continuous",
        "executable", "degraded", "timeout", "failure_stage",
        "failure_reason", "trajectory_length_m",
        "minimum_external_clearance_m", "t_total_ms",
        "t_runtime_first_incumbent_ms", "t_final_external_validation_ms",
    ]
    for column in index_columns:
        if column not in frame:
            frame[column] = np.nan
    run_index = frame[index_columns].copy()
    run_index.insert(
        0, "run_id_short",
        run_index["run_id"].fillna("").astype(str).str.slice(0, 12))
    run_index["result_file"] = run_index["run_id"].map(
        lambda run_id: str(batch / "runs" / str(run_id) / "result.json"))
    run_index["trajectory_file"] = run_index["run_id"].map(
        lambda run_id: str(batch / "trajectories" / (str(run_id) + ".csv")))
    run_index["log_file"] = run_index["run_id"].map(
        lambda run_id: str(batch / "logs" / (str(run_id) + ".log")))
    run_index["rviz_command"] = run_index["run_id"].map(
        lambda run_id:
        "roslaunch planner_experiments visualize_experiment.launch "
        f"result_directory:={shlex.quote(str(batch))} "
        f"run_id:={shlex.quote(str(run_id))}")
    run_index.sort_values(
        ["map_name", "robot_shape", "method_name", "task_id", "random_seed"],
        inplace=True, kind="stable")
    run_index.to_csv(summary / "run_index.csv", index=False)

    method_rows = []
    for method, group in frame.groupby("method_name", dropna=False):
        row = {"method_name": method}
        row.update(summary_row(group))
        method_rows.append(row)
    pd.DataFrame(method_rows).to_csv(summary / "method_summary.csv", index=False)

    cross_map_rows = []
    for (map_name, method), group in frame.groupby(
            ["map_name", "method_name"], dropna=False):
        row = {
            "map_name": map_name,
            "method_name": method,
            "shape_count": int(group["robot_shape"].nunique(dropna=True)),
        }
        row.update(summary_row(group))
        cross_map_rows.append(row)
    pd.DataFrame(cross_map_rows).to_csv(
        summary / "cross_map_method_summary.csv", index=False)

    cross_map_total_rows = []
    for (map_name, robot_shape, method), group in frame.groupby(
            ["map_name", "robot_shape", "method_name"], dropna=False):
        row = {
            "map_name": map_name,
            "robot_shape": robot_shape,
            "method_name": method,
        }
        row.update(summary_row(group))
        cross_map_total_rows.append(row)
    cross_map_total = pd.DataFrame(cross_map_total_rows)
    cross_map_total.to_csv(summary / "cross_map_total_table.csv", index=False)
    success_matrix = frame.pivot_table(
        index=["map_name", "robot_shape"], columns="method_name",
        values="executable", aggfunc="mean", dropna=False)
    success_matrix.columns = [
        f"{column}_executable_rate" for column in success_matrix.columns]
    success_matrix.reset_index().to_csv(
        summary / "cross_map_executable_rate.csv", index=False)

    def grouped_rates(field: str, output: str) -> None:
        rows = []
        for key, group in frame.groupby(field, dropna=False):
            row = {field: key}
            row.update(rate_row(group, "executable"))
            rows.append(row)
        pd.DataFrame(rows).to_csv(summary / output, index=False)

    grouped_rates("robot_shape", "success_by_shape.csv")
    grouped_rates("map_name", "success_by_map.csv")

    timing_rows = []
    for method, group in frame.groupby("method_name"):
        for field in ["t_total_ms", "t_runtime_first_incumbent_ms",
                      "t_final_external_validation_ms"]:
            values = continuous(group[field]) if field in group else continuous(pd.Series(dtype=float))
            timing_rows.append({"method_name": method, "metric": field, **values})
    pd.DataFrame(timing_rows).to_csv(summary / "timing_quantiles.csv", index=False)

    recovery_rows = []
    for method, group in frame.groupby("method_name"):
        recovery_rows.append({"method_name": method, "n": len(group),
                              **recovery_funnel_row(group),
                              "initial_level_upgrade_attempts": float(numeric_column(
                                  group, "initial_level_upgrade_attempt_count").sum()),
                              "frontend_fallback_attempts": float(numeric_column(
                                  group, "frontend_fallback_attempt_count").sum()),
                              "validation_recovery_attempts": float(numeric_column(
                                  group, "validation_recovery_attempt_count").sum()),
                              "validation_recovery_successes": float(numeric_column(
                                  group, "validation_recovery_success_count").sum()),
                              **{"trajectory_reuse_ratio_" + key: value
                                 for key, value in continuous(group.get(
                                     "trajectory_reuse_ratio", pd.Series(dtype=float))).items()}})
    pd.DataFrame(recovery_rows).to_csv(summary / "recovery_summary.csv", index=False)

    svsdf_source_rows = []
    for method, group in frame.groupby("method_name"):
        svsdf_source_rows.append({
            "method_name": method,
            "n": len(group),
            "total_calls": float(numeric_column(
                group, "full_svsdf_call_count").sum()),
            "initial_assigned_calls": float(numeric_column(
                group, "full_svsdf_initial_assigned_call_count").sum()),
            "initial_escalation_calls": float(numeric_column(
                group, "full_svsdf_initial_escalation_call_count").sum()),
            "frontend_fallback_calls": float(numeric_column(
                group, "full_svsdf_frontend_fallback_call_count").sum()),
            "validation_recovery_calls": float(numeric_column(
                group, "full_svsdf_validation_recovery_call_count").sum()),
            "tail_refinement_calls": float(numeric_column(
                group, "full_svsdf_tail_refinement_call_count").sum()),
            "post_incumbent_calls": float(numeric_column(
                group, "post_incumbent_svsdf_call_count").sum()),
            "post_incumbent_budget_rejects": float(numeric_column(
                group, "post_incumbent_svsdf_budget_reject_count").sum()),
            "tail_budget_rejects": float(numeric_column(
                group, "tail_refinement_svsdf_budget_reject_count").sum()),
            "other_calls": float(numeric_column(
                group, "full_svsdf_other_call_count").sum()),
        })
    pd.DataFrame(svsdf_source_rows).to_csv(
        summary / "svsdf_call_sources.csv", index=False)

    risk_rows = []
    for method, group in frame.groupby("method_name"):
        risk_rows.append({"method_name": method, "n": len(group),
                          **{"high_state_ratio_" + key: value
                             for key, value in continuous(group.get(
                                 "high_state_ratio", pd.Series(dtype=float))).items()},
                          "strict_calls": float(numeric_column(group, "strict_solver_call_count").sum()),
                          "full_svsdf_calls": float(numeric_column(group, "full_svsdf_call_count").sum()),
                          "full_svsdf_initial_assigned_calls": float(numeric_column(
                              group, "full_svsdf_initial_assigned_call_count").sum()),
                          "full_svsdf_initial_escalation_calls": float(numeric_column(
                              group, "full_svsdf_initial_escalation_call_count").sum()),
                          "full_svsdf_frontend_fallback_calls": float(numeric_column(
                              group, "full_svsdf_frontend_fallback_call_count").sum()),
                          "full_svsdf_validation_recovery_calls": float(numeric_column(
                              group, "full_svsdf_validation_recovery_call_count").sum()),
                          "full_svsdf_tail_refinement_calls": float(numeric_column(
                              group, "full_svsdf_tail_refinement_call_count").sum())})
    pd.DataFrame(risk_rows).to_csv(summary / "risk_summary.csv", index=False)
    frame.groupby(["method_name", "failure_stage", "failure_reason"], dropna=False).size().reset_index(
        name="count").to_csv(summary / "failure_reasons.csv", index=False)

    observed_methods = list(frame["method_name"].dropna().unique())
    configured_methods = manifest.get("methods", [])
    methods = [method for method in configured_methods
               if method in observed_methods]
    methods.extend(method for method in observed_methods
                   if method not in methods)
    success = [float(frame.loc[frame.method_name == method, "executable"].mean()) for method in methods]
    intervals = [wilson(int(frame.loc[frame.method_name == method, "executable"].sum()),
                        int((frame.method_name == method).sum())) for method in methods]
    fig, ax = plt.subplots()
    ax.bar(methods, success, yerr=[[success[i] - intervals[i][0] for i in range(len(methods))],
                                   [intervals[i][1] - success[i] for i in range(len(methods))]], capsize=4)
    ax.set_ylabel("Executable success rate")
    ax.set_xlabel("Method")
    ax.tick_params(axis="x", rotation=25)
    for i, method in enumerate(methods):
        ax.text(i, success[i], f"n={(frame.method_name == method).sum()}", ha="center", va="bottom")
    save_figure(fig, summary, "success_rate_by_method")

    map_method = frame.groupby(
        ["map_name", "method_name"], dropna=False)["executable"].agg(
            ["sum", "count", "mean"]).reset_index()
    map_names = list(map_method["map_name"].drop_duplicates())
    rates = np.full((len(map_names), len(methods)), np.nan)
    counts = np.zeros((len(map_names), len(methods)), dtype=int)
    successes_by_cell = np.zeros_like(counts)
    for map_index, map_name in enumerate(map_names):
        for method_index, method in enumerate(methods):
            selected = map_method[
                (map_method["map_name"] == map_name) &
                (map_method["method_name"] == method)]
            if selected.empty:
                continue
            successes_by_cell[map_index, method_index] = int(selected.iloc[0]["sum"])
            counts[map_index, method_index] = int(selected.iloc[0]["count"])
            rates[map_index, method_index] = float(selected.iloc[0]["mean"])
    fig_width = max(8.0, 1.55 * len(methods))
    fig_height = max(5.0, 0.62 * len(map_names))
    fig, ax = plt.subplots(figsize=(fig_width, fig_height))
    image = ax.imshow(rates, vmin=0.0, vmax=1.0, cmap="YlGnBu", aspect="auto")
    for row in range(len(map_names)):
        for column in range(len(methods)):
            if counts[row, column] == 0:
                continue
            color = "white" if rates[row, column] >= 0.62 else "black"
            ax.text(column, row,
                    f"{successes_by_cell[row, column]}/{counts[row, column]}\n"
                    f"{100.0 * rates[row, column]:.0f}%",
                    ha="center", va="center", color=color, fontsize=8)
    ax.set_xticks(np.arange(len(methods)), methods, rotation=25, ha="right")
    ax.set_yticks(np.arange(len(map_names)), map_names)
    ax.set_xlabel("Method")
    ax.set_ylabel("Map")
    colorbar = fig.colorbar(image, ax=ax)
    colorbar.set_label("Executable success rate")
    save_figure(fig, summary, "success_rate_by_map_method")

    fig, ax = plt.subplots()
    values = [frame.loc[(frame.method_name == method) & ~frame.timeout.fillna(False),
                        "t_total_ms"].dropna() for method in methods]
    if any(len(value) for value in values):
        ax.boxplot(values, tick_labels=methods, showfliers=True)
        ax.set_ylabel("Planning time (ms), non-timeout runs")
        ax.tick_params(axis="x", rotation=25)
        ax.set_title("Timeouts: " + str(int(frame.timeout.fillna(False).sum())))
        save_figure(fig, summary, "planning_time_boxplot")
    else:
        save_figure(no_data_figure("Planning time", "No non-timeout samples"), summary,
                    "planning_time_boxplot")

    # t_full_svsdf_ms is nested inside t_strict_solve_ms.  Search-patch
    # t_recovery_ms also overlaps solver timers.  Keep both out of the stack;
    # they remain available as inclusive diagnostic metrics in runs.jsonl.
    timing_fields = ["t_roadmap_ms", "t_dijkstra_ms", "t_shortcut_ms",
                     "t_se2_sequence_ms", "t_low_solve_ms", "t_medium_solve_ms",
                     "t_strict_solve_ms", "t_external_validation_ms"]
    means = np.array([
        [float(numeric_column(
            frame.loc[frame.method_name == method], field).mean())
         for field in timing_fields]
        for method in methods])
    totals = np.array([
        pd.to_numeric(frame.loc[frame.method_name == method, "t_total_ms"],
                      errors="coerce").mean()
        for method in methods])
    measured = np.nansum(means, axis=1)
    other = np.maximum(0.0, totals - measured)
    breakdown = pd.DataFrame(means, columns=timing_fields)
    breakdown.insert(0, "method_name", methods)
    breakdown.insert(1, "n", [int((frame.method_name == method).sum())
                               for method in methods])
    breakdown["t_other_uninstrumented_ms"] = other
    breakdown["t_total_ms"] = totals
    breakdown["nested_t_full_svsdf_ms"] = [
        float(numeric_column(frame.loc[frame.method_name == method],
                             "t_full_svsdf_ms").mean())
        for method in methods]
    breakdown["overlapping_t_recovery_ms"] = [
        float(numeric_column(frame.loc[frame.method_name == method],
                             "t_recovery_ms").mean())
        for method in methods]
    breakdown.to_csv(summary / "planning_time_breakdown.csv", index=False)
    fig, ax = plt.subplots()
    bottom = np.zeros(len(methods))
    stacked_fields = timing_fields + ["t_other_uninstrumented_ms"]
    stacked_values = np.column_stack([means, other])
    for index, field in enumerate(stacked_fields):
        values = np.nan_to_num(stacked_values[:, index])
        label = field.removeprefix("t_").removesuffix("_ms").replace("_", " ")
        ax.bar(methods, values, bottom=bottom, label=label)
        bottom += values
    ax.set_ylabel("Mean non-overlapping run time (ms)")
    ax.tick_params(axis="x", rotation=25)
    ax.legend(fontsize=7)
    ax.set_title("Full-SVSDF is included in strict solve; recovery is not restacked")
    save_figure(fig, summary, "planning_time_breakdown")

    runtime_values = [frame.loc[frame.method_name == method,
                                "t_runtime_first_incumbent_ms"].dropna()
                      for method in methods]
    external_values = [frame.loc[frame.method_name == method,
                                 "t_final_external_validation_ms"].dropna()
                       for method in methods]
    if any(len(value) for value in runtime_values + external_values):
        fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True)
        for axis, values, title in [
                (axes[0], runtime_values, "First runtime-validated incumbent"),
                (axes[1], external_values, "Final externally validated result")]:
            positions = np.arange(1, len(methods) + 1)
            nonempty = [(position, value) for position, value in zip(positions, values)
                        if len(value)]
            if nonempty:
                axis.boxplot([value for _, value in nonempty],
                             positions=[position for position, _ in nonempty],
                             tick_labels=[methods[position - 1]
                                          for position, _ in nonempty],
                             showfliers=True)
            axis.set_title(title)
            axis.tick_params(axis="x", rotation=25)
            axis.set_yscale("log")
        axes[0].set_ylabel("Elapsed time (ms, log scale)")
        save_figure(fig, summary, "runtime_incumbent_vs_external_validation_time")

    def scatter_or_empty(xfield: str, yfield: str, title: str, ylabel: str, name: str):
        if xfield not in frame or yfield not in frame or frame[[xfield, yfield]].dropna().empty:
            save_figure(no_data_figure(title, f"No real {xfield} data"), summary, name)
            return
        fig, ax = plt.subplots()
        for method, group in frame.groupby("method_name"):
            valid = group[[xfield, yfield]].dropna()
            ax.scatter(valid[xfield], valid[yfield], label=f"{method} (n={len(valid)})")
        ax.set_xlabel(xfield.replace("_", " ").title())
        ax.set_ylabel(ylabel)
        ax.legend()
        save_figure(fig, summary, name)

    scatter_or_empty("normalized_width", "executable", "Success vs normalized width",
                     "Executable", "success_vs_normalized_width")
    scatter_or_empty("normalized_width", "high_state_ratio", "High-risk ratio vs width",
                     "HIGH state ratio", "high_risk_ratio_vs_width")

    required_ground_truth_fields = {
        "label_available", "l2_required", "l3_required",
        "assigned_risk_label", "maximum_risk_score", "outcome",
    }
    labeled_ground_truth = pd.DataFrame()
    if required_ground_truth_fields.issubset(risk_ground_truth.columns):
        labeled_ground_truth = risk_ground_truth[
            risk_ground_truth["label_available"].fillna(False).astype(bool)].copy()
    if not labeled_ground_truth.empty:
        labeled_ground_truth["predicted_l2_or_higher"] = (
            labeled_ground_truth["assigned_risk_label"].isin(["MEDIUM", "HIGH"]))
        labeled_ground_truth["predicted_high"] = (
            labeled_ground_truth["assigned_risk_label"] == "HIGH")
        for field in ["l2_required", "l3_required"]:
            labeled_ground_truth[field] = labeled_ground_truth[
                field].fillna(False).astype(bool)

        def save_confusion(label_field: str, predicted_field: str,
                           ylabel: str, name: str):
            matrix = pd.crosstab(
                labeled_ground_truth[label_field],
                labeled_ground_truth[predicted_field]).reindex(
                    index=[False, True], columns=[False, True],
                    fill_value=0).to_numpy()
            fig, ax = plt.subplots()
            image = ax.imshow(matrix, cmap="Blues")
            for (row, column), value in np.ndenumerate(matrix):
                ax.text(column, row, str(value), ha="center", va="center")
            ax.set_xlabel("Predicted upgrade")
            ax.set_ylabel(ylabel)
            fig.colorbar(image, ax=ax)
            save_figure(fig, summary, name)

        save_confusion("l2_required", "predicted_l2_or_higher",
                       "R2 failed, SE(2) executable",
                       "risk_l2_confusion_matrix")
        save_confusion("l3_required", "predicted_high",
                       "SE(2) failed, SVSDF executable",
                       "risk_confusion_matrix")

        scores = pd.to_numeric(
            labeled_ground_truth["maximum_risk_score"], errors="coerce")
        finite = scores.notna().to_numpy()
        score_values = scores.to_numpy(dtype=float)

        def classification_stats(label_field: str, predicted_field: str):
            labels = labeled_ground_truth[label_field].to_numpy(dtype=bool)
            predicted = labeled_ground_truth[predicted_field].to_numpy(dtype=bool)
            precision_curve, recall_curve, average_precision = (
                precision_recall_curve(labels[finite], score_values[finite]))
            true_positive = int((predicted & labels).sum())
            false_positive = int((predicted & ~labels).sum())
            false_negative = int((~predicted & labels).sum())
            binary_precision = (true_positive / (true_positive + false_positive)
                                if true_positive + false_positive else math.nan)
            binary_recall = (true_positive / (true_positive + false_negative)
                             if true_positive + false_negative else math.nan)
            binary_f1 = (2.0 * binary_precision * binary_recall /
                         (binary_precision + binary_recall)
                         if np.isfinite(binary_precision) and
                         np.isfinite(binary_recall) and
                         binary_precision + binary_recall > 0 else math.nan)
            return {
                "positive_count": int(labels.sum()),
                "average_precision": average_precision,
                "binary_precision": binary_precision,
                "binary_recall": binary_recall,
                "binary_f1": binary_f1,
                "precision_curve": precision_curve,
                "recall_curve": recall_curve,
            }

        l2_stats = classification_stats(
            "l2_required", "predicted_l2_or_higher")
        l3_stats = classification_stats("l3_required", "predicted_high")
        risk_summary_record = {
            "n_trials": len(risk_ground_truth),
            "n_labeled": len(labeled_ground_truth),
            **{f"outcome_{key}": int(value) for key, value in
               risk_ground_truth["outcome"].value_counts(dropna=False).items()},
        }
        for prefix, values in [("l2", l2_stats), ("l3", l3_stats)]:
            for key in ["positive_count", "average_precision",
                        "binary_precision", "binary_recall", "binary_f1"]:
                risk_summary_record[f"{prefix}_{key}"] = values[key]
        pd.DataFrame([risk_summary_record]).to_csv(
            summary / "risk_ground_truth_summary.csv", index=False)

        fig, ax = plt.subplots()
        for label, values in [("L2 transition", l2_stats),
                              ("L3 transition", l3_stats)]:
            ax.step(np.concatenate(([0.0], values["recall_curve"])),
                    np.concatenate(([1.0], values["precision_curve"])),
                    where="post", label=(
                        f"{label} (AP={values['average_precision']:.3f})"))
        ax.set_xlabel("Recall")
        ax.set_ylabel("Precision")
        ax.set_title("Three-level independently validated risk PR curves")
        ax.legend()
        save_figure(fig, summary, "risk_precision_recall_curve")

        deciles = labeled_ground_truth.loc[finite, [
            "maximum_risk_score", "l2_required", "l3_required"]].copy()
        try:
            deciles["risk_quantile"] = pd.qcut(
                deciles["maximum_risk_score"], 10, duplicates="drop")
            decile_summary = deciles.groupby(
                "risk_quantile", observed=True).agg(
                    count=("l2_required", "count"),
                    l2_sum=("l2_required", "sum"),
                    l2_mean=("l2_required", "mean"),
                    l3_sum=("l3_required", "sum"),
                    l3_mean=("l3_required", "mean")).reset_index()
            decile_summary["risk_quantile"] = decile_summary[
                "risk_quantile"].astype(str)
        except ValueError:
            decile_summary = pd.DataFrame(
                columns=["risk_quantile", "count", "l2_sum", "l2_mean",
                         "l3_sum", "l3_mean"])
        decile_summary.to_csv(summary / "risk_decile_event_rate.csv", index=False)
    else:
        save_figure(no_data_figure("Risk confusion matrix", "Risk ground truth mode was not logged"),
                    summary, "risk_confusion_matrix")
        save_figure(no_data_figure("L2 risk confusion matrix",
                                   "No independently labeled three-level solves"),
                    summary, "risk_l2_confusion_matrix")
        save_figure(no_data_figure("Risk precision-recall curve",
                                   "No independently labeled three-level solves"),
                    summary, "risk_precision_recall_curve")
        pd.DataFrame(columns=["n_trials", "n_labeled", "l2_positive_count",
                              "l2_average_precision", "l2_binary_precision",
                              "l2_binary_recall", "l2_binary_f1",
                              "l3_positive_count", "l3_average_precision",
                              "l3_binary_precision", "l3_binary_recall",
                              "l3_binary_f1"]).to_csv(
            summary / "risk_ground_truth_summary.csv", index=False)
        pd.DataFrame(columns=["risk_quantile", "count", "l2_sum", "l2_mean",
                              "l3_sum", "l3_mean"]).to_csv(
            summary / "risk_decile_event_rate.csv", index=False)

    if not incumbent_frame.empty and {
            "run_id", "update_index", "elapsed_ms",
            "minimum_clearance_m"}.issubset(incumbent_frame.columns):
        run_metadata = frame[["run_id", "method_name", "planner_returned",
                              "executable"]].drop_duplicates("run_id")
        incumbent_frame = incumbent_frame.merge(
            run_metadata, on="run_id", how="left", validate="many_to_one")
        incumbent_frame["publication_accepted"] = boolean_column(
            incumbent_frame, "planner_returned")
        incumbent_frame["external_executable"] = boolean_column(
            incumbent_frame, "executable")
        incumbent_frame = incumbent_frame.sort_values(
            ["run_id", "update_index", "elapsed_ms"])
        event_summary = incumbent_frame.groupby("run_id", sort=False).agg(
            method_name=("method_name", "first"),
            update_count=("update_index", "count"),
            first_update_ms=("elapsed_ms", "min"),
            last_update_ms=("elapsed_ms", "max"),
            final_clearance_m=("minimum_clearance_m", "last"),
            final_length_m=("trajectory_length_m", "last"),
            final_duration_sec=("trajectory_duration_sec", "last"),
            publication_accepted=("publication_accepted", "first"),
            external_executable=("external_executable", "first"),
        ).reset_index()
        event_summary.to_csv(summary / "incumbent_event_summary.csv", index=False)
        published_events = incumbent_frame[
            incumbent_frame["external_executable"]].copy()
        if not published_events.empty:
            published_events["initial_clearance_m"] = published_events.groupby(
                "run_id")["minimum_clearance_m"].transform("first")
            published_events["clearance_gain_m"] = (
                published_events["minimum_clearance_m"] -
                published_events["initial_clearance_m"])
            update_events = published_events[
                published_events["update_index"] > 0]
            fig, ax = plt.subplots()
            for method in methods:
                selected = update_events[
                    update_events["method_name"] == method]
                if selected.empty:
                    continue
                ax.scatter(selected["elapsed_ms"], selected["clearance_gain_m"],
                           alpha=0.45, s=20,
                           label=f"{method} (runs={selected.run_id.nunique()})")
            if not update_events.empty:
                ax.axhline(0.0, color="black", linewidth=0.7)
                ax.set_xscale("log")
                ax.set_xlabel("Elapsed planning time (ms, log scale)")
                ax.set_ylabel("Clearance gain from first incumbent (m)")
                ax.set_title(
                    "Accepted replacements after the first incumbent; executable runs only")
                ax.legend(fontsize=7)
                save_figure(fig, summary, "incumbent_clearance_over_time")
            else:
                plt.close(fig)
                save_figure(no_data_figure(
                    "Incumbent quality over time",
                    "No executable run replaced its first incumbent"),
                    summary, "incumbent_clearance_over_time")
        else:
            save_figure(no_data_figure(
                "Incumbent quality over time",
                "No incumbent event belongs to an externally executable run"),
                summary, "incumbent_clearance_over_time")
    else:
        pd.DataFrame(columns=["run_id", "method_name", "update_count", "first_update_ms",
                              "last_update_ms", "final_clearance_m",
                              "final_length_m", "final_duration_sec",
                              "publication_accepted", "external_executable"]).to_csv(
            summary / "incumbent_event_summary.csv", index=False)
        save_figure(no_data_figure("Incumbent quality over time",
                                   "No incumbent event stream"), summary,
                    "incumbent_clearance_over_time")

    def bar_metric(field: str, ylabel: str, name: str, aggregate="mean"):
        if field not in frame:
            save_figure(no_data_figure(ylabel, f"No real {field} data"), summary, name)
            return
        grouped = frame.groupby("method_name")[field]
        values = grouped.sum() if aggregate == "sum" else grouped.mean()
        fig, ax = plt.subplots()
        ax.bar(values.index, values.values)
        ax.set_ylabel(ylabel)
        ax.tick_params(axis="x", rotation=25)
        for i, method in enumerate(values.index):
            ax.text(i, values.iloc[i], f"n={(frame.method_name == method).sum()}",
                    ha="center", va="bottom")
        save_figure(fig, summary, name)

    recovery_plot_rows = []
    for method, group in frame.groupby("method_name"):
        values = recovery_funnel_row(group)
        if values["triggered_run_count"] > 0:
            recovery_plot_rows.append((method, values))
    if recovery_plot_rows:
        stages = ["Triggered", "Local validated", "Planner returned",
                  "Externally executable"]
        fig, ax = plt.subplots()
        x = np.arange(len(stages))
        for method, values in recovery_plot_rows:
            counts_for_method = [
                values["triggered_run_count"],
                values["local_success_run_count"],
                values["final_planner_return_after_local_success_count"],
                values["final_external_executable_after_local_success_count"],
            ]
            ax.plot(x, counts_for_method, marker="o", linewidth=2,
                    label=method)
            for position, count in zip(x, counts_for_method):
                ax.text(position, count, str(count), ha="center", va="bottom",
                        fontsize=8)
        ax.set_xticks(x, stages)
        ax.set_ylabel("Run count")
        ax.set_title("Validation-recovery funnel (run-level)")
        ax.legend()
        save_figure(fig, summary, "recovery_funnel")

        fig, ax = plt.subplots()
        width = 0.24
        rate_fields = [
            ("local_success_given_trigger_rate", "Local / triggered"),
            ("final_planner_return_given_local_success_rate", "Returned / local"),
            ("final_external_executable_given_planner_return_rate",
             "External / returned"),
        ]
        method_names = [method for method, _ in recovery_plot_rows]
        positions = np.arange(len(method_names))
        for offset, (field, label) in enumerate(rate_fields):
            plotted = [values[field] for _, values in recovery_plot_rows]
            ax.bar(positions + (offset - 1) * width, plotted, width=width,
                   label=label)
        ax.set_xticks(positions, method_names, rotation=25, ha="right")
        ax.set_ylim(0.0, 1.05)
        ax.set_ylabel("Conditional stage rate")
        ax.set_title("Recovery rates use the preceding funnel stage as denominator")
        ax.legend(fontsize=8)
        save_figure(fig, summary, "recovery_success_rate")
    else:
        save_figure(no_data_figure("Recovery funnel", "No recovery was triggered"),
                    summary, "recovery_funnel")
        save_figure(no_data_figure("Recovery stage rates", "No recovery was triggered"),
                    summary, "recovery_success_rate")

    reuse_groups = []
    for method, group in frame.groupby("method_name"):
        values = pd.to_numeric(group.get(
            "trajectory_reuse_ratio", pd.Series(dtype=float)),
            errors="coerce").dropna()
        if len(values):
            reuse_groups.append((method, values))
    if reuse_groups:
        fig, ax = plt.subplots()
        rng = np.random.default_rng(1)
        for position, (method, values) in enumerate(reuse_groups, start=1):
            jitter = rng.uniform(-0.06, 0.06, size=len(values))
            ax.scatter(np.full(len(values), position) + jitter, values,
                       alpha=0.8)
            median = float(values.median())
            ax.hlines(median, position - 0.18, position + 0.18,
                      colors="black", linewidth=2)
            ax.text(position, min(1.02, float(values.max()) + 0.035),
                    f"n={len(values)}", ha="center", va="bottom")
        ax.set_xticks(range(1, len(reuse_groups) + 1),
                      [method for method, _ in reuse_groups],
                      rotation=25, ha="right")
        ax.set_ylim(-0.03, 1.08)
        ax.set_ylabel("Trajectory reuse ratio")
        ax.set_title("Only runs with a measured local replacement are shown")
        save_figure(fig, summary, "trajectory_reuse_ratio")
    else:
        save_figure(no_data_figure("Trajectory reuse ratio",
                                   "No measured local replacements"),
                    summary, "trajectory_reuse_ratio")
    clearance_groups = []
    for method in methods:
        selected = frame[(frame.method_name == method) &
                         boolean_column(frame, "executable")]
        values = pd.to_numeric(selected.get(
            "minimum_external_clearance_m", pd.Series(dtype=float)),
            errors="coerce").dropna()
        if len(values):
            clearance_groups.append((method, values))
    if clearance_groups:
        fig, ax = plt.subplots()
        ax.boxplot([values for _, values in clearance_groups],
                   tick_labels=[method for method, _ in clearance_groups],
                   showfliers=True)
        ax.set_ylabel("Minimum external clearance (m)")
        ax.set_yscale("log")
        ax.tick_params(axis="x", rotation=25)
        for position, (_, values) in enumerate(clearance_groups, start=1):
            ax.text(position, float(values.max()), f"n={len(values)}",
                    ha="center", va="bottom", fontsize=8)
        ax.set_title("Externally executable trajectories only; log clearance scale")
        save_figure(fig, summary, "minimum_clearance")
    else:
        save_figure(no_data_figure("Minimum clearance",
                                   "No externally executable trajectories"),
                    summary, "minimum_clearance")

    svsdf_phase_fields = [
        ("full_svsdf_initial_assigned_call_count", "Initial assigned"),
        ("full_svsdf_initial_escalation_call_count", "Initial escalation"),
        ("full_svsdf_frontend_fallback_call_count", "Front-end fallback"),
        ("full_svsdf_validation_recovery_call_count", "Local recovery"),
        ("full_svsdf_tail_refinement_call_count", "Tail refinement"),
        ("full_svsdf_other_call_count", "Other"),
    ]
    phase_means = np.array([
        [float(numeric_column(frame.loc[frame.method_name == method], field).mean())
         for field, _ in svsdf_phase_fields]
        for method in methods])
    fig, ax = plt.subplots()
    bottom = np.zeros(len(methods))
    for field_index, (_, label) in enumerate(svsdf_phase_fields):
        values = phase_means[:, field_index]
        ax.bar(methods, values, bottom=bottom, label=label)
        bottom += values
    for position, method in enumerate(methods):
        count = int((frame.method_name == method).sum())
        ax.text(position, bottom[position],
                f"{bottom[position]:.1f}/run\nn={count}",
                ha="center", va="bottom", fontsize=8)
    if len(bottom) and float(np.max(bottom)) > 0.0:
        ax.set_ylim(0.0, 1.18 * float(np.max(bottom)))
    ax.set_ylabel("Mean full-SVSDF calls per run")
    ax.tick_params(axis="x", rotation=25)
    ax.set_title("Full-SVSDF calls separated by invocation phase")
    ax.legend(fontsize=7)
    save_figure(fig, summary, "full_svsdf_calls")

    provenance = {"manifest": str(batch / "manifest.json"),
                  "manifest_configuration_hash": manifest.get("configuration_hash"),
                  "dataset_manifest_hash": manifest.get("dataset_manifest_hash"),
                  "runs_jsonl": str(batch / "runs.jsonl"),
                  "incumbent_events_jsonl": str(batch / "incumbent_events.jsonl"),
                  "risk_ground_truth_jsonl": str(batch / "risk_ground_truth.jsonl")}
    (summary / "analysis_provenance.json").write_text(
        json.dumps(provenance, indent=2) + "\n", encoding="utf-8")
    print(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
