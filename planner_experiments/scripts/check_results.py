#!/usr/bin/env python3
"""Validate batch-result invariants without altering results."""

import argparse
import json
import math
import pathlib
import statistics
from collections import Counter
from typing import Any, Dict, List

from _common import atomic_write_text


REQUIRED = [
    "run_id", "configuration_hash", "method_name", "map_name", "map_hash",
    "robot_shape", "task_id",
    "random_seed", "planner_status", "planner_returned", "external_footprint_valid",
    "dynamics_valid", "boundary_continuous", "endpoint_check_performed",
    "goal_reached", "endpoints_match", "executable", "degraded", "timeout",
    "geometry_consistency_passed", "t_total_ms",
    "full_svsdf_initial_assigned_call_count",
    "full_svsdf_initial_escalation_call_count",
    "full_svsdf_frontend_fallback_call_count",
    "full_svsdf_validation_recovery_call_count",
    "full_svsdf_tail_refinement_call_count", "full_svsdf_other_call_count",
    "initial_level_upgrade_attempt_count", "initial_level_upgrade_success_count",
    "frontend_fallback_attempt_count", "frontend_fallback_success_count",
    "validation_recovery_attempt_count", "validation_recovery_success_count",
]
TIME_FIELDS = [
    "t_map_load_ms", "t_esdf_ms", "t_geometry_precompute_ms", "t_roadmap_ms",
    "t_dijkstra_ms", "t_shortcut_ms", "t_path_adjust_ms", "t_se2_sequence_ms",
    "t_risk_classification_ms", "t_low_solve_ms", "t_medium_solve_ms",
    "t_strict_solve_ms",
    "t_full_svsdf_ms", "t_stitch_ms", "t_internal_validation_ms",
    "t_external_validation_ms", "t_recovery_ms", "t_total_ms", "t_first_feasible_ms",
    "t_risk_ground_truth_ms",
]


def batch_path(value: pathlib.Path) -> pathlib.Path:
    if value.joinpath("manifest.json").exists():
        return value
    candidates = sorted(path for path in value.iterdir()
                        if path.is_dir() and path.joinpath("manifest.json").exists())
    if not candidates:
        raise FileNotFoundError(f"no manifest below {value}")
    return candidates[-1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_directory")
    args = parser.parse_args()
    batch = batch_path(pathlib.Path(args.result_directory).resolve())
    manifest = json.loads(
        batch.joinpath("manifest.json").read_text(encoding="utf-8"))
    records: List[Dict[str, Any]] = []
    errors: List[str] = []
    warnings: List[str] = []
    incumbent_last: Dict[str, tuple] = {}
    risk_labeled_counts = Counter()
    risk_l2_required_counts = Counter()
    risk_l3_required_counts = Counter()
    risk_strict_required_counts = Counter()
    with batch.joinpath("runs.jsonl").open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exception:
                errors.append(f"runs.jsonl:{line_number}: invalid JSON: {exception}")
    detail_counts = {}
    for filename, key in [("state_metrics.jsonl", "states"),
                          ("segment_metrics.jsonl", "segments"),
                          ("incumbent_events.jsonl", "incumbents"),
                          ("risk_ground_truth.jsonl", "risk_ground_truth")]:
        counts_for_file = Counter()
        path = batch / filename
        if path.exists():
            for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                if not line.strip():
                    continue
                try:
                    detail = json.loads(line)
                except json.JSONDecodeError as exception:
                    errors.append(f"{filename}:{line_number}: invalid JSON: {exception}")
                    continue
                if "run_id" not in detail:
                    errors.append(f"{filename}:{line_number}: missing run_id")
                    continue
                counts_for_file[detail["run_id"]] += 1
                if key == "incumbents":
                    if not detail.get("internally_validated", False):
                        errors.append(
                            f"{filename}:{line_number}: unvalidated incumbent event")
                    elapsed = detail.get("elapsed_ms")
                    if not isinstance(elapsed, (int, float)) or elapsed < 0:
                        errors.append(
                            f"{filename}:{line_number}: invalid incumbent elapsed_ms")
                    update_index = detail.get("update_index")
                    previous = incumbent_last.get(detail["run_id"])
                    if not isinstance(update_index, int) or update_index < 0:
                        errors.append(
                            f"{filename}:{line_number}: invalid incumbent update_index")
                    elif previous is None and update_index != 0:
                        errors.append(
                            f"{filename}:{line_number}: incumbent stream does not start at zero")
                    elif previous is not None and update_index != previous[0] + 1:
                        errors.append(
                            f"{filename}:{line_number}: non-contiguous incumbent update_index")
                    if previous is not None and isinstance(elapsed, (int, float)) \
                            and elapsed < previous[1]:
                        errors.append(
                            f"{filename}:{line_number}: non-monotonic incumbent elapsed_ms")
                    if isinstance(update_index, int) and isinstance(elapsed, (int, float)):
                        incumbent_last[detail["run_id"]] = (update_index, elapsed)
                if key == "risk_ground_truth":
                    if detail.get("label_available") and not (
                            detail.get("low_solver_attempted") and
                            detail.get("medium_solver_attempted") and
                            detail.get("strict_solver_attempted") and
                            detail.get("full_svsdf_used") and
                            detail.get("full_svsdf_success")):
                        errors.append(
                            f"{filename}:{line_number}: label without complete three-level solve")
                    if detail.get("l2_required") and not (
                            detail.get("label_available") and
                            detail.get("medium_executable") and
                            not detail.get("low_executable")):
                        errors.append(
                            f"{filename}:{line_number}: inconsistent L2-required label")
                    if detail.get("l3_required") and not (
                            detail.get("label_available") and
                            detail.get("strict_executable") and
                            not detail.get("medium_executable")):
                        errors.append(
                            f"{filename}:{line_number}: inconsistent L3-required label")
                    if detail.get("strict_required") and not (
                            detail.get("label_available") and
                            detail.get("strict_executable") and
                            not detail.get("low_executable") and
                            not detail.get("medium_executable") and
                            detail.get("minimum_required_level") == "HIGH"):
                        errors.append(
                            f"{filename}:{line_number}: inconsistent strict_required label")
                    for level in ("low", "medium", "strict"):
                        if detail.get(f"{level}_executable") and not (
                                detail.get(f"{level}_endpoints_match") and
                                detail.get(f"{level}_validation", {}).get(
                                    "endpoint_check_performed") and
                                detail.get(f"{level}_validation", {}).get(
                                    "goal_reached")):
                            errors.append(
                                f"{filename}:{line_number}: executable {level} "
                                "trajectory without endpoint validation")
                    if detail.get("label_available"):
                        risk_labeled_counts[detail["run_id"]] += 1
                    if detail.get("l2_required"):
                        risk_l2_required_counts[detail["run_id"]] += 1
                    if detail.get("l3_required"):
                        risk_l3_required_counts[detail["run_id"]] += 1
                    if detail.get("strict_required"):
                        risk_strict_required_counts[detail["run_id"]] += 1
        detail_counts[key] = counts_for_file
    counts = Counter(record.get("run_id") for record in records)
    for run_id, count in counts.items():
        if count > 1:
            errors.append(f"duplicate run_id {run_id}: {count} records")
    totals = [record.get("t_total_ms") for record in records
              if isinstance(record.get("t_total_ms"), (int, float))]
    median_total = statistics.median(totals) if totals else None
    for record in records:
        run_id = record.get("run_id", "<missing>")
        missing = [field for field in REQUIRED if field not in record]
        if missing:
            errors.append(f"{run_id}: missing fields: {', '.join(missing)}")
        for field in TIME_FIELDS:
            value = record.get(field)
            if isinstance(value, (int, float)) and value < 0:
                errors.append(f"{run_id}: negative {field}={value}")
        if record.get("state_count") is not None:
            low = record.get("low_state_count", 0)
            medium = record.get("medium_state_count", 0)
            high = record.get("high_state_count", 0)
            if low + medium + high != record.get("state_count"):
                errors.append(f"{run_id}: state counts do not sum")
            if detail_counts["states"][run_id] != record.get("state_count"):
                errors.append(
                    f"{run_id}: state_metrics has {detail_counts['states'][run_id]} "
                    f"rows but state_count={record.get('state_count')}")
        if record.get("segment_count") is not None and \
                detail_counts["segments"][run_id] != record.get("segment_count"):
            errors.append(
                f"{run_id}: segment_metrics has {detail_counts['segments'][run_id]} "
                f"rows but segment_count={record.get('segment_count')}")
        if record.get("incumbent_update_count") is not None and \
                detail_counts["incumbents"][run_id] != record.get("incumbent_update_count"):
            errors.append(
                f"{run_id}: incumbent_events has "
                f"{detail_counts['incumbents'][run_id]} rows but "
                f"incumbent_update_count={record.get('incumbent_update_count')}")
        has_incumbent_event = detail_counts["incumbents"][run_id] > 0
        first_incumbent_time = record.get("t_first_feasible_ms")
        has_first_incumbent_time = isinstance(first_incumbent_time, (int, float)) \
            and math.isfinite(first_incumbent_time)
        if has_incumbent_event != has_first_incumbent_time:
            errors.append(
                f"{run_id}: incumbent event stream and t_first_feasible_ms disagree")
        if (has_incumbent_event or has_first_incumbent_time) and not \
                record.get("planner_returned", False):
            errors.append(
                f"{run_id}: first incumbent recorded but publication gate failed")
        if record.get("risk_ground_truth_trial_count") is not None and \
                detail_counts["risk_ground_truth"][run_id] != \
                record.get("risk_ground_truth_trial_count"):
            errors.append(
                f"{run_id}: risk_ground_truth has "
                f"{detail_counts['risk_ground_truth'][run_id]} rows but "
                f"risk_ground_truth_trial_count="
                f"{record.get('risk_ground_truth_trial_count')}")
        if record.get("risk_ground_truth_enabled"):
            labeled = int(record.get("risk_ground_truth_labeled_count", 0))
            trials = int(record.get("risk_ground_truth_trial_count", 0))
            if labeled > trials:
                errors.append(f"{run_id}: labeled risk trials exceed total trials")
            if risk_labeled_counts[run_id] != labeled:
                errors.append(
                    f"{run_id}: labeled risk row count does not match run summary")
            l2_required = int(
                record.get("risk_ground_truth_l2_required_count", 0))
            if risk_l2_required_counts[run_id] != l2_required:
                errors.append(
                    f"{run_id}: L2-required row count does not match run summary")
            l3_required = int(
                record.get("risk_ground_truth_l3_required_count", 0))
            if risk_l3_required_counts[run_id] != l3_required:
                errors.append(
                    f"{run_id}: L3-required row count does not match run summary")
            strict_required = int(
                record.get("risk_ground_truth_strict_required_count", 0))
            if risk_strict_required_counts[run_id] != strict_required:
                errors.append(
                    f"{run_id}: strict-required row count does not match run summary")
        if record.get("executable") and not record.get("external_footprint_valid"):
            errors.append(f"{run_id}: executable despite external validation failure")
        if record.get("executable") and not record.get("endpoint_check_performed"):
            errors.append(f"{run_id}: executable without endpoint validation")
        if record.get("executable") and not record.get("goal_reached"):
            errors.append(f"{run_id}: executable despite missing requested goal")
        if record.get("executable") and not record.get("endpoints_match"):
            errors.append(f"{run_id}: executable despite endpoint mismatch")
        if record.get("executable") and record.get("degraded"):
            errors.append(f"{run_id}: degraded counted executable")
        if record.get("planner_returned") and not record.get("executable"):
            errors.append(
                f"{run_id}: planner publication passed but final external contract failed")
        if record.get("strict_solver_call_count", 0) > 0:
            for field in ["full_svsdf_call_count", "preserve_shape_bypass_count"]:
                if field not in record:
                    errors.append(f"{run_id}: strict called but {field} missing")
        count_fields = [
            "full_svsdf_call_count",
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
            "external_equivalent_gate_reject_count",
        ]
        if any(record.get(field, 0) < 0 for field in count_fields):
            errors.append(f"{run_id}: negative phase/event counter")
        source_sum = sum(record.get(field, 0) for field in [
            "full_svsdf_initial_assigned_call_count",
            "full_svsdf_initial_escalation_call_count",
            "full_svsdf_frontend_fallback_call_count",
            "full_svsdf_validation_recovery_call_count",
            "full_svsdf_tail_refinement_call_count",
            "full_svsdf_other_call_count",
        ])
        if source_sum != record.get("full_svsdf_call_count", 0):
            errors.append(
                f"{run_id}: full-SVSDF phase counts {source_sum} do not sum to "
                f"total {record.get('full_svsdf_call_count', 0)}")
        if record.get("initial_level_upgrade_success_count", 0) > \
                record.get("initial_level_upgrade_attempt_count", 0):
            errors.append(f"{run_id}: initial upgrade successes exceed attempts")
        if record.get("frontend_fallback_success_count", 0) > \
                record.get("frontend_fallback_attempt_count", 0):
            errors.append(f"{run_id}: frontend fallback successes exceed attempts")
        if record.get("validation_recovery_success_count", 0) > \
                record.get("validation_recovery_attempt_count", 0):
            errors.append(f"{run_id}: validation recovery successes exceed attempts")
        if record.get("external_equivalent_gate_reject_count", 0) > \
                record.get("external_equivalent_gate_attempt_count", 0):
            errors.append(f"{run_id}: external-equivalent gate rejects exceed attempts")
        method = record.get("method_name")
        if method == "full_r2" and any(record.get(field, 0) != 0 for field in [
                "medium_solver_call_count", "strict_solver_call_count",
                "full_svsdf_call_count", "initial_level_upgrade_attempt_count",
                "frontend_fallback_attempt_count",
                "validation_recovery_attempt_count"]):
            errors.append(f"{run_id}: full_r2 escaped the forced LOW solver level")
        if method == "full_strict":
            if record.get("low_solver_call_count", 0) != 0 or \
                    record.get("medium_solver_call_count", 0) != 0:
                errors.append(f"{run_id}: full_strict used a non-SVSDF solver level")
            if record.get("strict_solver_call_count", 0) != \
                    record.get("full_svsdf_call_count", 0):
                errors.append(
                    f"{run_id}: full_strict strict calls did not all enter full SVSDF")
            if record.get("preserve_shape_bypass_count", 0) != 0:
                errors.append(f"{run_id}: full_strict used preserve-shape bypass")
            if record.get("planner_returned") and \
                    record.get("full_svsdf_call_count", 0) < \
                    record.get("high_segment_count", 0):
                errors.append(
                    f"{run_id}: full_strict did not solve every returned HIGH segment")
        if method == "hybrid_astar" and record.get("full_svsdf_call_count", 0) != 0:
            errors.append(f"{run_id}: hybrid_astar unexpectedly called full SVSDF")
        if method in {"current_no_recovery", "no_recovery"} and \
                record.get("validation_recovery_attempt_count", 0) != 0:
            errors.append(f"{run_id}: no-recovery method entered validation recovery")
        if not record.get("geometry_consistency_passed", False) and \
                record.get("planner_returned", False):
            errors.append(f"{run_id}: geometry mismatch entered formal planning")
        if record.get("planner_returned"):
            trajectory = batch / "trajectories" / f"{run_id}.csv"
            if not trajectory.exists():
                errors.append(f"{run_id}: missing trajectory file")
        clearance = record.get("minimum_external_clearance_m")
        if isinstance(clearance, (int, float)) and abs(clearance) > 1000:
            warnings.append(f"{run_id}: unusually large clearance {clearance}")
        if median_total and isinstance(record.get("t_total_ms"), (int, float)) and \
                record["t_total_ms"] > max(600000, 100 * median_total):
            warnings.append(f"{run_id}: unusually large total time {record['t_total_ms']} ms")
    manifest_hash = manifest.get("configuration_hash")
    embedded_hashes = {record.get("configuration_hash") for record in records
                       if record.get("configuration_hash") is not None}
    if len(embedded_hashes) > 1 or (embedded_hashes and manifest_hash not in embedded_hashes):
        errors.append("records with different configurations were grouped together")
    dataset_hash = manifest.get("dataset_manifest_hash")
    if dataset_hash is not None:
        embedded_dataset_hashes = {
            record.get("dataset_manifest_hash") for record in records}
        if embedded_dataset_hashes != {dataset_hash}:
            errors.append(
                "records do not all match the batch dataset_manifest_hash")
    task_sources = manifest.get("task_sources", [])
    expected_dataset_ids = {
        source.get("dataset_id") for source in task_sources
        if source.get("dataset_id") is not None}
    if int(manifest.get("dataset_count", 0)) != len(expected_dataset_ids):
        errors.append("manifest dataset_count does not match task_sources")
    if expected_dataset_ids:
        for record in records:
            dataset_id = record.get("dataset_id")
            if dataset_id is None:
                errors.append(
                    f"{record.get('run_id', '<missing>')}: missing dataset_id")
            elif dataset_id not in expected_dataset_ids:
                errors.append(
                    f"{record.get('run_id', '<missing>')}: unknown dataset_id "
                    f"{dataset_id}")
    if manifest.get("status") == "complete":
        expected_runs = manifest.get("run_count")
        if isinstance(expected_runs, int) and expected_runs != len(records):
            errors.append(
                f"complete manifest expects {expected_runs} runs, found "
                f"{len(records)}")
        completed_runs = manifest.get("completed_runs")
        if isinstance(completed_runs, int) and completed_runs != len(records):
            errors.append(
                f"manifest completed_runs={completed_runs}, found {len(records)}")
    lines = ["# Result Integrity Report", "", f"Batch: `{batch}`", "",
             f"Runs checked: {len(records)}", f"Errors: {len(errors)}",
             f"Warnings: {len(warnings)}", "", "## Errors", ""]
    lines.extend(f"- {item}" for item in errors)
    if not errors:
        lines.append("- None")
    lines.extend(["", "## Warnings", ""])
    lines.extend(f"- {item}" for item in warnings)
    if not warnings:
        lines.append("- None")
    output = batch / "result_integrity_report.md"
    atomic_write_text(output, "\n".join(lines) + "\n")
    print(output)
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
