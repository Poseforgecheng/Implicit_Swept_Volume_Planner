#!/usr/bin/env python3
"""Run reproducible offline experiment combinations and aggregate atomically."""

import argparse
import concurrent.futures
import csv
import datetime as dt
import json
import os
import pathlib
import platform
import re
import shutil
import signal
import socket
import subprocess
import sys
import time
import urllib.parse
import xmlrpc.client
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

import yaml

from _common import (PACKAGE_ROOT, REPOSITORY_ROOT, atomic_write_json,
                     atomic_write_text, load_yaml, resolve_catkin_executable,
                     resolve_path, sha256_file,
                     stable_hash)
from generate_tasks import generate_task_document, write_task_document


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--tasks",
                        help="one legacy task file (one map and one robot)")
    source.add_argument(
        "--datasets",
        help="dataset manifest containing explicit task files and/or a map x robot matrix")
    parser.add_argument("--output", required=True)
    parser.add_argument("--methods", nargs="+")
    parser.add_argument("--maps", nargs="+")
    parser.add_argument("--robots", nargs="+")
    parser.add_argument("--task-ids", nargs="+")
    parser.add_argument("--seeds", nargs="+", type=int)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--parallel-workers", type=int, default=1)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--postprocess", action="store_true",
        help="after the batch, run integrity checks, analysis, and paper-table export")
    return parser.parse_args()


def first_existing(paths: Iterable[pathlib.Path]) -> pathlib.Path:
    for path in paths:
        if path.exists():
            return path.resolve()
    raise FileNotFoundError("none of these paths exists: " +
                            ", ".join(map(str, paths)))


def resolve_map(config: Dict[str, Any], config_path: pathlib.Path,
                tasks: Dict[str, Any], tasks_path: pathlib.Path) -> pathlib.Path:
    if tasks.get("map_yaml"):
        return resolve_path(str(tasks["map_yaml"]), tasks_path.parent)
    name = str(tasks["map"])
    root_value = config.get("paths", {}).get(
        "maps_root", "package://planner_experiments/maps")
    root = resolve_path(str(root_value), config_path.parent)
    return first_existing([root / f"{name}.yaml", root / name / "map.yaml",
                           pathlib.Path(name)])


def resolve_robot(config: Dict[str, Any], config_path: pathlib.Path,
                  tasks: Dict[str, Any], tasks_path: pathlib.Path) -> pathlib.Path:
    if tasks.get("robot_yaml"):
        return resolve_path(str(tasks["robot_yaml"]), tasks_path.parent)
    name = str(tasks["robot_shape"])
    root_value = config.get("paths", {}).get(
        "robot_shapes_dir", "package://planner_experiments/config/robot_shapes")
    root = resolve_path(str(root_value), config_path.parent)
    return first_existing([root / f"{name}.yaml", pathlib.Path(name)])


def map_hash(map_yaml: pathlib.Path) -> str:
    data = load_yaml(map_yaml)
    if data.get("map_hash"):
        return str(data["map_hash"])
    image = resolve_path(str(data["image"]), map_yaml.parent)
    return sha256_file(image)


def safe_identifier(value: str) -> str:
    normalized = re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_.-")
    if not normalized:
        raise ValueError(f"invalid empty identifier derived from {value!r}")
    return normalized


def scalar_metadata(value: Any, context: str) -> Dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"{context} metadata must be a mapping")
    output: Dict[str, Any] = {}
    reserved = {
        "run_id", "configuration_hash", "method_name", "map_name", "map_hash",
        "robot_shape", "task_id", "random_seed", "planner_status", "executable",
        "timeout", "degraded", "dataset_id", "dataset_manifest_hash",
        "tasks_file", "failure_stage", "failure_reason", "planner_returned",
        "external_footprint_valid", "internal_footprint_valid",
        "dynamics_valid", "boundary_continuous",
    }
    for key, item in value.items():
        name = str(key)
        if name in reserved or name.startswith("t_") or \
                name.endswith(("_count", "_valid")):
            raise ValueError(f"{context} metadata key is reserved: {name}")
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name):
            raise ValueError(f"{context} metadata key is not a field identifier: {name}")
        if item is not None and not isinstance(item, (bool, int, float, str)):
            raise ValueError(f"{context} metadata value must be scalar: {name}")
        output[name] = item
    return output


def inferred_map_metadata(map_data: Dict[str, Any]) -> Dict[str, Any]:
    output: Dict[str, Any] = {}
    mapping = {
        "generator_type": "map_generator_type",
        "generator_version": "map_generator_version",
        "random_seed": "map_generation_seed",
        "width": "map_width_cells",
        "height": "map_height_cells",
    }
    for source, target in mapping.items():
        value = map_data.get(source)
        if value is not None and isinstance(value, (bool, int, float, str)):
            output[target] = value
    geometry = map_data.get("all_geometry_parameters", {})
    if isinstance(geometry, dict):
        for key, value in geometry.items():
            name = "map_" + safe_identifier(str(key)).replace(".", "_")
            if value is None or isinstance(value, (bool, int, float, str)):
                output[name] = value
    return output


def prepare_dataset(config: Dict[str, Any], config_path: pathlib.Path,
                    dataset_id: str, tasks: Dict[str, Any],
                    tasks_path: Optional[pathlib.Path],
                    map_yaml: pathlib.Path, robot_yaml: pathlib.Path,
                    metadata: Dict[str, Any], generated: bool,
                    task_ids: Optional[Iterable[str]] = None) -> Dict[str, Any]:
    robot = load_yaml(robot_yaml)
    map_data = load_yaml(map_yaml)
    map_name = str(tasks.get("map", map_yaml.parent.name
                             if map_yaml.stem == "map" else map_yaml.stem))
    robot_name = str(tasks.get("robot_shape",
                               robot.get("name", robot_yaml.stem)))
    combined_metadata = inferred_map_metadata(map_data)
    combined_metadata.update(metadata)
    return {
        "id": safe_identifier(dataset_id),
        "tasks": tasks,
        "tasks_path": tasks_path,
        "map_yaml": map_yaml,
        "map_name": map_name,
        "map_hash": map_hash(map_yaml),
        "map_data": map_data,
        "robot_yaml": robot_yaml,
        "robot_name": robot_name,
        "robot": robot,
        "robot_hash": sha256_file(robot_yaml),
        "metadata": combined_metadata,
        "generated": generated,
        "task_ids": set(map(str, task_ids)) if task_ids else None,
    }


def explicit_dataset(config: Dict[str, Any], config_path: pathlib.Path,
                     tasks_path: pathlib.Path, dataset_id: str,
                     metadata: Dict[str, Any],
                     task_ids: Optional[Iterable[str]] = None) -> Dict[str, Any]:
    tasks_path = tasks_path.resolve()
    tasks = load_yaml(tasks_path)
    map_yaml = resolve_map(config, config_path, tasks, tasks_path)
    robot_yaml = resolve_robot(config, config_path, tasks, tasks_path)
    return prepare_dataset(
        config, config_path, dataset_id, tasks, tasks_path, map_yaml, robot_yaml,
        metadata, generated=False, task_ids=task_ids)


def path_item(item: Any, key: str, relative_to: pathlib.Path,
              context: str) -> Tuple[str, pathlib.Path, Dict[str, Any]]:
    if isinstance(item, str):
        path = resolve_path(item, relative_to)
        name = path.parent.name if path.stem == "map" else path.stem
        return name, path, {}
    if not isinstance(item, dict):
        raise ValueError(f"{context} item must be a path string or mapping")
    raw_path = item.get(key, item.get("path"))
    if not raw_path:
        raise ValueError(f"{context} item is missing {key}")
    path = resolve_path(str(raw_path), relative_to)
    default_name = path.parent.name if path.stem == "map" else path.stem
    name = str(item.get("name", default_name))
    metadata = scalar_metadata(item.get("metadata"), context)
    return name, path, metadata


def load_datasets(config: Dict[str, Any], config_path: pathlib.Path,
                  tasks_argument: Optional[str],
                  datasets_argument: Optional[str]) -> Tuple[List[Dict[str, Any]],
                                                            Optional[pathlib.Path],
                                                            Dict[str, Any]]:
    if tasks_argument:
        tasks_path = pathlib.Path(tasks_argument).resolve()
        dataset = explicit_dataset(
            config, config_path, tasks_path, tasks_path.stem, {})
        return [dataset], None, {}

    manifest_path = pathlib.Path(str(datasets_argument)).resolve()
    manifest = load_yaml(manifest_path)
    if int(manifest.get("schema_version", 1)) != 1:
        raise ValueError("unsupported dataset manifest schema_version")
    datasets: List[Dict[str, Any]] = []
    for index, entry in enumerate(manifest.get("datasets", [])):
        if isinstance(entry, str):
            task_value = entry
            entry = {}
        elif isinstance(entry, dict):
            if not entry.get("enabled", True):
                continue
            task_value = entry.get("tasks", entry.get("tasks_file"))
        else:
            raise ValueError(f"datasets[{index}] must be a path or mapping")
        if not task_value:
            raise ValueError(f"datasets[{index}] is missing tasks")
        tasks_path = resolve_path(str(task_value), manifest_path.parent)
        dataset_id = str(entry.get("id", tasks_path.stem))
        metadata = scalar_metadata(entry.get("metadata"),
                                   f"datasets[{index}]")
        datasets.append(explicit_dataset(
            config, config_path, tasks_path, dataset_id, metadata,
            task_ids=entry.get("task_ids")))

    matrix = manifest.get("matrix")
    if matrix:
        if not isinstance(matrix, dict):
            raise ValueError("matrix must be a mapping")
        map_items = [
            path_item(item, "yaml", manifest_path.parent, "matrix.maps")
            for item in matrix.get("maps", [])
        ]
        robot_items = [
            path_item(item, "yaml", manifest_path.parent, "matrix.robots")
            for item in matrix.get("robots", [])
        ]
        if not map_items or not robot_items:
            raise ValueError("matrix requires non-empty maps and robots")
        generation = dict(matrix.get("task_generation", {}))
        num_tasks = int(generation.get("num_tasks", 30))
        seed = int(generation.get("seed", 1))
        min_distance = float(generation.get("min_distance", 2.0))
        yaw_mode = str(generation.get("yaw_mode", "random"))
        different_direction = bool(
            generation.get("different_topology_direction", False))
        exclusions: Set[Tuple[str, str]] = set()
        for exclusion in matrix.get("exclude", []):
            if not isinstance(exclusion, dict):
                raise ValueError("matrix.exclude entries must be mappings")
            exclusions.add((str(exclusion.get("map", "")),
                            str(exclusion.get("robot", ""))))
        matrix_metadata = scalar_metadata(matrix.get("metadata"), "matrix")
        for map_name, map_yaml, map_metadata in map_items:
            for configured_robot_name, robot_yaml, robot_metadata in robot_items:
                if (map_name, configured_robot_name) in exclusions:
                    continue
                document = generate_task_document(
                    map_yaml, robot_yaml, num_tasks=num_tasks, seed=seed,
                    min_distance=min_distance, yaw_mode=yaw_mode,
                    different_topology_direction=different_direction,
                    map_name=map_name)
                robot_name = str(document["robot_shape"])
                dataset_id = f"{map_name}__{robot_name}"
                metadata = dict(matrix_metadata)
                metadata.update(map_metadata)
                metadata.update(robot_metadata)
                datasets.append(prepare_dataset(
                    config, config_path, dataset_id, document, None,
                    map_yaml, robot_yaml, metadata, generated=True))

    if not datasets:
        raise ValueError("dataset manifest selected no datasets")
    identifiers: Set[str] = set()
    for dataset in datasets:
        if dataset["id"] in identifiers:
            raise ValueError(f"duplicate dataset id: {dataset['id']}")
        identifiers.add(dataset["id"])
    return datasets, manifest_path, manifest


def proc_value(path: pathlib.Path, prefix: str) -> str:
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.startswith(prefix):
                return line.split(":", 1)[-1].strip()
    except OSError:
        pass
    return ""


def cmake_build_type(workspace: pathlib.Path) -> str:
    cache = workspace / "build" / "CMakeCache.txt"
    try:
        for line in cache.read_text(encoding="utf-8").splitlines():
            if line.startswith("CMAKE_BUILD_TYPE:STRING="):
                return line.split("=", 1)[1]
    except OSError:
        pass
    return ""


def master_online(uri: str) -> bool:
    try:
        proxy = xmlrpc.client.ServerProxy(uri)
        code, _, _ = proxy.getUri("/planner_experiments_probe")
        return code == 1
    except Exception:
        return False


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def start_master(environment: Dict[str, str]) -> Tuple[Optional[subprocess.Popen], str]:
    existing = environment.get("ROS_MASTER_URI", "http://127.0.0.1:11311")
    if master_online(existing):
        return None, existing
    port = free_port()
    uri = f"http://127.0.0.1:{port}"
    environment["ROS_MASTER_URI"] = uri
    process = subprocess.Popen(["roscore", "-p", str(port)], env=environment,
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL,
                               start_new_session=True)
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("roscore exited during startup")
        if master_online(uri):
            return process, uri
        time.sleep(0.1)
    process.terminate()
    raise RuntimeError("timed out starting roscore")


def failure_record(combo: Dict[str, Any], status: str, stage: str,
                   reason: str, elapsed_ms: float) -> Dict[str, Any]:
    task = combo["task"]
    record = {
        "schema_version": 4, "run_id": combo["run_id"],
        "configuration_hash": combo["configuration_hash"],
        "dataset_manifest_hash": combo.get("dataset_manifest_hash"),
        "dataset_id": combo["dataset"]["id"],
        "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(),
        "method_name": combo["method"], "map_name": combo["map_name"],
        "map_hash": combo["map_hash"], "robot_shape": combo["robot_name"],
        "task_id": task["id"], "start_x": task["start"][0],
        "start_y": task["start"][1], "start_yaw": task["start"][2],
        "goal_x": task["goal"][0], "goal_y": task["goal"][1],
        "goal_yaw": task["goal"][2], "random_seed": combo["seed"],
        "planner_status": status, "failure_stage": stage,
        "failure_reason": reason, "planner_returned": False,
        "internal_footprint_valid": False, "external_footprint_valid": False,
        "dynamics_valid": False, "boundary_continuous": False,
        "endpoint_check_performed": False, "start_matches": False,
        "goal_reached": False, "endpoints_match": False,
        "geometry_consistency_passed": False,
        "executable": False, "degraded": False,
        "timeout": status == "TIMEOUT", "t_total_ms": elapsed_ms,
        "full_svsdf_call_count": 0,
        "full_svsdf_initial_assigned_call_count": 0,
        "full_svsdf_initial_escalation_call_count": 0,
        "full_svsdf_frontend_fallback_call_count": 0,
        "full_svsdf_validation_recovery_call_count": 0,
        "full_svsdf_tail_refinement_call_count": 0,
        "full_svsdf_other_call_count": 0,
        "initial_level_upgrade_attempt_count": 0,
        "initial_level_upgrade_success_count": 0,
        "frontend_fallback_attempt_count": 0,
        "frontend_fallback_success_count": 0,
        "validation_recovery_attempt_count": 0,
        "validation_recovery_success_count": 0,
        "post_incumbent_svsdf_call_count": 0,
        "post_incumbent_svsdf_budget_reject_count": 0,
        "tail_refinement_svsdf_budget_reject_count": 0,
        "external_equivalent_gate_attempt_count": 0,
        "external_equivalent_gate_reject_count": 0,
    }
    record.update(combo["dataset"]["metadata"])
    return record


def write_failure_run(run_directory: pathlib.Path, record: Dict[str, Any]) -> None:
    run_directory.mkdir(parents=True, exist_ok=True)
    atomic_write_json(run_directory / "result.json", record)
    atomic_write_text(run_directory / "state_metrics.jsonl", "")
    atomic_write_text(run_directory / "segment_metrics.jsonl", "")
    atomic_write_text(run_directory / "incumbent_events.jsonl", "")
    atomic_write_text(run_directory / "risk_ground_truth.jsonl", "")
    atomic_write_text(run_directory / "trajectory.csv",
                      "t,x,y,yaw,vx,vy,ax,ay,yaw_rate,yaw_acceleration\n")
    atomic_write_text(run_directory / "complete", "complete\n")


def enrich_run_result(run_directory: pathlib.Path,
                      combo: Dict[str, Any]) -> None:
    result_path = run_directory / "result.json"
    record = json.loads(result_path.read_text(encoding="utf-8"))
    record["dataset_id"] = combo["dataset"]["id"]
    record["dataset_manifest_hash"] = combo.get("dataset_manifest_hash")
    record["tasks_file"] = str(combo["dataset"]["tasks_path"])
    record.update(combo["dataset"]["metadata"])
    atomic_write_json(result_path, record)


def run_one(combo: Dict[str, Any], batch: pathlib.Path,
            config_path: pathlib.Path,
            planner_config: Optional[pathlib.Path], base_env: Dict[str, str],
            budget: float,
            planner_overrides: Dict[str, Any]) -> Tuple[str, bool, str]:
    run_directory = batch / "runs" / combo["run_id"]
    if run_directory.joinpath("complete").exists():
        enrich_run_result(run_directory, combo)
        return combo["run_id"], True, "resumed"
    run_directory.mkdir(parents=True, exist_ok=True)
    node_name = "planner_experiment_" + combo["run_id"][:16]
    environment = dict(base_env)
    environment["OMP_NUM_THREADS"] = str(combo["threads"])
    environment["OPENBLAS_NUM_THREADS"] = "1"
    environment["MKL_NUM_THREADS"] = "1"
    if planner_config:
        loaded = subprocess.run(
            ["rosparam", "load", str(planner_config), f"/{node_name}"],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, timeout=20)
        if loaded.returncode:
            write_failure_run(run_directory, failure_record(
                combo, "CONFIGURATION_ERROR", "planner_parameters",
                loaded.stderr.strip(), 0.0))
            return combo["run_id"], False, loaded.stderr.strip()
    for key, value in planner_overrides.items():
        parameter = f"/{node_name}/{key.strip('/')}"
        assigned = subprocess.run(
            ["rosparam", "set", parameter,
             yaml.safe_dump(value, default_flow_style=True).strip()],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, timeout=20)
        if assigned.returncode:
            write_failure_run(run_directory, failure_record(
                combo, "CONFIGURATION_ERROR", "planner_parameter_override",
                assigned.stderr.strip(), 0.0))
            return combo["run_id"], False, assigned.stderr.strip()

    executable = resolve_catkin_executable(
        "planner_experiments", "experiment_runner_node", environment)
    command = [
        str(executable), "--config", str(config_path),
        "--tasks", str(combo["dataset"]["tasks_path"]),
        "--method", combo["method"], "--task-id", combo["task"]["id"],
        "--map-yaml", str(combo["map_yaml"]), "--robot-yaml", str(combo["robot_yaml"]),
        "--output-dir", str(run_directory), "--run-id", combo["run_id"],
        "--configuration-hash", combo["configuration_hash"],
        "--seed", str(combo["seed"]), f"__name:={node_name}",
    ]
    log_path = batch / "logs" / f"{combo['run_id']}.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    try:
        with log_path.open("w", encoding="utf-8") as log:
            completed = subprocess.run(command, env=environment, stdout=log,
                                       stderr=subprocess.STDOUT,
                                       timeout=max(5.0, budget + 15.0))
        elapsed = (time.monotonic() - started) * 1000.0
        if completed.returncode != 0 or not run_directory.joinpath("complete").exists():
            write_failure_run(run_directory, failure_record(
                combo, "INTERNAL_ERROR", "runner_process",
                f"runner returned {completed.returncode}", elapsed))
            return combo["run_id"], False, f"exit {completed.returncode}"
        enrich_run_result(run_directory, combo)
        return combo["run_id"], True, "completed"
    except subprocess.TimeoutExpired:
        elapsed = (time.monotonic() - started) * 1000.0
        write_failure_run(run_directory, failure_record(
            combo, "TIMEOUT", "process_timeout",
            "runner exceeded batch hard timeout", elapsed))
        return combo["run_id"], False, "timeout"


def run_one_safe(combo: Dict[str, Any], batch: pathlib.Path,
                 config_path: pathlib.Path,
                 planner_config: Optional[pathlib.Path],
                 base_env: Dict[str, str], budget: float,
                 planner_overrides: Dict[str, Any]) -> Tuple[str, bool, str]:
    started = time.monotonic()
    try:
        return run_one(combo, batch, config_path, planner_config, base_env,
                       budget, planner_overrides)
    except Exception as exception:
        elapsed = (time.monotonic() - started) * 1000.0
        reason = f"{type(exception).__name__}: {exception}"
        write_failure_run(
            batch / "runs" / combo["run_id"],
            failure_record(combo, "INTERNAL_ERROR",
                           "batch_run_exception", reason, elapsed))
        return combo["run_id"], False, reason


def aggregate(batch: pathlib.Path) -> List[Dict[str, Any]]:
    records: List[Dict[str, Any]] = []
    state_lines: List[str] = []
    segment_lines: List[str] = []
    incumbent_lines: List[str] = []
    risk_ground_truth_lines: List[str] = []
    trajectories = batch / "trajectories"
    failures = batch / "failures"
    trajectories.mkdir(exist_ok=True)
    failures.mkdir(exist_ok=True)
    for run_dir in sorted((batch / "runs").glob("*")):
        if not run_dir.joinpath("complete").exists():
            continue
        try:
            record = json.loads(run_dir.joinpath("result.json").read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        records.append(record)
        for name, target in [("state_metrics.jsonl", state_lines),
                             ("segment_metrics.jsonl", segment_lines),
                             ("incumbent_events.jsonl", incumbent_lines),
                             ("risk_ground_truth.jsonl", risk_ground_truth_lines)]:
            path = run_dir / name
            if path.exists():
                target.extend(line for line in path.read_text(encoding="utf-8").splitlines()
                              if line.strip())
        trajectory = run_dir / "trajectory.csv"
        if trajectory.exists():
            shutil.copy2(trajectory, trajectories / f"{record['run_id']}.csv")
        if not record.get("executable", False):
            atomic_write_json(failures / f"{record['run_id']}.json", record)
    atomic_write_text(batch / "runs.jsonl",
                      "".join(json.dumps(record, sort_keys=True) + "\n"
                              for record in records))
    atomic_write_text(batch / "state_metrics.jsonl",
                      "".join(line + "\n" for line in state_lines))
    atomic_write_text(batch / "segment_metrics.jsonl",
                      "".join(line + "\n" for line in segment_lines))
    atomic_write_text(batch / "incumbent_events.jsonl",
                      "".join(line + "\n" for line in incumbent_lines))
    atomic_write_text(batch / "risk_ground_truth.jsonl",
                      "".join(line + "\n" for line in risk_ground_truth_lines))
    flattened = []
    for record in records:
        row = {key: value for key, value in record.items()
               if not isinstance(value, (dict, list))}
        for nested in (record.get("validation", {}), record.get("geometry", {})):
            row.update(nested)
        flattened.append(row)
    fields = sorted({key for row in flattened for key in row})
    output = []
    if fields:
        import io
        stream = io.StringIO(newline="")
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(flattened)
        output.append(stream.getvalue())
    atomic_write_text(batch / "runs.csv", output[0] if output else "")
    return records


def materialize_generated_tasks(batch: pathlib.Path,
                                datasets: List[Dict[str, Any]]) -> None:
    generated_root = batch / "generated_tasks"
    for dataset in datasets:
        if not dataset["generated"]:
            continue
        destination = generated_root / f"{dataset['id']}.yaml"
        write_task_document(dataset["tasks"], destination,
                            dataset["map_yaml"], dataset["robot_yaml"])
        dataset["tasks_path"] = destination.resolve()


def unique_manifest_items(datasets: List[Dict[str, Any]],
                          item_type: str) -> List[Dict[str, Any]]:
    items: List[Dict[str, Any]] = []
    seen: Set[str] = set()
    for dataset in datasets:
        if item_type == "map":
            identity = dataset["map_hash"]
            item = {
                "name": dataset["map_name"],
                "hash": dataset["map_hash"],
                "yaml": str(dataset["map_yaml"]),
                "metadata": dataset["map_data"],
            }
        else:
            identity = dataset["robot_hash"]
            item = {
                "name": dataset["robot_name"],
                "hash": dataset["robot_hash"],
                "yaml": str(dataset["robot_yaml"]),
                "configuration": dataset["robot"],
            }
        if identity not in seen:
            seen.add(identity)
            items.append(item)
    return items


def run_postprocess(batch: pathlib.Path,
                    environment: Dict[str, str]) -> Tuple[bool, List[Dict[str, Any]]]:
    scripts = PACKAGE_ROOT / "scripts"
    steps = [
        ("integrity_check", scripts / "check_results.py"),
        ("analysis", scripts / "analyze_results.py"),
        ("paper_tables", scripts / "export_paper_tables.py"),
    ]
    statuses: List[Dict[str, Any]] = []
    for name, script in steps:
        started = time.monotonic()
        completed = subprocess.run(
            [sys.executable, str(script), str(batch)], env=environment,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        output = completed.stdout.strip()
        if output:
            print(output)
        statuses.append({
            "name": name,
            "return_code": completed.returncode,
            "elapsed_ms": (time.monotonic() - started) * 1000.0,
            "output": output,
        })
        if completed.returncode != 0:
            return False, statuses
    return True, statuses


def main() -> int:
    args = parse_args()
    if args.parallel_workers < 1:
        raise SystemExit("--parallel-workers must be positive")
    config_path = pathlib.Path(args.config).resolve()
    config = load_yaml(config_path)
    try:
        datasets, dataset_manifest_path, dataset_manifest = load_datasets(
            config, config_path, args.tasks, args.datasets)
    except (OSError, ValueError, RuntimeError, KeyError) as exception:
        raise SystemExit(f"dataset configuration error: {exception}")
    datasets = [
        dataset for dataset in datasets
        if (not args.maps or dataset["map_name"] in args.maps) and
           (not args.robots or dataset["robot_name"] in args.robots)
    ]
    if not datasets:
        raise SystemExit("map/robot filters selected no datasets")
    methods = args.methods or config.get("experiment", {}).get("methods") or [
        "current_full", "current_no_recovery", "full_r2", "full_strict", "hybrid_astar"]
    seeds = args.seeds or [int(config.get("reproducibility", {}).get("random_seed", 1))]
    configuration_hash = stable_hash(config)
    resolved_dataset_definition = [{
        "id": dataset["id"],
        "tasks": dataset["tasks"],
        "map_hash": dataset["map_hash"],
        "robot_hash": dataset["robot_hash"],
        "metadata": dataset["metadata"],
        "task_ids": (
            sorted(dataset["task_ids"]) if dataset["task_ids"] else None),
    } for dataset in datasets]
    dataset_manifest_hash = stable_hash({
        "manifest": dataset_manifest if dataset_manifest_path else None,
        "resolved_datasets": resolved_dataset_definition,
    })
    combos = []
    threads = int(config.get("reproducibility", {}).get("fixed_cpu_threads", 1))
    run_ids: Set[str] = set()
    selected_task_count = 0
    for dataset in datasets:
        tasks = dataset["tasks"].get("tasks", [])
        selected_tasks = [
            task for task in tasks
            if (not args.task_ids or str(task["id"]) in args.task_ids) and
               (not dataset["task_ids"] or
                str(task["id"]) in dataset["task_ids"])
        ]
        if not selected_tasks:
            continue
        dataset["selected_task_count"] = len(selected_tasks)
        selected_task_count += len(selected_tasks)
        for method in methods:
            for task in selected_tasks:
                for seed in seeds:
                    canonical = {
                        "method": method, "map_hash": dataset["map_hash"],
                        "robot_hash": dataset["robot_hash"],
                        "task": task, "seed": seed,
                        "configuration_hash": configuration_hash,
                    }
                    run_id = stable_hash(canonical)
                    if run_id in run_ids:
                        raise SystemExit(
                            "duplicate experimental condition across datasets: "
                            f"{dataset['id']} {method} {task['id']} seed={seed}")
                    run_ids.add(run_id)
                    combos.append({
                        "run_id": run_id, "method": method, "task": task,
                        "seed": seed, "map_yaml": dataset["map_yaml"],
                        "map_hash": dataset["map_hash"],
                        "map_name": dataset["map_name"],
                        "robot_yaml": dataset["robot_yaml"],
                        "robot_name": dataset["robot_name"],
                        "configuration_hash": configuration_hash,
                        "dataset_manifest_hash": dataset_manifest_hash,
                        "threads": threads, "dataset": dataset,
                    })
    datasets = [dataset for dataset in datasets
                if dataset.get("selected_task_count", 0) > 0]
    if not combos:
        raise SystemExit("task filters selected no experiment combinations")
    if args.dry_run:
        for combo in combos:
            print(combo["run_id"], combo["dataset"]["id"], combo["method"],
                  combo["map_name"], combo["robot_name"],
                  combo["task"]["id"], combo["seed"])
        print(f"datasets: {len(datasets)}")
        print(f"selected tasks: {selected_task_count}")
        print(f"total combinations: {len(combos)}")
        return 0

    output_root = pathlib.Path(args.output).resolve()
    previous_manifest: Optional[Dict[str, Any]] = None
    if args.resume and output_root.joinpath("manifest.json").exists():
        batch = output_root
        previous_manifest = json.loads(
            output_root.joinpath("manifest.json").read_text(encoding="utf-8"))
    else:
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        batch = output_root / stamp
    if batch.exists() and args.overwrite and not args.resume:
        shutil.rmtree(batch)
    if batch.exists() and not args.resume and any(batch.iterdir()):
        raise SystemExit(f"output exists; use --resume or --overwrite: {batch}")
    if previous_manifest is not None:
        if previous_manifest.get("configuration_hash") != configuration_hash:
            raise SystemExit(
                "--resume configuration differs from the existing batch")
        previous_dataset_hash = previous_manifest.get("dataset_manifest_hash")
        if previous_dataset_hash is not None and \
                previous_dataset_hash != dataset_manifest_hash:
            raise SystemExit(
                "--resume dataset manifest differs from the existing batch")
        previous_run_count = previous_manifest.get("run_count")
        if isinstance(previous_run_count, int) and \
                previous_run_count != len(combos):
            raise SystemExit(
                "--resume filters/methods/seeds differ from the existing batch")
    for directory in ["runs", "trajectories", "failures", "logs", "summary"]:
        (batch / directory).mkdir(parents=True, exist_ok=True)
    materialize_generated_tasks(batch, datasets)
    task_sources = [{
        "dataset_id": dataset["id"],
        "tasks_file": str(dataset["tasks_path"]),
        "generated": bool(dataset["generated"]),
        "map_name": dataset["map_name"],
        "map_hash": dataset["map_hash"],
        "robot_shape": dataset["robot_name"],
        "robot_hash": dataset["robot_hash"],
        "selected_task_count": dataset["selected_task_count"],
        "metadata": dataset["metadata"],
    } for dataset in datasets]
    manifest = {
        "schema_version": 1, "status": "running",
        "start_time": (
            previous_manifest.get("start_time") if previous_manifest
            else dt.datetime.now(dt.timezone.utc).isoformat()),
        "end_time": None, "configuration": config,
        "configuration_hash": configuration_hash,
        "dataset_manifest_file": (
            str(dataset_manifest_path) if dataset_manifest_path else None),
        "dataset_manifest_hash": dataset_manifest_hash,
        "dataset_manifest": dataset_manifest if dataset_manifest_path else None,
        "dataset_count": len(datasets),
        "task_sources": task_sources,
        "map_list": unique_manifest_items(datasets, "map"),
        "robot_geometries": unique_manifest_items(datasets, "robot"),
        "methods": list(methods), "task_count": selected_task_count,
        "random_seeds": list(seeds), "run_count": len(combos),
        "parallel_workers": args.parallel_workers,
        "formal_timing": args.parallel_workers == 1 and threads == 1,
        "thread_count": threads,
        "host_information": {
            "platform": platform.platform(), "machine": platform.machine(),
            "cpu_model": proc_value(pathlib.Path("/proc/cpuinfo"), "model name"),
            "memory_total": proc_value(pathlib.Path("/proc/meminfo"), "MemTotal"),
            "logical_cpu_count": os.cpu_count(),
        },
        "build_information": {
            "build_type": cmake_build_type(PACKAGE_ROOT.parents[2]),
            "python_version": platform.python_version(),
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
        },
        "command": sys.argv,
        "git_commit": subprocess.run(
            ["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"],
            stdout=subprocess.PIPE, text=True, check=False).stdout.strip(),
        "git_dirty": bool(subprocess.run(
            ["git", "-C", str(REPOSITORY_ROOT), "status", "--porcelain"],
            stdout=subprocess.PIPE, text=True, check=False).stdout.strip()),
    }
    atomic_write_json(batch / "manifest.json", manifest)
    environment = dict(os.environ)
    # Keep package:// resolution deterministic even when the caller invokes the
    # script directly after building but has only sourced /opt/ros/noetic.
    package_source_root = str(PACKAGE_ROOT.parent)
    inherited_package_path = environment.get("ROS_PACKAGE_PATH", "")
    environment["ROS_PACKAGE_PATH"] = package_source_root + (
        os.pathsep + inherited_package_path if inherited_package_path else "")
    environment["ROS_HOME"] = str(batch / "logs" / "ros_home")
    environment["ROS_LOG_DIR"] = str(batch / "logs" / "ros")
    pathlib.Path(environment["ROS_HOME"]).mkdir(parents=True, exist_ok=True)
    pathlib.Path(environment["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    master = None
    try:
        master, _ = start_master(environment)
        planner_value = config.get("paths", {}).get(
            "planner_config", "package://isweep_planner/config/planner.yaml")
        planner_config = resolve_path(str(planner_value), config_path.parent)
        budget = float(config.get("experiment", {}).get("total_time_budget_sec", 20.0))
        planner_overrides = dict(config.get("planner_overrides", {}))
        pending = [combo for combo in combos
                   if not (args.resume and
                           batch.joinpath("runs", combo["run_id"], "complete").exists())]
        if args.parallel_workers == 1:
            for index, combo in enumerate(pending, 1):
                run_id, ok, message = run_one_safe(
                    combo, batch, config_path, planner_config, environment,
                    budget, planner_overrides)
                print(f"[{index}/{len(pending)}] {run_id[:12]} {message}")
                aggregate(batch)
        else:
            with concurrent.futures.ThreadPoolExecutor(
                    max_workers=args.parallel_workers) as executor:
                futures = [executor.submit(
                    run_one_safe, combo, batch, config_path, planner_config,
                    environment, budget, planner_overrides)
                           for combo in pending]
                for index, future in enumerate(concurrent.futures.as_completed(futures), 1):
                    run_id, ok, message = future.result()
                    print(f"[{index}/{len(futures)}] {run_id[:12]} {message}")
            aggregate(batch)
        records = aggregate(batch)
        manifest["status"] = "complete"
        manifest["end_time"] = dt.datetime.now(dt.timezone.utc).isoformat()
        manifest["completed_runs"] = len(records)
        manifest["executable_runs"] = sum(bool(record.get("executable")) for record in records)
        atomic_write_json(batch / "manifest.json", manifest)
        if args.postprocess:
            postprocess_ok, statuses = run_postprocess(batch, environment)
            manifest["postprocess"] = statuses
            manifest["postprocess_status"] = (
                "complete" if postprocess_ok else "failed")
            atomic_write_json(batch / "manifest.json", manifest)
            if not postprocess_ok:
                print(f"postprocessing failed; results remain in {batch}",
                      file=sys.stderr)
                return 2
        print(batch)
        return 0
    except KeyboardInterrupt:
        manifest["status"] = "interrupted"
        manifest["end_time"] = dt.datetime.now(dt.timezone.utc).isoformat()
        aggregate(batch)
        atomic_write_json(batch / "manifest.json", manifest)
        print(f"interrupted; completed results remain in {batch}", file=sys.stderr)
        return 130
    finally:
        if master is not None and master.poll() is None:
            os.killpg(master.pid, signal.SIGTERM)
            try:
                master.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(master.pid, signal.SIGKILL)


if __name__ == "__main__":
    raise SystemExit(main())
