#!/usr/bin/env python3
"""Shared deterministic I/O helpers for planner_experiments scripts."""

import hashlib
import json
import os
import pathlib
import tempfile
from typing import Any, Dict

import yaml


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parent


def resolve_path(value: str, relative_to: pathlib.Path) -> pathlib.Path:
    if value.startswith("package://"):
        rest = value[len("package://"):]
        package, _, suffix = rest.partition("/")
        if package == "planner_experiments":
            return (PACKAGE_ROOT / suffix).resolve()
        if package == "isweep_planner":
            return (REPOSITORY_ROOT / "isweep_planner" / suffix).resolve()
        raise ValueError(f"unknown package URI: {value}")
    path = pathlib.Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (relative_to / path).resolve()


def load_yaml(path: pathlib.Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    return data or {}


def resolve_catkin_executable(package: str, executable: str,
                              environment: Dict[str, str]) -> pathlib.Path:
    """Find a built node in the sourced overlay, not by source-tree depth.

    A repository may be symlinked into a workspace, so resolving __file__ and
    climbing to a guessed workspace can point at an entirely different tree.
    CMAKE_PREFIX_PATH also covers install spaces and isolated devel prefixes.
    """
    searched = []
    for prefix in environment.get("CMAKE_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        candidate = pathlib.Path(prefix).expanduser() / "lib" / package / executable
        searched.append(str(candidate))
        if candidate.is_file() and os.access(str(candidate), os.X_OK):
            return candidate.resolve()
    raise FileNotFoundError(
        f"Cannot find executable {package}/{executable} in the sourced catkin "
        "prefixes. Build the workspace and source devel/setup.bash (or "
        "install/setup.bash). Searched: " + ", ".join(searched))


def sha256_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stable_hash(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def atomic_write_text(path: pathlib.Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=path.name + ".",
                                             dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def atomic_write_json(path: pathlib.Path, value: Any) -> None:
    atomic_write_text(path, json.dumps(value, indent=2, sort_keys=True) + "\n")
