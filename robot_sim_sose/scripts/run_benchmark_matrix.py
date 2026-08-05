#!/usr/bin/env python3
"""Run a resumable SLAM benchmark matrix for one recorded robot bag."""

from __future__ import annotations

import argparse
import csv
import fcntl
import hashlib
import json
import os
import random
import re
import shutil
import signal
import socket
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError as exc:  # pragma: no cover - depends on the robot image.
    raise SystemExit(
        "PyYAML is required. Install python3-yaml in the robot container."
    ) from exc


ALGORITHMS = {"slam_toolbox", "cartographer", "rtabmap"}
SCAN_VARIANTS = {"raw", "filtered"}
SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")
RESULT_PATTERN = re.compile(r"(?:Results:|Benchmark saved to)\s+(.+?)\s*$")


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def positive_int(value: Any, name: str) -> int:
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be an integer") from exc
    if result <= 0:
        raise ValueError(f"{name} must be greater than zero")
    return result


def nonnegative_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number") from exc
    if result < 0.0:
        raise ValueError(f"{name} cannot be negative")
    return result


def as_bool(value: Any, name: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "yes", "on", "1"}:
            return True
        if normalized in {"false", "no", "off", "0"}:
            return False
    raise ValueError(f"{name} must be true or false")


def safe_name(value: Any, name: str) -> str:
    result = str(value).strip()
    if not SAFE_NAME.fullmatch(result):
        raise ValueError(
            f"{name} may contain only letters, numbers, dot, underscore, and dash"
        )
    return result


def resolve_path(value: Any, workspace: Path) -> Path:
    text = os.path.expandvars(os.path.expanduser(str(value)))
    path = Path(text)
    if not path.is_absolute():
        path = workspace / path
    return path.resolve()


def infer_bag_name(bag_path: Path) -> str:
    name = bag_path.name
    if name == "bag" or name.startswith("bag_"):
        name = bag_path.parent.name
    sanitized = re.sub(r"[^A-Za-z0-9._-]+", "_", name).strip("_")
    return safe_name(sanitized or "unnamed_bag", "bag.name")


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def normalize_algorithms(raw: Any) -> dict[str, dict[str, Any]]:
    if raw is None:
        raw = ["slam_toolbox", "cartographer", "rtabmap"]

    result: dict[str, dict[str, Any]] = {}
    if isinstance(raw, list):
        for entry in raw:
            if isinstance(entry, str):
                name = entry
                settings: dict[str, Any] = {}
            elif isinstance(entry, dict) and "name" in entry:
                name = str(entry["name"])
                settings = {key: value for key, value in entry.items() if key != "name"}
            else:
                raise ValueError("Each algorithms entry must be a name or a mapping with name")
            result[name] = settings
    elif isinstance(raw, dict):
        for name, settings in raw.items():
            if settings is None:
                settings = {}
            if not isinstance(settings, dict):
                raise ValueError(f"Settings for algorithm {name} must be a mapping")
            result[str(name)] = dict(settings)
    else:
        raise ValueError("algorithms must be a list or mapping")

    if not result:
        raise ValueError("At least one algorithm is required")
    unknown = set(result) - ALGORITHMS
    if unknown:
        raise ValueError(f"Unsupported algorithms: {', '.join(sorted(unknown))}")

    for name, settings in result.items():
        default_camera = name == "rtabmap"
        settings["use_camera"] = as_bool(
            settings.get("use_camera", default_camera),
            f"algorithms.{name}.use_camera",
        )
        if name != "rtabmap" and settings["use_camera"]:
            raise ValueError(f"Camera input is supported only for RTAB-Map, not {name}")
    return result


def normalize_config(path: Path) -> dict[str, Any]:
    raw = load_yaml(path)
    workspace = resolve_path(
        raw.get("workspace", os.environ.get("WORKSPACE", "/workspaces/robot_sim_sose")),
        Path.cwd(),
    )
    experiment_raw = raw.get("experiment", {})
    if not isinstance(experiment_raw, dict):
        raise ValueError("experiment must be a mapping")
    bag_raw = raw.get("bag")
    if not isinstance(bag_raw, dict):
        raise ValueError("bag must be a mapping")
    if not bag_raw.get("path"):
        raise ValueError("bag.path is required")
    if not bag_raw.get("waypoints"):
        raise ValueError("bag.waypoints is required for accuracy evaluation")

    bag_path = resolve_path(bag_raw["path"], workspace)
    waypoint_path = resolve_path(bag_raw["waypoints"], workspace)
    bag_name = safe_name(
        bag_raw.get("name", infer_bag_name(bag_path)),
        "bag.name",
    )
    experiment_name = safe_name(
        experiment_raw.get("name", "slam_comparison"),
        "experiment.name",
    )

    variants = raw.get("scan_variants", ["raw", "filtered"])
    if not isinstance(variants, list) or not variants:
        raise ValueError("scan_variants must be a non-empty list")
    variants = [str(value) for value in variants]
    unknown_variants = set(variants) - SCAN_VARIANTS
    if unknown_variants:
        raise ValueError(
            f"Unsupported scan variants: {', '.join(sorted(unknown_variants))}"
        )

    max_temperature = experiment_raw.get("max_start_temperature_c", 65.0)
    if max_temperature is not None:
        max_temperature = float(max_temperature)
    ros_domain_id = experiment_raw.get("ros_domain_id")
    if ros_domain_id is not None:
        ros_domain_id = int(ros_domain_id)
        if not 0 <= ros_domain_id <= 232:
            raise ValueError("experiment.ros_domain_id must be between 0 and 232")

    config = {
        "workspace": str(workspace),
        "results_root": str(
            resolve_path(raw.get("results_root", "evaluation/results"), workspace)
        ),
        "experiment": {
            "name": experiment_name,
            "repetitions": positive_int(
                experiment_raw.get("repetitions", 5),
                "experiment.repetitions",
            ),
            "playback_rate": float(experiment_raw.get("playback_rate", 1.0)),
            "resource_interval": float(
                experiment_raw.get("resource_interval", 1.0)
            ),
            "post_playback_delay_sec": nonnegative_float(
                experiment_raw.get("post_playback_delay_sec", 5.0),
                "experiment.post_playback_delay_sec",
            ),
            "cooldown_seconds": nonnegative_float(
                experiment_raw.get("cooldown_seconds", 30.0),
                "experiment.cooldown_seconds",
            ),
            "max_start_temperature_c": max_temperature,
            "max_cooldown_wait_seconds": positive_int(
                experiment_raw.get("max_cooldown_wait_seconds", 900),
                "experiment.max_cooldown_wait_seconds",
            ),
            "random_seed": int(experiment_raw.get("random_seed", 20260805)),
            "normalize_scan": as_bool(
                experiment_raw.get("normalize_scan", False),
                "experiment.normalize_scan",
            ),
            "start_foxglove": as_bool(
                experiment_raw.get("start_foxglove", False),
                "experiment.start_foxglove",
            ),
            "continue_on_failure": as_bool(
                experiment_raw.get("continue_on_failure", True),
                "experiment.continue_on_failure",
            ),
            "ros_domain_id": ros_domain_id,
        },
        "bag": {
            "name": bag_name,
            "path": str(bag_path),
            "waypoints": str(waypoint_path),
        },
        "algorithms": normalize_algorithms(raw.get("algorithms")),
        "scan_variants": variants,
    }

    if config["experiment"]["playback_rate"] <= 0.0:
        raise ValueError("experiment.playback_rate must be greater than zero")
    if config["experiment"]["resource_interval"] <= 0.0:
        raise ValueError("experiment.resource_interval must be greater than zero")
    return config


def config_digest(config: dict[str, Any]) -> str:
    encoded = json.dumps(config, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def build_jobs(config: dict[str, Any]) -> list[dict[str, Any]]:
    jobs: list[dict[str, Any]] = []
    repetitions = config["experiment"]["repetitions"]
    seed = config["experiment"]["random_seed"]
    bag_name = config["bag"]["name"]
    for repetition in range(1, repetitions + 1):
        block = []
        for algorithm, settings in config["algorithms"].items():
            for variant in config["scan_variants"]:
                job_id = (
                    f"{bag_name}__{algorithm}__{variant}__rep{repetition:02d}"
                )
                block.append(
                    {
                        "id": job_id,
                        "bag": bag_name,
                        "algorithm": algorithm,
                        "scan_variant": variant,
                        "repetition": repetition,
                        "use_camera": settings["use_camera"],
                    }
                )
        random.Random(seed + repetition).shuffle(block)
        jobs.extend(block)
    for order, job in enumerate(jobs, start=1):
        job["order"] = order
    return jobs


def initial_state(config: dict[str, Any], jobs: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "experiment": config["experiment"]["name"],
        "bag": config["bag"]["name"],
        "config_sha256": config_digest(config),
        "created_at": now_iso(),
        "updated_at": now_iso(),
        "jobs": {
            job["id"]: {
                **job,
                "status": "pending",
                "attempts": 0,
                "result_directory": None,
                "exit_code": None,
                "error": None,
            }
            for job in jobs
        },
    }


def atomic_write_yaml(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def save_state(path: Path, state: dict[str, Any]) -> None:
    state["updated_at"] = now_iso()
    atomic_write_yaml(path, state)


def read_temperature_c() -> float | None:
    values = []
    for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            value = float(path.read_text(encoding="utf-8").strip())
        except (OSError, ValueError):
            continue
        if value > 1000.0:
            value /= 1000.0
        if -20.0 < value < 150.0:
            values.append(value)
    return max(values) if values else None


def wait_for_cooldown(config: dict[str, Any], previous_job_ran: bool) -> None:
    settings = config["experiment"]
    fixed_wait = settings["cooldown_seconds"] if previous_job_ran else 0.0
    if fixed_wait > 0.0:
        print(f"Cooling down for {fixed_wait:.0f}s before the next job...", flush=True)
        time.sleep(fixed_wait)

    threshold = settings["max_start_temperature_c"]
    if threshold is None:
        return
    temperature = read_temperature_c()
    if temperature is None:
        print("Temperature sensor unavailable; continuing without thermal gate.")
        return

    start = time.monotonic()
    last_report = float("-inf")
    while temperature > threshold:
        elapsed = time.monotonic() - start
        if elapsed > settings["max_cooldown_wait_seconds"]:
            raise RuntimeError(
                f"CPU stayed above {threshold:.1f} C for "
                f"{settings['max_cooldown_wait_seconds']}s"
            )
        if elapsed - last_report >= 30.0:
            print(
                f"CPU temperature {temperature:.1f} C; waiting for <= "
                f"{threshold:.1f} C...",
                flush=True,
            )
            last_report = elapsed
        time.sleep(5.0)
        temperature = read_temperature_c() or temperature


def read_key_values(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    if not path.is_file():
        return result
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" in line:
            key, value = line.split("=", maxsplit=1)
            result[key] = value
    return result


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def validate_result(result_dir: Path, exit_code: int) -> str | None:
    if exit_code != 0:
        return f"benchmark process exited with {exit_code}"
    run_config = read_key_values(result_dir / "run_config.txt")
    if run_config.get("launch_exit_code") != "0":
        return "run_config.txt does not report launch_exit_code=0"
    accuracy = load_json(result_dir / "metrics" / "waypoint_accuracy.json")
    if not accuracy:
        return "metrics/waypoint_accuracy.json is missing or invalid"
    resources = load_json(result_dir / "resources" / "summary.json")
    if not resources:
        return "resources/summary.json is missing or invalid"
    return None


def nested(data: dict[str, Any], *keys: str) -> Any:
    value: Any = data
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def cpu_frequency_summary(path: Path) -> tuple[float | None, float | None]:
    values = []
    if not path.is_file():
        return None, None
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            for row in csv.DictReader(handle):
                text = row.get("cpu_frequency_mhz", "")
                if text:
                    values.append(float(text))
    except (OSError, ValueError):
        return None, None
    if not values:
        return None, None
    return sum(values) / len(values), min(values)


def write_summary_csv(path: Path, state: dict[str, Any]) -> None:
    fields = [
        "job_id", "status", "bag", "algorithm", "scan_variant", "repetition",
        "attempts", "exit_code", "result_directory", "evaluated_count",
        "failed_waypoint_count", "mae_m", "rmse_m", "median_m", "max_error_m",
        "system_cpu_mean_percent", "system_cpu_max_percent",
        "memory_mean_mb", "memory_max_mb", "temperature_mean_c",
        "temperature_max_c", "cpu_frequency_mean_mhz", "cpu_frequency_min_mhz",
        "mapper_cpu_mean_one_core", "mapper_cpu_max_one_core",
        "mapper_rss_mean_mb", "mapper_rss_max_mb",
        "filter_cpu_mean_one_core", "filter_cpu_max_one_core",
        "wall_duration_s", "error",
    ]
    rows = []
    jobs = sorted(state["jobs"].values(), key=lambda job: int(job["order"]))
    for job in jobs:
        result_text = job.get("result_directory")
        result_dir = Path(result_text) if result_text else None
        accuracy = (
            load_json(result_dir / "metrics" / "waypoint_accuracy.json")
            if result_dir else {}
        )
        resources = (
            load_json(result_dir / "resources" / "summary.json")
            if result_dir else {}
        )
        run_config = (
            read_key_values(result_dir / "run_config.txt") if result_dir else {}
        )
        frequency_mean, frequency_min = (
            cpu_frequency_summary(result_dir / "resources" / "system.csv")
            if result_dir else (None, None)
        )
        component = nested(resources, "components", job["algorithm"]) or {}
        filter_component = nested(resources, "components", "dynamic_filter") or {}
        rows.append({
            "job_id": job["id"],
            "status": job["status"],
            "bag": job["bag"],
            "algorithm": job["algorithm"],
            "scan_variant": job["scan_variant"],
            "repetition": job["repetition"],
            "attempts": job["attempts"],
            "exit_code": job.get("exit_code"),
            "result_directory": result_text,
            "evaluated_count": accuracy.get("evaluated_count"),
            "failed_waypoint_count": accuracy.get("failed_count"),
            "mae_m": accuracy.get("mae_m"),
            "rmse_m": accuracy.get("rmse_m"),
            "median_m": accuracy.get("median_m"),
            "max_error_m": accuracy.get("max_error_m"),
            "system_cpu_mean_percent": nested(resources, "system", "cpu_percent", "mean"),
            "system_cpu_max_percent": nested(resources, "system", "cpu_percent", "max"),
            "memory_mean_mb": nested(resources, "system", "memory_used_mb", "mean"),
            "memory_max_mb": nested(resources, "system", "memory_used_mb", "max"),
            "temperature_mean_c": nested(resources, "system", "temperature_c", "mean"),
            "temperature_max_c": nested(resources, "system", "temperature_c", "max"),
            "cpu_frequency_mean_mhz": frequency_mean,
            "cpu_frequency_min_mhz": frequency_min,
            "mapper_cpu_mean_one_core": nested(component, "cpu_percent_one_core", "mean"),
            "mapper_cpu_max_one_core": nested(component, "cpu_percent_one_core", "max"),
            "mapper_rss_mean_mb": nested(component, "rss_mb", "mean"),
            "mapper_rss_max_mb": nested(component, "rss_mb", "max"),
            "filter_cpu_mean_one_core": nested(filter_component, "cpu_percent_one_core", "mean"),
            "filter_cpu_max_one_core": nested(filter_component, "cpu_percent_one_core", "max"),
            "wall_duration_s": run_config.get("wall_duration_s"),
            "error": job.get("error"),
        })

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def write_aggregate_csv(path: Path, detailed_path: Path) -> None:
    metric_fields = [
        "mae_m", "rmse_m", "median_m", "max_error_m",
        "system_cpu_mean_percent", "memory_mean_mb", "temperature_max_c",
        "cpu_frequency_min_mhz", "mapper_cpu_mean_one_core",
        "mapper_rss_mean_mb", "filter_cpu_mean_one_core", "wall_duration_s",
    ]
    groups: dict[tuple[str, str], list[dict[str, str]]] = {}
    with detailed_path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            key = (row["algorithm"], row["scan_variant"])
            groups.setdefault(key, []).append(row)

    fields = ["algorithm", "scan_variant", "completed_runs", "planned_runs"]
    for metric in metric_fields:
        fields.extend((f"{metric}_mean", f"{metric}_std"))

    rows = []
    for (algorithm, variant), group in sorted(groups.items()):
        completed = [row for row in group if row["status"] == "completed"]
        output: dict[str, Any] = {
            "algorithm": algorithm,
            "scan_variant": variant,
            "completed_runs": len(completed),
            "planned_runs": len(group),
        }
        for metric in metric_fields:
            values = []
            for row in completed:
                text = row.get(metric, "")
                if text not in {"", None}:
                    try:
                        values.append(float(text))
                    except ValueError:
                        pass
            output[f"{metric}_mean"] = (
                statistics.fmean(values) if values else None
            )
            output[f"{metric}_std"] = (
                statistics.stdev(values) if len(values) >= 2 else None
            )
        rows.append(output)

    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def write_summaries(experiment_root: Path, state: dict[str, Any]) -> None:
    detailed = experiment_root / "summary.csv"
    write_summary_csv(detailed, state)
    write_aggregate_csv(experiment_root / "aggregate_summary.csv", detailed)


def command_output(command: list[str], cwd: Path) -> str | None:
    try:
        result = subprocess.run(
            command,
            cwd=cwd,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip() or None


def snapshot_experiment(
    config_path: Path,
    config: dict[str, Any],
    experiment_root: Path,
) -> None:
    experiment_root.mkdir(parents=True, exist_ok=True)
    manifest_copy = experiment_root / "experiment.yaml"
    if not manifest_copy.exists() and config_path.resolve() != manifest_copy.resolve():
        shutil.copy2(config_path, manifest_copy)

    workspace = Path(config["workspace"])
    snapshot_dir = experiment_root / "configuration_snapshot"
    snapshot_dir.mkdir(exist_ok=True)
    candidates = [
        workspace / "src/xgo_driver_bridge/config/slam_toolbox_robot.yaml",
        workspace / "src/xgo_driver_bridge/config/cartographer_robot_2d.lua",
        workspace / "src/xgo_driver_bridge/config/dynamic_scan_filter_robot.yaml",
        workspace / "src/xgo_driver_bridge/launch/rtabmap_robot.launch.py",
        workspace / "src/xgo_driver_bridge/launch/headless_bag_benchmark.launch.py",
        workspace / "scripts/robot_bag_benchmark.sh",
        workspace / "scripts/run_benchmark_matrix.py",
    ]
    hashes = {}
    for source in candidates:
        if not source.is_file():
            continue
        relative_name = source.relative_to(workspace).as_posix().replace("/", "__")
        destination = snapshot_dir / relative_name
        if not destination.exists():
            shutil.copy2(source, destination)
        hashes[str(source.relative_to(workspace))] = hashlib.sha256(
            source.read_bytes()
        ).hexdigest()

    metadata_path = experiment_root / "experiment_metadata.yaml"
    if not metadata_path.exists():
        metadata = {
            "created_at": now_iso(),
            "hostname": socket.gethostname(),
            "workspace": str(workspace),
            "config_sha256": config_digest(config),
            "git_commit": command_output(
                ["git", "rev-parse", "HEAD"], workspace
            ),
            "git_status": command_output(
                ["git", "status", "--short"], workspace
            ),
            "ros_distro": os.environ.get("ROS_DISTRO"),
            "configuration_sha256": hashes,
        }
        atomic_write_yaml(metadata_path, metadata)


def progress_counts(state: dict[str, Any]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for job in state["jobs"].values():
        status = job["status"]
        counts[status] = counts.get(status, 0) + 1
    return counts


def print_status(state: dict[str, Any]) -> None:
    counts = progress_counts(state)
    total = len(state["jobs"])
    parts = [f"{key}={counts[key]}" for key in sorted(counts)]
    print(f"Experiment {state['experiment']}: {total} jobs; " + ", ".join(parts))
    running_or_failed = [
        job for job in state["jobs"].values()
        if job["status"] in {"running", "interrupted", "failed"}
    ]
    for job in sorted(running_or_failed, key=lambda item: int(item["order"])):
        print(f"  {job['status']:11s} {job['id']}: {job.get('error') or ''}")


def run_job(
    job: dict[str, Any],
    config: dict[str, Any],
    experiment_root: Path,
    runner_log: Path,
) -> tuple[int, Path | None]:
    workspace = Path(config["workspace"])
    job_output_root = (
        experiment_root
        / "bags"
        / config["bag"]["name"]
        / job["algorithm"]
        / job["scan_variant"]
    )
    job_output_root.mkdir(parents=True, exist_ok=True)
    benchmark_script = workspace / "scripts" / "robot_bag_benchmark.sh"
    command = [
        "bash",
        str(benchmark_script),
        job["algorithm"],
        job["scan_variant"],
        config["bag"]["path"],
        config["bag"]["waypoints"],
    ]
    settings = config["experiment"]
    environment = os.environ.copy()
    environment.update({
        "WORKSPACE": str(workspace),
        "OUTPUT_ROOT": str(job_output_root),
        "RESOURCE_INTERVAL": str(settings["resource_interval"]),
        "PLAYBACK_RATE": str(settings["playback_rate"]),
        "NORMALIZE_SCAN": str(settings["normalize_scan"]).lower(),
        "START_FOXGLOVE": str(settings["start_foxglove"]).lower(),
        "POST_PLAYBACK_DELAY_SEC": str(settings["post_playback_delay_sec"]),
        "USE_CAMERA": str(job["use_camera"]).lower(),
        "RUN_TAG": f"rep{int(job['repetition']):02d}",
    })
    if settings["ros_domain_id"] is not None:
        environment["ROS_DOMAIN_ID"] = str(settings["ros_domain_id"])

    result_dir: Path | None = None
    with runner_log.open("a", encoding="utf-8") as log:
        log.write(f"\n[{now_iso()}] START {job['id']}\n")
        log.write("command=" + " ".join(command) + "\n")
        log.flush()
        process = subprocess.Popen(
            command,
            cwd=workspace,
            env=environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
            start_new_session=True,
        )
        try:
            assert process.stdout is not None
            for line in process.stdout:
                print(line, end="", flush=True)
                log.write(line)
                match = RESULT_PATTERN.search(line)
                if match:
                    result_dir = Path(match.group(1)).resolve()
            exit_code = process.wait()
        except KeyboardInterrupt:
            os.killpg(process.pid, signal.SIGINT)
            try:
                exit_code = process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                exit_code = process.wait(timeout=10)
            log.write(f"[{now_iso()}] INTERRUPTED exit_code={exit_code}\n")
            log.flush()
            raise
        log.write(f"[{now_iso()}] END exit_code={exit_code}\n")
    return exit_code, result_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run every configured SLAM/filter/repetition benchmark for one bag "
            "with resumable state and a combined CSV summary."
        )
    )
    parser.add_argument("config", type=Path, help="Benchmark matrix YAML file")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="Print jobs only")
    mode.add_argument("--status", action="store_true", help="Show saved progress")
    parser.add_argument(
        "--retry-failed",
        action="store_true",
        help="Include previously failed jobs when resuming",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config_path = args.config.expanduser().resolve()
    try:
        config = normalize_config(config_path)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"Configuration error: {exc}", file=sys.stderr)
        return 2

    jobs = build_jobs(config)
    experiment_root = (
        Path(config["results_root"]) / config["experiment"]["name"]
    )
    state_path = experiment_root / "state.yaml"
    summary_path = experiment_root / "summary.csv"
    runner_log = experiment_root / "runner.log"

    if args.dry_run:
        print(f"Experiment: {config['experiment']['name']}")
        print(f"Bag:        {config['bag']['path']}")
        print(f"Waypoints:  {config['bag']['waypoints']}")
        print(f"Results:    {experiment_root}")
        print(f"Jobs:       {len(jobs)}")
        for job in jobs:
            print(
                f"  {job['order']:02d}. {job['algorithm']:12s} "
                f"{job['scan_variant']:8s} repetition {job['repetition']}"
            )
        return 0

    if state_path.exists():
        state = load_yaml(state_path)
        if state.get("config_sha256") != config_digest(config):
            print(
                "The configuration differs from the state file. Use a new "
                "experiment.name instead of modifying a running experiment.",
                file=sys.stderr,
            )
            return 2
    else:
        state = initial_state(config, jobs)

    if args.status:
        print_status(state)
        if summary_path.exists():
            print(f"Summary: {summary_path}")
        return 0

    experiment_root.mkdir(parents=True, exist_ok=True)
    lock_handle = (experiment_root / ".runner.lock").open("w", encoding="utf-8")
    try:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print(
            f"Another matrix runner is already active for {experiment_root}.",
            file=sys.stderr,
        )
        return 2

    bag_path = Path(config["bag"]["path"])
    waypoint_path = Path(config["bag"]["waypoints"])
    benchmark_script = Path(config["workspace"]) / "scripts/robot_bag_benchmark.sh"
    for required in (bag_path / "metadata.yaml", waypoint_path, benchmark_script):
        if not required.is_file():
            print(f"Required input does not exist: {required}", file=sys.stderr)
            return 2

    snapshot_experiment(config_path, config, experiment_root)
    for job in state["jobs"].values():
        if job["status"] == "running":
            job["status"] = "interrupted"
            job["error"] = "Runner stopped while this job was active"
    save_state(state_path, state)
    write_summaries(experiment_root, state)

    eligible = {"pending", "interrupted"}
    if args.retry_failed:
        eligible.add("failed")
    ordered_jobs = sorted(state["jobs"].values(), key=lambda job: int(job["order"]))
    pending_jobs = [job for job in ordered_jobs if job["status"] in eligible]
    if not pending_jobs:
        print_status(state)
        print("No eligible jobs remain.")
        return 0

    print(
        f"Starting {len(pending_jobs)} eligible jobs for "
        f"{config['experiment']['name']}. Results: {experiment_root}",
        flush=True,
    )
    previous_job_ran = False
    try:
        for sequence, job in enumerate(pending_jobs, start=1):
            wait_for_cooldown(config, previous_job_ran)
            print(
                f"\n=== Job {sequence}/{len(pending_jobs)}: {job['id']} ===",
                flush=True,
            )
            job["status"] = "running"
            job["attempts"] = int(job.get("attempts", 0)) + 1
            job["started_at"] = now_iso()
            job["ended_at"] = None
            job["exit_code"] = None
            job["error"] = None
            save_state(state_path, state)

            invocation_error = None
            try:
                exit_code, result_dir = run_job(
                    job, config, experiment_root, runner_log
                )
            except KeyboardInterrupt:
                job["status"] = "interrupted"
                job["ended_at"] = now_iso()
                job["error"] = "Interrupted by user"
                save_state(state_path, state)
                write_summaries(experiment_root, state)
                print("\nMatrix interrupted; rerun the same command to resume.")
                return 130
            except OSError as exc:
                exit_code = 127
                result_dir = None
                invocation_error = f"could not start benchmark: {exc}"

            job["exit_code"] = exit_code
            job["ended_at"] = now_iso()
            job["result_directory"] = str(result_dir) if result_dir else None
            error = invocation_error or (
                validate_result(result_dir, exit_code)
                if result_dir else "benchmark did not report a result directory"
            )
            if error is None:
                job["status"] = "completed"
                print(f"Completed: {job['id']}")
            else:
                job["status"] = "failed"
                job["error"] = error
                print(f"FAILED: {job['id']}: {error}", file=sys.stderr)
            save_state(state_path, state)
            write_summaries(experiment_root, state)
            previous_job_ran = True

            if error and not config["experiment"]["continue_on_failure"]:
                print("Stopping because continue_on_failure is false.")
                break
    except RuntimeError as exc:
        print(f"Runner stopped: {exc}", file=sys.stderr)
        save_state(state_path, state)
        write_summaries(experiment_root, state)
        return 1
    except KeyboardInterrupt:
        for job in state["jobs"].values():
            if job["status"] == "running":
                job["status"] = "interrupted"
                job["ended_at"] = now_iso()
                job["error"] = "Interrupted by user"
        save_state(state_path, state)
        write_summaries(experiment_root, state)
        print("\nMatrix interrupted; rerun the same command to resume.")
        return 130

    print_status(state)
    print(f"State:   {state_path}")
    print(f"Summary: {summary_path}")
    failed = progress_counts(state).get("failed", 0)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
