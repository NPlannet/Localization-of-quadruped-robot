#!/usr/bin/env python3
"""Record low-overhead system and ROS-process resource measurements.

This script intentionally uses Linux /proc and /sys instead of psutil so it can
run in the robot container without adding another Python dependency.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import statistics
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any


RUNNING = True


def stop_requested(_signum: int, _frame: Any) -> None:
    global RUNNING
    RUNNING = False


def timestamp_now() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def read_system_cpu() -> tuple[int, int]:
    fields = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()
    values = [int(value) for value in fields[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def read_memory() -> tuple[float, float, float]:
    values: dict[str, int] = {}
    for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
        key, value = line.split(":", maxsplit=1)
        values[key] = int(value.strip().split()[0])

    total_kib = values["MemTotal"]
    available_kib = values.get("MemAvailable", values.get("MemFree", 0))
    used_kib = total_kib - available_kib
    used_percent = 100.0 * used_kib / total_kib if total_kib else math.nan
    return used_kib / 1024.0, available_kib / 1024.0, used_percent


def read_temperature_c() -> float | None:
    temperatures = []
    for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            value = float(path.read_text(encoding="utf-8").strip())
        except (OSError, ValueError):
            continue
        if value > 1000.0:
            value /= 1000.0
        if -20.0 < value < 150.0:
            temperatures.append(value)
    return max(temperatures) if temperatures else None


def read_cpu_frequency_mhz() -> float | None:
    frequencies = []
    pattern = "/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq"
    for path in Path("/").glob(pattern.lstrip("/")):
        try:
            frequencies.append(float(path.read_text(encoding="utf-8").strip()) / 1000.0)
        except (OSError, ValueError):
            continue
    return statistics.fmean(frequencies) if frequencies else None


def classify_process(command: str, comm: str) -> str | None:
    text = f"{comm} {command}".lower()

    if "monitor_robot_resources.py" in text:
        return None
    if (
        "ros2 bag play" in text
        or "/rosbag2_transport/play" in text
        or "rosbag2_transport play" in text
    ):
        return "bag_player"
    if (
        "ros2 bag record" in text
        or "/rosbag2_transport/record" in text
        or "rosbag2_transport record" in text
    ):
        return "bag_recorder"
    if "ros2 launch" in text or "robot_sensor_bringup.launch.py" in text:
        return "ros_launch"

    if "async_slam_toolbox_node" in text or "/lib/slam_toolbox/" in text:
        return "slam_toolbox"
    if (
        "cartographer_node" in text
        or "cartographer_occupancy_grid_node" in text
        or "/lib/cartographer_ros/" in text
    ):
        return "cartographer"
    if "/lib/rtabmap_slam/rtabmap" in text:
        return "rtabmap"

    if "scan_normalizer_node" in text:
        return "scan_normalizer"
    if "dynamic_scan_filter_node" in text or "/lib/dynamic_scan_filter/" in text:
        return "dynamic_filter"
    if "xgo_offline_odom_node" in text:
        return "offline_odom"
    if "waypoint_accuracy_node" in text:
        return "waypoint_evaluator"
    if "xgo_bridge_node" in text:
        return "xgo_bridge"
    if "ldlidar" in text:
        return "lidar"
    if "camera_node" in text and "camera_ros" in text:
        return "camera"
    if "foxglove_bridge" in text:
        return "foxglove"
    if "static_transform_publisher" in text:
        return "static_tf"
    if "/opt/ros/" in text and "/lib/" in text:
        return "other_ros"
    return None


def read_rss_mb(pid: int) -> float:
    try:
        for line in Path(f"/proc/{pid}/status").read_text(
            encoding="utf-8"
        ).splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        pass
    return 0.0


def read_processes() -> dict[tuple[int, int], dict[str, Any]]:
    processes: dict[tuple[int, int], dict[str, Any]] = {}
    own_pid = os.getpid()

    for path in Path("/proc").iterdir():
        if not path.name.isdigit():
            continue
        pid = int(path.name)
        if pid == own_pid:
            continue

        try:
            stat_text = (path / "stat").read_text(encoding="utf-8")
            close_paren = stat_text.rfind(")")
            open_paren = stat_text.find("(")
            comm = stat_text[open_paren + 1 : close_paren]
            fields = stat_text[close_paren + 2 :].split()
            cpu_ticks = int(fields[11]) + int(fields[12])
            threads = int(fields[17])
            start_ticks = int(fields[19])
            raw_command = (path / "cmdline").read_bytes()
            command = raw_command.replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            ).strip()
        except (
            FileNotFoundError,
            PermissionError,
            ProcessLookupError,
            ValueError,
        ):
            continue

        component = classify_process(command, comm)
        if component is None:
            continue

        processes[(pid, start_ticks)] = {
            "pid": pid,
            "component": component,
            "command": command or comm,
            "cpu_ticks": cpu_ticks,
            "rss_mb": read_rss_mb(pid),
            "threads": threads,
        }

    return processes


def rounded(value: float | None, digits: int = 3) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return round(value, digits)


def metric_summary(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"mean": None, "max": None}
    return {
        "mean": rounded(statistics.fmean(values)),
        "max": rounded(max(values)),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Log whole-system and per-component CPU/RAM use from /proc. "
            "Stop with Ctrl+C or SIGINT."
        )
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Directory for system.csv, processes.csv, and summary.json.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Sampling period in seconds (default: 1.0).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Optional duration in seconds; zero means run until stopped.",
    )
    parser.add_argument(
        "--label",
        default="",
        help="Run label copied into summary.json.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.interval <= 0.0:
        raise SystemExit("--interval must be greater than zero")
    if args.duration < 0.0:
        raise SystemExit("--duration cannot be negative")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    system_path = args.output_dir / "system.csv"
    process_path = args.output_dir / "processes.csv"
    summary_path = args.output_dir / "summary.json"

    signal.signal(signal.SIGINT, stop_requested)
    signal.signal(signal.SIGTERM, stop_requested)

    logical_cpus = os.cpu_count() or 1
    clock_ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    started_wall = timestamp_now()
    started_monotonic = time.monotonic()
    previous_sample_time = started_monotonic
    previous_total, previous_idle = read_system_cpu()
    previous_processes = read_processes()
    next_sample_time = started_monotonic + args.interval

    system_cpu_samples: list[float] = []
    system_memory_samples: list[float] = []
    temperature_samples: list[float] = []
    component_cpu_samples: dict[str, list[float]] = defaultdict(list)
    component_rss_samples: dict[str, list[float]] = defaultdict(list)
    component_process_max: dict[str, int] = defaultdict(int)
    sample_count = 0

    system_fields = [
        "timestamp",
        "elapsed_s",
        "system_cpu_percent",
        "memory_used_mb",
        "memory_available_mb",
        "memory_used_percent",
        "load_1",
        "load_5",
        "load_15",
        "temperature_c",
        "cpu_frequency_mhz",
    ]
    process_fields = [
        "timestamp",
        "elapsed_s",
        "component",
        "pid",
        "cpu_percent_one_core",
        "cpu_percent_total_capacity",
        "rss_mb",
        "threads",
        "command",
    ]

    with system_path.open("w", newline="", encoding="utf-8") as system_file, (
        process_path.open("w", newline="", encoding="utf-8")
    ) as process_file:
        system_writer = csv.DictWriter(system_file, fieldnames=system_fields)
        process_writer = csv.DictWriter(process_file, fieldnames=process_fields)
        system_writer.writeheader()
        process_writer.writeheader()

        while RUNNING:
            remaining = next_sample_time - time.monotonic()
            if remaining > 0.0:
                time.sleep(min(remaining, 0.2))
                continue

            now_monotonic = time.monotonic()
            elapsed = now_monotonic - started_monotonic
            interval = max(now_monotonic - previous_sample_time, 1e-9)
            now_timestamp = timestamp_now()

            total, idle = read_system_cpu()
            delta_total = total - previous_total
            delta_idle = idle - previous_idle
            system_cpu = (
                100.0 * (delta_total - delta_idle) / delta_total
                if delta_total > 0
                else math.nan
            )
            memory_used, memory_available, memory_percent = read_memory()
            load_1, load_5, load_15 = os.getloadavg()
            temperature = read_temperature_c()
            frequency = read_cpu_frequency_mhz()

            system_writer.writerow(
                {
                    "timestamp": now_timestamp,
                    "elapsed_s": rounded(elapsed),
                    "system_cpu_percent": rounded(system_cpu),
                    "memory_used_mb": rounded(memory_used),
                    "memory_available_mb": rounded(memory_available),
                    "memory_used_percent": rounded(memory_percent),
                    "load_1": rounded(load_1),
                    "load_5": rounded(load_5),
                    "load_15": rounded(load_15),
                    "temperature_c": rounded(temperature),
                    "cpu_frequency_mhz": rounded(frequency),
                }
            )

            current_processes = read_processes()
            aggregate_cpu: dict[str, float] = defaultdict(float)
            aggregate_rss: dict[str, float] = defaultdict(float)
            aggregate_count: dict[str, int] = defaultdict(int)

            for key, process in current_processes.items():
                previous = previous_processes.get(key)
                if previous is None:
                    process_cpu = 0.0
                else:
                    delta_ticks = process["cpu_ticks"] - previous["cpu_ticks"]
                    process_cpu = 100.0 * delta_ticks / clock_ticks / interval

                component = process["component"]
                aggregate_cpu[component] += process_cpu
                aggregate_rss[component] += process["rss_mb"]
                aggregate_count[component] += 1
                process_writer.writerow(
                    {
                        "timestamp": now_timestamp,
                        "elapsed_s": rounded(elapsed),
                        "component": component,
                        "pid": process["pid"],
                        "cpu_percent_one_core": rounded(process_cpu),
                        "cpu_percent_total_capacity": rounded(
                            process_cpu / logical_cpus
                        ),
                        "rss_mb": rounded(process["rss_mb"]),
                        "threads": process["threads"],
                        "command": process["command"],
                    }
                )

            for component, cpu_value in aggregate_cpu.items():
                component_cpu_samples[component].append(cpu_value)
                component_rss_samples[component].append(aggregate_rss[component])
                component_process_max[component] = max(
                    component_process_max[component],
                    aggregate_count[component],
                )

            if math.isfinite(system_cpu):
                system_cpu_samples.append(system_cpu)
            system_memory_samples.append(memory_used)
            if temperature is not None:
                temperature_samples.append(temperature)

            system_file.flush()
            process_file.flush()
            sample_count += 1
            previous_sample_time = now_monotonic
            previous_total, previous_idle = total, idle
            previous_processes = current_processes
            next_sample_time += args.interval
            if next_sample_time < now_monotonic:
                next_sample_time = now_monotonic + args.interval

            if args.duration > 0.0 and elapsed >= args.duration:
                break

    ended_wall = timestamp_now()
    duration = time.monotonic() - started_monotonic
    components = {}
    for component in sorted(component_cpu_samples):
        cpu_one_core = metric_summary(component_cpu_samples[component])
        components[component] = {
            "cpu_percent_one_core": cpu_one_core,
            "cpu_percent_total_capacity": {
                key: rounded(value / logical_cpus) if value is not None else None
                for key, value in cpu_one_core.items()
            },
            "rss_mb": metric_summary(component_rss_samples[component]),
            "max_process_count": component_process_max[component],
        }

    summary = {
        "label": args.label,
        "started_at": started_wall,
        "ended_at": ended_wall,
        "duration_s": rounded(duration),
        "sample_interval_s": args.interval,
        "sample_count": sample_count,
        "logical_cpu_count": logical_cpus,
        "definitions": {
            "system_cpu_percent": (
                "Busy fraction of all logical CPUs together; 100% means the "
                "entire Raspberry Pi CPU is busy."
            ),
            "cpu_percent_one_core": (
                "Top-style process CPU; 100% equals one fully occupied logical "
                "CPU and a multithreaded component can exceed 100%."
            ),
            "cpu_percent_total_capacity": (
                "Process CPU divided by logical_cpu_count; 100% equals all "
                "logical CPUs fully occupied."
            ),
            "rss_mb": (
                "Resident memory. Component totals sum process RSS and can "
                "double-count shared library pages."
            ),
        },
        "system": {
            "cpu_percent": metric_summary(system_cpu_samples),
            "memory_used_mb": metric_summary(system_memory_samples),
            "temperature_c": metric_summary(temperature_samples),
        },
        "components": components,
        "files": {
            "system_samples": system_path.name,
            "process_samples": process_path.name,
        },
    }
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=False) + "\n",
        encoding="utf-8",
    )
    print(f"Resource measurements saved to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
