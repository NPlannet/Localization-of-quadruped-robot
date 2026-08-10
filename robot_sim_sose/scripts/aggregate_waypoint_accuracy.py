#!/usr/bin/env python3
"""Aggregate waypoint_accuracy.json across multiple robot_bag_benchmark.sh runs.

Scans run directories under evaluation/runs/, reads run_config.txt (for
slam_method, scan_variant, velocity_scale, source_bag_name, playback_rate)
and metrics/waypoint_accuracy.json (for aligned position error metrics),
groups by condition, and writes per-run and aggregated (mean/std across
repetitions) CSV files.

Usage:
  python3 scripts/aggregate_waypoint_accuracy.py \
    --runs-dir evaluation/runs \
    --output-dir evaluation/results/velocity_matrix

Only runs whose directory name matches --prefix (default: bag_) and that
contain both run_config.txt and metrics/waypoint_accuracy.json are used.
Runs with failed_count > 0 or missing evaluated_count are flagged and
excluded from the aggregate by default (use --include-partial to keep them).
"""

import argparse
import csv
import json
import statistics
from pathlib import Path


def parse_run_config(path: Path) -> dict:
    config = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or "=" not in line:
            continue
        key, _, value = line.partition("=")
        config[key] = value
    return config


def load_run(run_dir: Path) -> dict | None:
    config_path = run_dir / "run_config.txt"
    metrics_path = run_dir / "metrics" / "waypoint_accuracy.json"
    if not config_path.is_file() or not metrics_path.is_file():
        return None

    config = parse_run_config(config_path)
    try:
        metrics = json.loads(metrics_path.read_text())
    except json.JSONDecodeError:
        return None

    aligned = metrics.get("aligned_position_metrics", {})
    return {
        "run_name": run_dir.name,
        "slam_method": config.get("slam_method", ""),
        "scan_variant": config.get("scan_variant", ""),
        "velocity_scale": config.get("velocity_scale", "1.0"),
        "playback_rate": config.get("playback_rate", ""),
        "source_bag_name": config.get("source_bag_name", ""),
        "run_tag": config.get("run_tag", ""),
        "evaluated_count": metrics.get("evaluated_count", 0),
        "failed_count": metrics.get("failed_count", 0),
        "revisit_count": metrics.get("revisit_count", 0),
        "mae_m": aligned.get("mae_m"),
        "rmse_m": aligned.get("rmse_m"),
        "median_m": aligned.get("median_m"),
        "p95_m": aligned.get("p95_m"),
        "max_m": aligned.get("max_m"),
        "std_m": aligned.get("std_m"),
    }


def is_complete(run: dict) -> bool:
    return (
        run["evaluated_count"] not in (0, None)
        and run["failed_count"] == 0
        and run["mae_m"] is not None
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs-dir", default="evaluation/runs")
    parser.add_argument("--output-dir", default="evaluation/results/velocity_matrix")
    parser.add_argument("--prefix", default="bag_")
    parser.add_argument(
        "--include-partial",
        action="store_true",
        help="Include runs with failed_count > 0 in the aggregate.",
    )
    args = parser.parse_args()

    runs_dir = Path(args.runs_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    per_run_rows = []
    for run_dir in sorted(runs_dir.iterdir()):
        if not run_dir.is_dir() or not run_dir.name.startswith(args.prefix):
            continue
        run = load_run(run_dir)
        if run is None:
            continue
        per_run_rows.append(run)

    if not per_run_rows:
        print(f"No usable runs found under {runs_dir} with prefix '{args.prefix}'.")
        return

    per_run_path = output_dir / "per_run.csv"
    fieldnames = list(per_run_rows[0].keys())
    with per_run_path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(per_run_rows)

    usable_rows = [
        r for r in per_run_rows if args.include_partial or is_complete(r)
    ]
    excluded = len(per_run_rows) - len(usable_rows)

    groups: dict[tuple, list[dict]] = {}
    group_key = ("source_bag_name", "slam_method", "scan_variant", "velocity_scale")
    for row in usable_rows:
        key = tuple(row[k] for k in group_key)
        groups.setdefault(key, []).append(row)

    agg_rows = []
    for key, rows in sorted(groups.items()):
        mae_values = [r["mae_m"] for r in rows if r["mae_m"] is not None]
        rmse_values = [r["rmse_m"] for r in rows if r["rmse_m"] is not None]
        max_values = [r["max_m"] for r in rows if r["max_m"] is not None]
        agg_rows.append({
            "source_bag_name": key[0],
            "slam_method": key[1],
            "scan_variant": key[2],
            "velocity_scale": key[3],
            "n_runs": len(rows),
            "mae_mean_m": statistics.mean(mae_values) if mae_values else None,
            "mae_std_m": statistics.stdev(mae_values) if len(mae_values) > 1 else 0.0,
            "rmse_mean_m": statistics.mean(rmse_values) if rmse_values else None,
            "rmse_std_m": statistics.stdev(rmse_values) if len(rmse_values) > 1 else 0.0,
            "max_mean_m": statistics.mean(max_values) if max_values else None,
        })

    agg_path = output_dir / "aggregate.csv"
    with agg_path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(agg_rows[0].keys()))
        writer.writeheader()
        writer.writerows(agg_rows)

    print(f"Runs scanned: {len(per_run_rows)}")
    print(f"Runs excluded (incomplete/failed): {excluded}")
    print(f"Conditions aggregated: {len(agg_rows)}")
    print(f"Per-run CSV:   {per_run_path}")
    print(f"Aggregate CSV: {agg_path}")


if __name__ == "__main__":
    main()