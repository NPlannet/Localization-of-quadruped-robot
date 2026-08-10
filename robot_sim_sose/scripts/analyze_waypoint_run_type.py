#!/usr/bin/env python3
"""Aggregate waypoint_accuracy.json across all runs of one run type.

A "run type" is the shared prefix of run directories under
evaluation/runs/, e.g. all directories matching:

    bag_cartographer_raw_rtab_filtered_run_wp_3_run001
    bag_cartographer_raw_rtab_filtered_run_wp_3_run002
    ...

are one run type: "cartographer_raw_rtab_filtered_run_wp_3" (the leading
"bag_" and trailing "_runNNN" are stripped automatically -- you can also
just pass the full directory prefix, see --prefix).

For each run of the matching type, this script reads:
    <run_dir>/metrics/waypoint_accuracy.json
    <run_dir>/run_config.txt

It then, per waypoint label, computes across ALL matching observations
(all runs, including revisits within a run):
    count, mean, std (population), min, max, median
for both aligned_error_m and raw_error_m.

Output is written to:
    evaluation/waypoint_evaluation/<run_type>_runs/
        per_waypoint_stats.csv
        overall_stats.csv
        configs_summary.csv
        source_runs.txt

Usage:
    python3 scripts/analyze_waypoint_run_type.py [<run_type_or_prefix>] \
        [--runs-dir evaluation/runs] \
        [--output-root evaluation/waypoint_evaluation]

If <run_type_or_prefix> is omitted, every distinct run type found under
--runs-dir is discovered automatically and analyzed one after another.

Examples:
    # Analyze one specific run type
    python3 scripts/analyze_waypoint_run_type.py \
        cartographer_raw_rtab_filtered_run_wp_3

    python3 scripts/analyze_waypoint_run_type.py \
        bag_cartographer_raw_rtab_filtered_run_wp_3

    # Analyze every run type found under evaluation/runs
    python3 scripts/analyze_waypoint_run_type.py
"""

import argparse
import csv
import json
import re
import statistics
import sys
from pathlib import Path


# Known slam_method values used as the leading token of every run type
# (run_type = "<slam_method>_<scan_variant>_<source_bag_name>[_<param_suffix>]",
# see robot_bag_benchmark.sh RUN_BASE). Longest first so "slam_toolbox" (which
# itself contains an underscore) is matched before a shorter accidental
# prefix could. Kept in sync with analyze_comparison.py.
ALGORITHMS = ['slam_toolbox', 'cartographer', 'rtabmap']
SCAN_VARIANTS = ('raw', 'filtered')


def split_algorithm_and_scan(run_type: str):
    """Split a run type into (algorithm, scan_variant, rest), or (None, None, None)."""
    for algo in ALGORITHMS:
        prefix = f'{algo}_'
        if not run_type.startswith(prefix):
            continue
        rest = run_type[len(prefix):]
        for scan_variant in SCAN_VARIANTS:
            sv_prefix = f'{scan_variant}_'
            if rest.startswith(sv_prefix):
                return algo, scan_variant, rest[len(sv_prefix):]
    return None, None, None


def strip_run_suffix(dirname: str) -> str:
    """Remove a trailing _runNNN (and optional trailing digits) if present."""
    return re.sub(r'_run\d+$', '', dirname)


def normalize_type(run_type: str) -> str:
    """Normalize a user-provided run type to match directory naming.

    Accepts either the bare type ("cartographer_raw_..._wp_3") or the
    directory-style prefix ("bag_cartographer_raw_..._wp_3").
    """
    run_type = run_type.strip().rstrip('_')
    if run_type.startswith('bag_'):
        run_type = run_type[len('bag_'):]
    return run_type


def find_matching_runs(runs_dir: Path, run_type: str):
    normalized = normalize_type(run_type)
    matches = []
    for entry in sorted(runs_dir.iterdir()):
        if not entry.is_dir():
            continue
        name = entry.name
        if not name.startswith('bag_'):
            continue
        base = strip_run_suffix(name)
        base_without_bag = base[len('bag_'):]
        if base_without_bag == normalized:
            matches.append(entry)
    return matches


def read_run_config(run_dir: Path):
    config_path = run_dir / 'run_config.txt'
    config = {}
    if not config_path.is_file():
        return config
    for line in config_path.read_text().splitlines():
        line = line.strip()
        if not line or '=' not in line:
            continue
        key, _, value = line.partition('=')
        config[key.strip()] = value.strip()
    return config


def read_waypoint_accuracy(run_dir: Path):
    metrics_path = run_dir / 'metrics' / 'waypoint_accuracy.json'
    if not metrics_path.is_file() or metrics_path.stat().st_size == 0:
        return None
    with metrics_path.open() as f:
        try:
            return json.load(f)
        except json.JSONDecodeError as exc:
            print(f'  WARNING: could not parse {metrics_path}: {exc}', file=sys.stderr)
            return None


def population_std(values):
    if len(values) < 2:
        return 0.0
    return statistics.pstdev(values)


def compute_stats(values):
    return {
        'count': len(values),
        'mean': statistics.mean(values),
        'std': population_std(values),
        'min': min(values),
        'max': max(values),
        'median': statistics.median(values),
    }


def discover_run_types(runs_dir: Path):
    """Find every distinct run type present under runs_dir, in first-seen order."""
    seen = []
    for entry in sorted(runs_dir.iterdir()):
        if not entry.is_dir() or not entry.name.startswith('bag_'):
            continue
        base = strip_run_suffix(entry.name)
        run_type = base[len('bag_'):]
        if run_type and run_type not in seen:
            seen.append(run_type)
    return seen


def process_run_type(runs_dir: Path, run_type: str, output_root: Path) -> bool:
    """Analyze one run type. Returns True on success, False if nothing usable found."""
    normalized_type = normalize_type(run_type)
    matching_runs = find_matching_runs(runs_dir, run_type)

    if not matching_runs:
        print(f'No run directories found matching type "{normalized_type}" under {runs_dir}', file=sys.stderr)
        return False

    algo, scan_variant, _ = split_algorithm_and_scan(normalized_type)
    if algo is not None:
        output_dir = output_root / f'{algo}_{scan_variant}' / f'{normalized_type}_runs'
    else:
        # Doesn't match the "<algo>_<raw|filtered>_..." naming -- keep it flat
        # rather than guessing where it belongs.
        output_dir = output_root / f'{normalized_type}_runs'
    output_dir.mkdir(parents=True, exist_ok=True)

    per_label_aligned = {}
    per_label_raw = {}
    overall_aligned = []
    overall_raw = []
    config_rows = []
    used_runs = []
    skipped_runs = []

    for run_dir in matching_runs:
        data = read_waypoint_accuracy(run_dir)
        config = read_run_config(run_dir)

        if config:
            row = {'run_name': run_dir.name}
            row.update(config)
            config_rows.append(row)

        if data is None:
            skipped_runs.append(run_dir.name)
            continue

        used_runs.append(run_dir.name)

        for result in data.get('results', []):
            label = result.get('label')
            aligned_error = result.get('aligned_error_m')
            raw_error = result.get('raw_error_m')

            if aligned_error is not None:
                per_label_aligned.setdefault(label, []).append(aligned_error)
                overall_aligned.append(aligned_error)
            if raw_error is not None:
                per_label_raw.setdefault(label, []).append(raw_error)
                overall_raw.append(raw_error)

    if not overall_aligned and not overall_raw:
        print(f'Matched {len(matching_runs)} run directories for "{normalized_type}" but found no usable waypoint results.', file=sys.stderr)
        return False

    # --- per_waypoint_stats.csv ---
    all_labels = sorted(
        set(per_label_aligned) | set(per_label_raw),
        key=lambda lbl: (len(str(lbl)), str(lbl)),
    )
    per_waypoint_path = output_dir / 'per_waypoint_stats.csv'
    with per_waypoint_path.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'label',
            'aligned_count', 'aligned_mean_m', 'aligned_std_m', 'aligned_min_m', 'aligned_max_m', 'aligned_median_m',
            'raw_count', 'raw_mean_m', 'raw_std_m', 'raw_min_m', 'raw_max_m', 'raw_median_m',
        ])
        for label in all_labels:
            a = compute_stats(per_label_aligned[label]) if label in per_label_aligned else None
            r = compute_stats(per_label_raw[label]) if label in per_label_raw else None
            writer.writerow([
                label,
                a['count'] if a else '', a['mean'] if a else '', a['std'] if a else '',
                a['min'] if a else '', a['max'] if a else '', a['median'] if a else '',
                r['count'] if r else '', r['mean'] if r else '', r['std'] if r else '',
                r['min'] if r else '', r['max'] if r else '', r['median'] if r else '',
            ])

    # --- overall_stats.csv (across all waypoints/runs combined) ---
    overall_path = output_dir / 'overall_stats.csv'
    with overall_path.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['metric', 'count', 'mean_m', 'std_m', 'min_m', 'max_m', 'median_m'])
        if overall_aligned:
            s = compute_stats(overall_aligned)
            writer.writerow(['aligned_error_m', s['count'], s['mean'], s['std'], s['min'], s['max'], s['median']])
        if overall_raw:
            s = compute_stats(overall_raw)
            writer.writerow(['raw_error_m', s['count'], s['mean'], s['std'], s['min'], s['max'], s['median']])

    # --- configs_summary.csv ---
    configs_path = output_dir / 'configs_summary.csv'
    if config_rows:
        all_keys = []
        for row in config_rows:
            for key in row:
                if key not in all_keys:
                    all_keys.append(key)
        with configs_path.open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=all_keys)
            writer.writeheader()
            for row in config_rows:
                writer.writerow(row)

    # --- source_runs.txt ---
    source_path = output_dir / 'source_runs.txt'
    with source_path.open('w') as f:
        f.write(f'run_type={normalized_type}\n')
        f.write(f'runs_dir={runs_dir}\n')
        f.write(f'matched_run_count={len(matching_runs)}\n')
        f.write(f'used_run_count={len(used_runs)}\n')
        f.write(f'skipped_run_count={len(skipped_runs)}\n')
        f.write('\nused_runs:\n')
        for name in used_runs:
            f.write(f'  {name}\n')
        if skipped_runs:
            f.write('\nskipped_runs (no metrics/waypoint_accuracy.json found):\n')
            for name in skipped_runs:
                f.write(f'  {name}\n')

    print(f'[{normalized_type}] matched {len(matching_runs)} run directories ({len(used_runs)} used, {len(skipped_runs)} skipped) -> {output_dir}')
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        'run_type',
        nargs='?',
        default=None,
        help='Run type / directory prefix, e.g. cartographer_raw_rtab_filtered_run_wp_3. '
             'If omitted, every run type found under --runs-dir is analyzed automatically.',
    )
    parser.add_argument('--runs-dir', default='evaluation/runs', help='Directory containing run subdirectories (default: evaluation/runs)')
    parser.add_argument('--output-root', default='evaluation/waypoint_evaluation', help='Root output directory (default: evaluation/waypoint_evaluation)')
    args = parser.parse_args()

    runs_dir = Path(args.runs_dir)
    if not runs_dir.is_dir():
        print(f'Runs directory does not exist: {runs_dir}', file=sys.stderr)
        sys.exit(1)

    output_root = Path(args.output_root)

    if args.run_type is not None:
        ok = process_run_type(runs_dir, args.run_type, output_root)
        sys.exit(0 if ok else 1)

    run_types = discover_run_types(runs_dir)
    if not run_types:
        print(f'No run directories (bag_*_runNNN) found under {runs_dir}', file=sys.stderr)
        sys.exit(1)

    print(f'No run type given -- analyzing all {len(run_types)} run type(s) found under {runs_dir}:')
    for rt in run_types:
        print(f'  {rt}')
    print()

    success_count = 0
    failed_types = []
    for rt in run_types:
        if process_run_type(runs_dir, rt, output_root):
            success_count += 1
        else:
            failed_types.append(rt)

    print()
    print(f'Done: {success_count}/{len(run_types)} run type(s) analyzed successfully.')
    if failed_types:
        print(f'Failed/skipped run types: {", ".join(failed_types)}', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()