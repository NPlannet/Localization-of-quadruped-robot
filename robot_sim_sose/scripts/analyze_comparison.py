#!/usr/bin/env python3
"""Compare baseline vs. parameter-scaled runs, for several parameter types.

Reads pre-aggregated stats from evaluation/waypoint_evaluation/ (as produced
by analyze_waypoint_run_type.py) and compares, for a given base_type such as:

    cartographer_raw_rtab_filtered_run_wp_1

the baseline folder:

    cartographer_raw_rtab_filtered_run_wp_1_runs

against every folder that varies ONE parameter, matching the naming used by
robot_bag_benchmark.sh:

    parameter               folder suffix     produced by
    ----------------------  ----------------  --------------------------------
    velocity                _velN.N_runs      run_velocity_matrix.sh
    static_confirmations    _scN_runs         run_static_confirmations_matrix.sh
    static_speed_threshold  _sstN.N_runs      run_static_speed_threshold_matrix.sh
                                               run_static_speed_threshold_matrix_2.sh
    moving_confirmations    _mcN_runs         run_moving_confirmations_matrix.sh

For each parameter type, results are written to (created automatically if
missing):

    evaluation/velocity_evaluation/
    evaluation/static_confirmations_evaluation/
    evaluation/static_speed_threshold_evaluation/
    evaluation/moving_confirmations_evaluation/

Each comparison produces:
    <base_type>_<parameter>_comparison.json
    <base_type>_<parameter>_comparison.png

The JSON contains, per parameter value (baseline included at its node
default): count, mean, std, min, max, median for aligned_error_m and
raw_error_m, plus the full per-waypoint breakdown. The PNG plots mean
aligned_error_m vs. the parameter value (error bars = std), with the
baseline highlighted.

Additionally, within each parameter's output folder, base types that share
the same source_bag_name but differ in slam_method (cartographer / rtabmap /
slam_toolbox) AND/OR scan_variant (raw / filtered) are grouped into a single
cross-comparison with up to 6 lines (3 algorithms x 2 scan variants), written
to a dedicated "algorithm_comparison" subfolder so summaries don't mix with
the per-base-type files:

    <parameter>_evaluation/algorithm_comparison/<bag_name>_<parameter>_algorithm_comparison.json
    <parameter>_evaluation/algorithm_comparison/<bag_name>_<parameter>_algorithm_mean_std.png   (mean ± std)
    <parameter>_evaluation/algorithm_comparison/<bag_name>_<parameter>_algorithm_median.png      (median only)

Each line is colored by algorithm and styled by scan_variant (raw=dashed
square markers, filtered=solid circle markers).

Usage:
    python3 scripts/analyze_comparison.py [<base_type>] \
        [--parameter {velocity,static_confirmations,static_speed_threshold,moving_confirmations,all}] \
        [--waypoint-eval-dir evaluation/waypoint_evaluation] \
        [--evaluation-root evaluation]

If <base_type> is omitted, every base type that has both a baseline "_runs"
folder and at least one matching variant is discovered and compared
automatically. --parameter defaults to "all" (every parameter type above).

Examples:
    # Everything: all parameter types, all base types found
    python3 scripts/analyze_comparison.py

    # Only velocity, all base types
    python3 scripts/analyze_comparison.py --parameter velocity

    # Only one base type, one parameter type
    python3 scripts/analyze_comparison.py cartographer_raw_rtab_filtered_run_wp_1 \
        --parameter static_confirmations
"""

import argparse
import csv
import json
import re
import sys
from pathlib import Path

PARAM_TYPES = {
    'velocity': {
        'suffix': 'vel',
        'default': 1.0,
        'output_dir_name': 'velocity_evaluation',
        'axis_label': 'velocity_scale',
    },
    'static_confirmations': {
        'suffix': 'sc',
        'default': 20,
        'output_dir_name': 'static_confirmations_evaluation',
        'axis_label': 'static_confirmations',
    },
    'static_speed_threshold': {
        'suffix': 'sst',
        'default': 0.06,
        'output_dir_name': 'static_speed_threshold_evaluation',
        'axis_label': 'static_speed_threshold',
    },
    'moving_confirmations': {
        'suffix': 'mc',
        'default': 4,
        'output_dir_name': 'moving_confirmations_evaluation',
        'axis_label': 'moving_confirmations',
    },
}

BASELINE_SUFFIX_RE = re.compile(r'^(?P<base>.+)_runs$')

# Known slam_method values used as the leading token of every base_type
# (base_type = "<slam_method>_<scan_variant>_<source_bag_name>", see
# robot_bag_benchmark.sh RUN_BASE). Longest first so "slam_toolbox" (which
# itself contains an underscore) is matched before a shorter accidental
# prefix could.
ALGORITHMS = ['slam_toolbox', 'cartographer', 'rtabmap']
ALGORITHM_COLORS = {
    'slam_toolbox': 'tab:green',
    'cartographer': 'tab:blue',
    'rtabmap': 'tab:red',
}


SCAN_VARIANTS = ('raw', 'filtered')
SCAN_VARIANT_LINESTYLE = {'raw': '--', 'filtered': '-'}
SCAN_VARIANT_MARKER = {'raw': 's', 'filtered': 'o'}


def split_algorithm_and_scan(base_type: str):
    """Split base_type into (algorithm, scan_variant, bag_name), or (None, None, None).

    base_type has the form "<slam_method>_<scan_variant>_<source_bag_name>"
    (see robot_bag_benchmark.sh RUN_BASE), where scan_variant is always
    exactly "raw" or "filtered".
    """
    for algo in ALGORITHMS:
        prefix = f'{algo}_'
        if not base_type.startswith(prefix):
            continue
        rest = base_type[len(prefix):]
        for scan_variant in SCAN_VARIANTS:
            sv_prefix = f'{scan_variant}_'
            if rest.startswith(sv_prefix):
                return algo, scan_variant, rest[len(sv_prefix):]
    return None, None, None


CONTAINER_NAMES = {f'{a}_{s}' for a in ALGORITHMS for s in SCAN_VARIANTS}


def iter_waypoint_run_dirs(waypoint_eval_dir: Path):
    """Yield every run-type directory under waypoint_eval_dir.

    Supports the current nested layout (waypoint_eval_dir/<algo>_<scan>/<run>_runs)
    as well as older flat layouts (waypoint_eval_dir/<run>_runs), so results
    produced before the reorganisation still work.
    """
    for entry in sorted(waypoint_eval_dir.iterdir()):
        if not entry.is_dir():
            continue
        if entry.name in CONTAINER_NAMES:
            for sub in sorted(entry.iterdir()):
                if sub.is_dir():
                    yield sub
        else:
            yield entry


def resolve_run_dir(waypoint_eval_dir: Path, base_type: str, dirname: str) -> Path:
    """Locate a run-type directory by name, preferring the nested <algo>_<scan> container."""
    algo, scan_variant, _ = split_algorithm_and_scan(base_type)
    if algo is not None:
        nested = waypoint_eval_dir / f'{algo}_{scan_variant}' / dirname
        if nested.is_dir():
            return nested
    return waypoint_eval_dir / dirname


def variant_suffix_re(suffix: str):
    return re.compile(rf'^(?P<base>.+)_{re.escape(suffix)}(?P<val>\d+(?:\.\d+)?)_runs$')


def read_overall_stats(folder: Path):
    """Return {'aligned_error_m': {...}, 'raw_error_m': {...}} or None."""
    path = folder / 'overall_stats.csv'
    if not path.is_file():
        return None
    stats = {}
    with path.open(newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            metric = row.get('metric')
            if not metric:
                continue
            try:
                stats[metric] = {
                    'count': int(row['count']),
                    'mean': float(row['mean_m']),
                    'std': float(row['std_m']),
                    'min': float(row['min_m']),
                    'max': float(row['max_m']),
                    'median': float(row['median_m']),
                }
            except (ValueError, KeyError):
                continue
    return stats or None


def read_per_waypoint_stats(folder: Path):
    path = folder / 'per_waypoint_stats.csv'
    if not path.is_file():
        return None
    rows = []
    with path.open(newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows or None


def discover_base_types_for_param(waypoint_eval_dir: Path, param_key: str):
    """Return sorted list of base types with a baseline folder + >=1 variant for this parameter."""
    suffix = PARAM_TYPES[param_key]['suffix']
    pattern = variant_suffix_re(suffix)

    variant_bases = set()
    baseline_bases = set()
    for entry in iter_waypoint_run_dirs(waypoint_eval_dir):
        m = pattern.match(entry.name)
        if m:
            variant_bases.add(m.group('base'))
            continue
        m = BASELINE_SUFFIX_RE.match(entry.name)
        if m:
            baseline_bases.add(m.group('base'))

    return sorted(variant_bases & baseline_bases)


def process_base_type_param(waypoint_eval_dir: Path, base_type: str, param_key: str, evaluation_root: Path):
    """Returns the sorted entries list on success, or None on failure."""
    param = PARAM_TYPES[param_key]
    suffix = param['suffix']
    default_value = param['default']
    pattern = variant_suffix_re(suffix)

    baseline_folder = resolve_run_dir(waypoint_eval_dir, base_type, f'{base_type}_runs')
    if not baseline_folder.is_dir():
        print(f'[{param_key}/{base_type}] baseline folder not found: {baseline_folder}', file=sys.stderr)
        return None

    variant_folders = {}
    for entry in iter_waypoint_run_dirs(waypoint_eval_dir):
        m = pattern.match(entry.name)
        if m and m.group('base') == base_type:
            variant_folders[m.group('val')] = entry

    if not variant_folders:
        print(f'[{param_key}/{base_type}] no "_{suffix}N_runs" folders found next to baseline.', file=sys.stderr)
        return None

    entries = []

    baseline_overall = read_overall_stats(baseline_folder)
    baseline_per_wp = read_per_waypoint_stats(baseline_folder)
    if baseline_overall is None:
        print(f'[{param_key}/{base_type}] baseline folder has no usable overall_stats.csv: {baseline_folder}', file=sys.stderr)
    else:
        entries.append({
            'param_value': default_value,
            'is_baseline': True,
            'source_folder': str(baseline_folder),
            'overall': baseline_overall,
            'per_waypoint': baseline_per_wp,
        })

    for val_str, folder in variant_folders.items():
        overall = read_overall_stats(folder)
        per_wp = read_per_waypoint_stats(folder)
        if overall is None:
            print(f'[{param_key}/{base_type}] skipping {folder.name}: no usable overall_stats.csv', file=sys.stderr)
            continue
        entries.append({
            'param_value': float(val_str),
            'is_baseline': False,
            'source_folder': str(folder),
            'overall': overall,
            'per_waypoint': per_wp,
        })

    if len(entries) < 2:
        print(f'[{param_key}/{base_type}] not enough usable data points (need baseline + at least one variant).', file=sys.stderr)
        return None

    entries.sort(key=lambda e: e['param_value'])

    algo, scan_variant, _ = split_algorithm_and_scan(base_type)
    if algo is not None:
        output_dir = evaluation_root / param['output_dir_name'] / f'{algo}_{scan_variant}'
    else:
        output_dir = evaluation_root / param['output_dir_name']
    output_dir.mkdir(parents=True, exist_ok=True)

    json_path = output_dir / f'{base_type}_{param_key}_comparison.json'
    png_path = output_dir / f'{base_type}_{param_key}_comparison.png'

    with json_path.open('w') as f:
        json.dump({
            'base_type': base_type,
            'parameter': param_key,
            'waypoint_eval_dir': str(waypoint_eval_dir),
            'entries': entries,
        }, f, indent=2)

    plot_comparison(base_type, param_key, param['axis_label'], entries, png_path)

    print(f'[{param_key}/{base_type}] compared {len(entries)} data point(s) -> '
          f'{json_path.relative_to(evaluation_root)}, {png_path.relative_to(evaluation_root)}')
    return entries


def plot_comparison(base_type: str, param_key: str, axis_label: str, entries, png_path: Path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    values = [e['param_value'] for e in entries]
    means = [e['overall']['aligned_error_m']['mean'] for e in entries]
    stds = [e['overall']['aligned_error_m']['std'] for e in entries]
    mins = [e['overall']['aligned_error_m']['min'] for e in entries]
    maxs = [e['overall']['aligned_error_m']['max'] for e in entries]
    medians = [e['overall']['aligned_error_m']['median'] for e in entries]
    baseline_idx = next((i for i, e in enumerate(entries) if e['is_baseline']), None)

    fig, ax = plt.subplots(figsize=(8, 5))

    ax.errorbar(
        values, means, yerr=stds, fmt='o-', color='tab:blue',
        label='mean ± std', capsize=4, linewidth=2, markersize=6, zorder=3,
    )
    ax.plot(values, medians, 's--', color='tab:orange', label='median', zorder=2)
    ax.fill_between(values, mins, maxs, color='tab:blue', alpha=0.12, label='min–max range')

    if baseline_idx is not None:
        ax.scatter(
            [values[baseline_idx]], [means[baseline_idx]],
            s=140, facecolors='none', edgecolors='red', linewidths=2,
            label=f'baseline ({axis_label}={values[baseline_idx]:g})', zorder=4,
        )

    ax.set_xlabel(axis_label)
    ax.set_ylabel('aligned position error (m)')
    ax.set_title(f'Waypoint accuracy vs. {axis_label}\n{base_type}')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)


def plot_algorithm_mean_std(bag_name: str, param_key: str, axis_label: str, series: dict, png_path: Path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 5))

    for (algo, scan_variant), entries in sorted(series.items()):
        values = [e['param_value'] for e in entries]
        means = [e['overall']['aligned_error_m']['mean'] for e in entries]
        stds = [e['overall']['aligned_error_m']['std'] for e in entries]
        color = ALGORITHM_COLORS.get(algo)
        linestyle = SCAN_VARIANT_LINESTYLE.get(scan_variant, '-')
        marker = SCAN_VARIANT_MARKER.get(scan_variant, 'o')
        ax.errorbar(
            values, means, yerr=stds, fmt=f'{marker}{linestyle}', color=color,
            label=f'{algo} ({scan_variant})', capsize=4, linewidth=2, markersize=6,
        )
        baseline_idx = next((i for i, e in enumerate(entries) if e['is_baseline']), None)
        if baseline_idx is not None:
            ax.scatter(
                [values[baseline_idx]], [means[baseline_idx]],
                s=140, facecolors='none', edgecolors=color, linewidths=2, zorder=4,
            )

    ax.set_xlabel(axis_label)
    ax.set_ylabel('aligned position error (m)')
    ax.set_title(f'Mean ± std aligned error vs. {axis_label}\n{bag_name}')
    ax.legend(title='algorithm (scan variant)')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)


def plot_algorithm_median(bag_name: str, param_key: str, axis_label: str, series: dict, png_path: Path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 5))

    for (algo, scan_variant), entries in sorted(series.items()):
        values = [e['param_value'] for e in entries]
        medians = [e['overall']['aligned_error_m']['median'] for e in entries]
        color = ALGORITHM_COLORS.get(algo)
        linestyle = SCAN_VARIANT_LINESTYLE.get(scan_variant, '-')
        marker = SCAN_VARIANT_MARKER.get(scan_variant, 'o')
        ax.plot(
            values, medians, marker=marker, linestyle=linestyle, color=color,
            label=f'{algo} ({scan_variant})', linewidth=2, markersize=6,
        )
        baseline_idx = next((i for i, e in enumerate(entries) if e['is_baseline']), None)
        if baseline_idx is not None:
            ax.scatter(
                [values[baseline_idx]], [medians[baseline_idx]],
                s=140, facecolors='none', edgecolors=color, linewidths=2, zorder=4,
            )

    ax.set_xlabel(axis_label)
    ax.set_ylabel('median aligned position error (m)')
    ax.set_title(f'Median aligned error vs. {axis_label}\n{bag_name}')
    ax.legend(title='algorithm (scan variant)')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)


def process_algorithm_comparisons(param_key: str, evaluation_root: Path, waypoint_eval_dir: Path, results_by_base_type: dict):
    """Group already-computed entries by bare bag name, across BOTH algorithm and scan_variant.

    results_by_base_type: {base_type: entries} for this param_key, as returned
    by process_base_type_param. Produces, per bag name with >=2 series
    (algorithm x scan_variant combinations), inside a dedicated
    "algorithm_comparison" subfolder:
        <bag_name>_<param_key>_algorithm_comparison.json
        <bag_name>_<param_key>_algorithm_mean_std.png
        <bag_name>_<param_key>_algorithm_median.png
    """
    param = PARAM_TYPES[param_key]
    axis_label = param['axis_label']
    output_dir = evaluation_root / param['output_dir_name'] / 'algorithm_comparison'
    median_dir = evaluation_root / param['output_dir_name'] / 'algorithm_comparison/median'
    mean_dir = evaluation_root / param['output_dir_name'] / 'algorithm_comparison/mean'

    groups = {}  # bag_name -> {(algo, scan_variant): entries}
    for base_type, entries in results_by_base_type.items():
        algo, scan_variant, bag_name = split_algorithm_and_scan(base_type)
        if algo is None:
            continue
        groups.setdefault(bag_name, {})[(algo, scan_variant)] = entries

    produced = 0
    for bag_name, series in sorted(groups.items()):
        if len(series) < 2:
            continue

        output_dir.mkdir(parents=True, exist_ok=True)
        median_dir.mkdir(parents=True, exist_ok=True)
        mean_dir.mkdir(parents=True, exist_ok=True)
        json_path = output_dir / f'{bag_name}_{param_key}_algorithm_comparison.json'
        median_png = median_dir / f'{bag_name}_{param_key}_algorithm_median.png'
        mean_std_png = mean_dir / f'{bag_name}_{param_key}_algorithm_mean_std.png'

        json_series = {
            f'{algo}_{scan_variant}': entries
            for (algo, scan_variant), entries in series.items()
        }
        with json_path.open('w') as f:
            json.dump({
                'bag_name': bag_name,
                'parameter': param_key,
                'waypoint_eval_dir': str(waypoint_eval_dir),
                'series': json_series,
            }, f, indent=2)

        plot_algorithm_mean_std(bag_name, param_key, axis_label, series, mean_std_png)
        plot_algorithm_median(bag_name, param_key, axis_label, series, median_png)

        series_str = ', '.join(f'{a}/{s}' for (a, s) in sorted(series.keys()))
        print(f'[{param_key}/{bag_name}] algorithm+scan comparison ({series_str}) '
              f'-> {param["output_dir_name"]}/algorithm_comparison/{json_path.name}')
        produced += 1

    return produced


def pooled_stats(group_stats_list):
    """Combine several {'count','mean','std','min','max','median'} summaries into one.

    Uses count-weighted mean and the exact pooled-variance formula (law of
    total variance) so mean/std are statistically correct combinations of
    group summaries. min/max are the true combined min/max. median is only
    an approximation (count-weighted average of the per-group medians),
    since individual observations are not available at this stage -- this
    is noted in the output.
    """
    total_n = sum(g['count'] for g in group_stats_list)
    if total_n == 0:
        return None
    mean = sum(g['mean'] * g['count'] for g in group_stats_list) / total_n
    var = sum(
        g['count'] * (g['std'] ** 2 + (g['mean'] - mean) ** 2)
        for g in group_stats_list
    ) / total_n
    median_approx = sum(g['median'] * g['count'] for g in group_stats_list) / total_n
    return {
        'count': total_n,
        'mean': mean,
        'std': var ** 0.5,
        'min': min(g['min'] for g in group_stats_list),
        'max': max(g['max'] for g in group_stats_list),
        'median_approx': median_approx,
        'median_note': 'count-weighted average of per-run-type medians, not an exact pooled median',
    }


def plot_overall_comparison(param_key: str, axis_label: str, param_values, series, png_path: Path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 5))

    for label, pooled_list, color in series:
        xs = [v for v, p in zip(param_values, pooled_list) if p is not None]
        means = [p['mean'] for p in pooled_list if p is not None]
        stds = [p['std'] for p in pooled_list if p is not None]
        if not xs:
            continue
        ax.errorbar(
            xs, means, yerr=stds, fmt='o-', color=color, label=label,
            capsize=4, linewidth=2, markersize=6,
        )

    ax.set_xlabel(axis_label)
    ax.set_ylabel('pooled aligned position error (m)')
    ax.set_title(f'Overall impact of {axis_label} across all algorithms & bags')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)


def process_overall_comparison(param_key: str, evaluation_root: Path, waypoint_eval_dir: Path, results_by_base_type: dict) -> int:
    """Pool ALL base types for this parameter into 3 lines: raw only, filtered only, both combined.

    Written to a dedicated "overall_comparison" subfolder inside the
    parameter's output folder:
        all_algorithms_<parameter>_overall_comparison.json
        all_algorithms_<parameter>_overall_comparison.png
    """
    param = PARAM_TYPES[param_key]
    axis_label = param['axis_label']

    by_value = {}  # param_value -> {'raw': [group_stats, ...], 'filtered': [...]}
    for base_type, entries in results_by_base_type.items():
        _, scan_variant, _ = split_algorithm_and_scan(base_type)
        if scan_variant is None:
            continue
        for e in entries:
            stats = e.get('overall', {}).get('aligned_error_m')
            if stats is None:
                continue
            by_value.setdefault(e['param_value'], {'raw': [], 'filtered': []})[scan_variant].append(stats)

    if not by_value:
        return 0

    param_values = sorted(by_value.keys())
    raw_pooled = [pooled_stats(by_value[v]['raw']) if by_value[v]['raw'] else None for v in param_values]
    filtered_pooled = [pooled_stats(by_value[v]['filtered']) if by_value[v]['filtered'] else None for v in param_values]
    combined_pooled = [
        pooled_stats(by_value[v]['raw'] + by_value[v]['filtered'])
        if (by_value[v]['raw'] or by_value[v]['filtered']) else None
        for v in param_values
    ]

    output_dir = evaluation_root / param['output_dir_name'] / 'overall_comparison'
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / f'all_algorithms_{param_key}_overall_comparison.json'
    png_path = output_dir / f'all_algorithms_{param_key}_overall_comparison.png'

    with json_path.open('w') as f:
        json.dump({
            'parameter': param_key,
            'waypoint_eval_dir': str(waypoint_eval_dir),
            'param_values': param_values,
            'raw': raw_pooled,
            'filtered': filtered_pooled,
            'combined_raw_and_filtered': combined_pooled,
        }, f, indent=2)

    series = [
        ('raw only', raw_pooled, 'tab:orange'),
        ('filtered only', filtered_pooled, 'tab:green'),
        ('raw + filtered combined', combined_pooled, 'tab:purple'),
    ]
    plot_overall_comparison(param_key, axis_label, param_values, series, png_path)

    print(f'[{param_key}/overall] pooled raw/filtered/combined comparison across all algorithms & bags '
          f'-> {param["output_dir_name"]}/overall_comparison/{json_path.name}')
    return 1


BAG_COMPARISON_COLORS = {'raw': 'tab:orange', 'filtered': 'tab:green'}


def plot_bag_comparison(algo: str, bags: dict, png_path: Path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    bag_names = sorted(bags.keys())
    x = list(range(len(bag_names)))
    width = 0.35

    fig, ax = plt.subplots(figsize=(max(8, len(bag_names) * 1.2), 5))

    for i, scan_variant in enumerate(('raw', 'filtered')):
        means, stds = [], []
        for bag in bag_names:
            stat = bags[bag].get(scan_variant)
            means.append(stat['mean'] if stat else 0.0)
            stds.append(stat['std'] if stat else 0.0)
        offset = (i - 0.5) * width
        xs = [xi + offset for xi in x]
        ax.bar(
            xs, means, width=width, yerr=stds, capsize=4,
            label=scan_variant, color=BAG_COMPARISON_COLORS[scan_variant],
        )

    ax.set_xticks(x)
    ax.set_xticklabels(bag_names, rotation=30, ha='right')
    ax.set_ylabel('aligned position error (m)')
    ax.set_title(f'{algo}: baseline accuracy across bags (raw vs. filtered)')
    ax.legend(title='scan variant')
    ax.grid(True, axis='y', alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)


def process_bag_comparison(evaluation_root: Path, waypoint_eval_dir: Path) -> int:
    """For each algorithm, compare baseline (default-parameter) accuracy across all bags, raw vs. filtered.

    Uses only the baseline "_runs" folders (no parameter suffix). Written to
    a top-level output folder (not tied to any one swept parameter):
        evaluation/algorithm_bag_comparison/<algorithm>_bag_comparison.json
        evaluation/algorithm_bag_comparison/<algorithm>_bag_comparison.png
    """
    output_dir = evaluation_root / 'algorithm_bag_comparison'

    variant_patterns = [variant_suffix_re(p['suffix']) for p in PARAM_TYPES.values()]

    data = {}  # algo -> {bag_name: {'raw': stats, 'filtered': stats}}
    for entry in iter_waypoint_run_dirs(waypoint_eval_dir):
        if any(p.match(entry.name) for p in variant_patterns):
            continue  # parameter-sweep variant (e.g. _vel1.5_runs), not a plain baseline bag
        m = BASELINE_SUFFIX_RE.match(entry.name)
        if not m:
            continue
        base_type = m.group('base')
        algo, scan_variant, bag_name = split_algorithm_and_scan(base_type)
        if algo is None:
            continue
        overall = read_overall_stats(entry)
        if overall is None or 'aligned_error_m' not in overall:
            continue
        data.setdefault(algo, {}).setdefault(bag_name, {})[scan_variant] = overall['aligned_error_m']

    produced = 0
    for algo, bags in sorted(data.items()):
        if not bags:
            continue
        output_dir.mkdir(parents=True, exist_ok=True)
        json_path = output_dir / f'{algo}_bag_comparison.json'
        png_path = output_dir / f'{algo}_bag_comparison.png'

        with json_path.open('w') as f:
            json.dump({
                'algorithm': algo,
                'waypoint_eval_dir': str(waypoint_eval_dir),
                'bags': bags,
            }, f, indent=2)

        plot_bag_comparison(algo, bags, png_path)

        print(f'[bag_comparison/{algo}] {len(bags)} bag(s) -> algorithm_bag_comparison/{json_path.name}')
        produced += 1

    return produced


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        'base_type',
        nargs='?',
        default=None,
        help='Base run type, e.g. cartographer_raw_rtab_filtered_run_wp_1. '
             'If omitted, every base type with a baseline + at least one variant is compared automatically.',
    )
    parser.add_argument(
        '--parameter',
        choices=list(PARAM_TYPES.keys()) + ['all'],
        default='all',
        help='Which parameter to compare (default: all).',
    )
    parser.add_argument('--waypoint-eval-dir', default='evaluation/waypoint_evaluation', help='Directory produced by analyze_waypoint_run_type.py (default: evaluation/waypoint_evaluation)')
    parser.add_argument('--evaluation-root', default='evaluation', help='Root evaluation directory that holds the per-parameter output folders (default: evaluation)')
    args = parser.parse_args()

    waypoint_eval_dir = Path(args.waypoint_eval_dir)
    if not waypoint_eval_dir.is_dir():
        print(f'Waypoint evaluation directory does not exist: {waypoint_eval_dir}', file=sys.stderr)
        sys.exit(1)

    evaluation_root = Path(args.evaluation_root)

    param_keys = list(PARAM_TYPES.keys()) if args.parameter == 'all' else [args.parameter]

    total_attempted = 0
    total_success = 0
    total_algorithm_comparisons = 0
    failed = []

    for param_key in param_keys:
        if args.base_type is not None:
            base_types = [args.base_type]
        else:
            base_types = discover_base_types_for_param(waypoint_eval_dir, param_key)
            if not base_types:
                print(f'[{param_key}] no base type with baseline + variants found, skipping.')
                continue
            print(f'[{param_key}] found {len(base_types)} base type(s): {", ".join(base_types)}')

        results_by_base_type = {}
        for base_type in base_types:
            total_attempted += 1
            entries = process_base_type_param(waypoint_eval_dir, base_type, param_key, evaluation_root)
            if entries is not None:
                total_success += 1
                results_by_base_type[base_type] = entries
            else:
                failed.append(f'{param_key}/{base_type}')

        if len(results_by_base_type) >= 2:
            total_algorithm_comparisons += process_algorithm_comparisons(
                param_key, evaluation_root, waypoint_eval_dir, results_by_base_type,
            )

        if results_by_base_type:
            process_overall_comparison(param_key, evaluation_root, waypoint_eval_dir, results_by_base_type)

    # Baseline (default-parameter) accuracy per algorithm across all bags, raw vs.
    # filtered -- independent of any parameter sweep, so it's generated once.
    total_bag_comparisons = process_bag_comparison(evaluation_root, waypoint_eval_dir)

    print()
    print(f'Done: {total_success}/{total_attempted} comparison(s) generated successfully, '
          f'{total_algorithm_comparisons} cross-algorithm comparison(s), '
          f'{total_bag_comparisons} per-algorithm bag comparison(s).')
    if failed:
        print(f'Failed/skipped: {", ".join(failed)}', file=sys.stderr)
    if total_success == 0 and total_bag_comparisons == 0:
        sys.exit(1)


if __name__ == '__main__':
    main()