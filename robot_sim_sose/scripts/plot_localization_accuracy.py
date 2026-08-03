#!/usr/bin/env python3
"""Create a presentation-ready localization comparison chart from JSON runs."""

import argparse
import glob
import json
import math
import os
import statistics
from pathlib import Path

# Avoid warnings on machines where ~/.config is mounted read-only.
os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-cache')
os.environ.setdefault('XDG_CACHE_HOME', '/tmp/xdg-cache')

import matplotlib.pyplot as plt


ALGORITHM_ORDER = ['SLAM Toolbox', 'Cartographer', 'RTAB-Map RGB']
VARIANT_ORDER = ['Raw', 'Filtered']
COLORS = {'Raw': '#8C96A3', 'Filtered': '#D62728'}


def parse_args():
    project_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--metrics-dir',
        type=Path,
        default=project_dir / 'metrics',
        help='Directory containing the waypoint evaluation JSON files.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=project_dir / 'metrics' / 'localization_accuracy_mae.png',
        help='PNG file to create.',
    )
    parser.add_argument(
        '--metric',
        choices=['mae', 'rmse', 'median', 'max', 'revisit'],
        default='mae',
        help='Metric shown in the graph. p95 is intentionally not supported.',
    )
    parser.add_argument('--dpi', type=int, default=600)
    return parser.parse_args()


def classify_file(path):
    name = path.name.lower()
    if 'rtabmap_rgb' in name:
        algorithm = 'RTAB-Map RGB'
    elif 'cartographer' in name:
        algorithm = 'Cartographer'
    elif 'slam_toolbox' in name:
        algorithm = 'SLAM Toolbox'
    else:
        return None

    if 'filtered' in name:
        variant = 'Filtered'
    elif 'raw' in name:
        variant = 'Raw'
    else:
        return None
    return algorithm, variant


def load_runs(metrics_dir):
    runs = []
    # Existing run names use both "speed1_run1" and "speed_1_run_1".
    pattern = str(metrics_dir / '*.json')
    for filename in sorted(glob.glob(pattern)):
        path = Path(filename)
        if 'speed' not in path.stem.lower() or 'run' not in path.stem.lower():
            continue
        classification = classify_file(path)
        if classification is None:
            continue
        with path.open(encoding='utf-8') as stream:
            data = json.load(stream)
        if not data.get('results'):
            continue
        runs.append(
            {
                'path': path,
                'algorithm': classification[0],
                'variant': classification[1],
                'data': data,
            }
        )
    return runs


def common_waypoint_indices(runs):
    index_sets = [
        {int(result['index']) for result in run['data']['results']}
        for run in runs
    ]
    return sorted(set.intersection(*index_sets))


def aligned_errors_by_index(results):
    """Fit one rigid 2D transform from estimates to surveyed positions."""
    mean_est_x = statistics.mean(result['est_x'] for result in results)
    mean_est_y = statistics.mean(result['est_y'] for result in results)
    mean_gt_x = statistics.mean(result['gt_x'] for result in results)
    mean_gt_y = statistics.mean(result['gt_y'] for result in results)

    dot = 0.0
    cross = 0.0
    for result in results:
        est_x = result['est_x'] - mean_est_x
        est_y = result['est_y'] - mean_est_y
        gt_x = result['gt_x'] - mean_gt_x
        gt_y = result['gt_y'] - mean_gt_y
        dot += est_x * gt_x + est_y * gt_y
        cross += est_x * gt_y - est_y * gt_x

    yaw = math.atan2(cross, dot)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    translation_x = mean_gt_x - (cosine * mean_est_x - sine * mean_est_y)
    translation_y = mean_gt_y - (sine * mean_est_x + cosine * mean_est_y)

    errors = {}
    for result in results:
        aligned_x = (
            translation_x
            + cosine * result['est_x']
            - sine * result['est_y']
        )
        aligned_y = (
            translation_y
            + sine * result['est_x']
            + cosine * result['est_y']
        )
        errors[int(result['index'])] = math.hypot(
            aligned_x - result['gt_x'],
            aligned_y - result['gt_y'],
        )
    return errors


def run_metric(run, indices, metric):
    selected_results = [
        result
        for result in run['data']['results']
        if int(result['index']) in indices
    ]
    errors = list(aligned_errors_by_index(selected_results).values())

    if metric == 'mae':
        return statistics.mean(errors)
    if metric == 'rmse':
        return math.sqrt(statistics.mean(error * error for error in errors))
    if metric == 'median':
        return statistics.median(errors)
    if metric == 'max':
        return max(errors)

    revisit_drifts = [
        revisit['position_drift_m']
        for revisit in run['data'].get('revisits', [])
        if int(revisit['first_index']) in indices
        and int(revisit['last_index']) in indices
    ]
    if not revisit_drifts:
        raise ValueError(f"No common revisit measurements in {run['path']}")
    return statistics.mean(revisit_drifts)


def summarize(runs, indices, metric):
    summaries = {}
    for algorithm in ALGORITHM_ORDER:
        for variant in VARIANT_ORDER:
            values = [
                run_metric(run, indices, metric)
                for run in runs
                if run['algorithm'] == algorithm and run['variant'] == variant
            ]
            if not values:
                continue
            summaries[(algorithm, variant)] = {
                'values': values,
                'mean': statistics.mean(values),
                'std': statistics.stdev(values) if len(values) > 1 else 0.0,
                'count': len(values),
            }
    return summaries


def metric_title(metric):
    return {
        'mae': 'Mean Absolute Position Error',
        'rmse': 'Position RMSE',
        'median': 'Median Position Error',
        'max': 'Maximum Position Error per Run',
        'revisit': 'Mean Revisit Drift',
    }[metric]


def create_plot(summaries, indices, metric, output, dpi):
    plt.style.use('seaborn-v0_8-whitegrid')
    figure, axis = plt.subplots(figsize=(13.333, 7.5))

    maximum_bar_height = max(
        summary['mean'] for summary in summaries.values()
    ) * 100.0
    annotation_gap = max(1.35, maximum_bar_height * 0.10)

    group_positions = list(range(len(ALGORITHM_ORDER)))
    width = 0.34
    offsets = {'Raw': -width / 2, 'Filtered': width / 2}

    for variant in VARIANT_ORDER:
        positions = []
        heights = []
        for position, algorithm in zip(group_positions, ALGORITHM_ORDER):
            summary = summaries[(algorithm, variant)]
            positions.append(position + offsets[variant])
            heights.append(summary['mean'] * 100.0)

        bars = axis.bar(
            positions,
            heights,
            width,
            color=COLORS[variant],
            edgecolor='white',
            linewidth=1.2,
            label=variant,
            zorder=3,
        )
        for bar, height in zip(bars, heights):
            axis.text(
                bar.get_x() + bar.get_width() / 2,
                height + 0.18,
                f'{height:.2f} cm',
                ha='center',
                va='bottom',
                fontsize=11,
                fontweight='semibold',
            )

    # Show the relative filter change above each algorithm pair.
    for position, algorithm in zip(group_positions, ALGORITHM_ORDER):
        raw = summaries[(algorithm, 'Raw')]['mean']
        filtered = summaries[(algorithm, 'Filtered')]['mean']
        improvement = (raw - filtered) / raw * 100.0
        highest = max(raw, filtered) * 100.0
        axis.text(
            position,
            highest + annotation_gap,
            f'Filter: {improvement:.1f}% lower',
            ha='center',
            color='#B3181B',
            fontsize=11,
            fontweight='bold',
        )

    axis.set_title(
        f'{metric_title(metric)} — Real-Robot Recording',
        fontsize=22,
        fontweight='bold',
        pad=24,
    )
    axis.set_ylabel('Position error [cm]', fontsize=15)
    axis.set_xticks(group_positions, ALGORITHM_ORDER, fontsize=14)
    axis.tick_params(axis='y', labelsize=12)
    axis.legend(frameon=False, fontsize=13, ncol=2, loc='upper left')
    axis.spines[['top', 'right']].set_visible(False)
    axis.grid(axis='x', visible=False)
    axis.set_axisbelow(True)

    headroom = max(3.5, maximum_bar_height * 0.22)
    axis.set_ylim(0.0, maximum_bar_height + headroom)

    indices_text = ', '.join(str(index) for index in indices)
    figure.text(
        0.5,
        0.015,
        'Bars show the mean across runs. '
        f'SE(2)-aligned comparison on common waypoint indices: {indices_text}.',
        ha='center',
        fontsize=10.5,
        color='#4A5560',
    )
    figure.tight_layout(rect=(0.03, 0.06, 0.98, 0.97))

    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=dpi, bbox_inches='tight', facecolor='white')
    plt.close(figure)


def main():
    args = parse_args()
    runs = load_runs(args.metrics_dir)
    if not runs:
        raise SystemExit(f'No speed1 run JSON files found in {args.metrics_dir}')

    indices = common_waypoint_indices(runs)
    if not indices:
        raise SystemExit('The evaluation files have no common waypoint indices.')

    summaries = summarize(runs, indices, args.metric)
    expected = {
        (algorithm, variant)
        for algorithm in ALGORITHM_ORDER
        for variant in VARIANT_ORDER
    }
    missing = expected - set(summaries)
    if missing:
        raise SystemExit(f'Missing run groups: {sorted(missing)}')

    create_plot(summaries, indices, args.metric, args.output, args.dpi)
    print(f'Created {args.output.resolve()}')
    for algorithm in ALGORITHM_ORDER:
        raw = summaries[(algorithm, 'Raw')]
        filtered = summaries[(algorithm, 'Filtered')]
        print(
            f'{algorithm}: raw={raw["mean"] * 100:.2f} cm (n={raw["count"]}), '
            f'filtered={filtered["mean"] * 100:.2f} cm '
            f'(n={filtered["count"]})'
        )


if __name__ == '__main__':
    main()
