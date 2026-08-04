#!/usr/bin/env python3
"""Record ROS BatteryState samples and write a compact run summary."""

from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import statistics
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState


RUNNING = True


def stop_requested(_signum: int, _frame: Any) -> None:
    global RUNNING
    RUNNING = False


def finite_or_none(value: float) -> float | None:
    value = float(value)
    return value if math.isfinite(value) else None


def rounded(value: float | None, digits: int = 4) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return round(value, digits)


def iso_now() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def linear_rate_per_second(samples: list[tuple[float, float]]) -> float | None:
    """Return least-squares percentage-point slope against elapsed seconds."""
    if len(samples) < 2:
        return None
    mean_time = statistics.fmean(sample[0] for sample in samples)
    mean_value = statistics.fmean(sample[1] for sample in samples)
    denominator = sum((sample[0] - mean_time) ** 2 for sample in samples)
    if denominator <= 0.0:
        return None
    numerator = sum(
        (sample[0] - mean_time) * (sample[1] - mean_value)
        for sample in samples
    )
    return numerator / denominator


class BatteryMonitor(Node):
    def __init__(self, topic: str, output_dir: Path, label: str) -> None:
        super().__init__('robot_battery_monitor')
        self.topic = topic
        self.output_dir = output_dir
        self.label = label
        self.started_at = iso_now()
        self.started_monotonic = time.monotonic()
        self.samples: list[dict[str, float | int | bool | None]] = []
        self.closed = False

        output_dir.mkdir(parents=True, exist_ok=True)
        self.samples_path = output_dir / 'samples.csv'
        self.summary_path = output_dir / 'summary.json'
        self.samples_file = self.samples_path.open(
            'w', newline='', encoding='utf-8'
        )
        self.fields = [
            'wall_timestamp',
            'elapsed_s',
            'ros_stamp_ns',
            'percentage',
            'voltage_v',
            'current_a',
            'charge_ah',
            'capacity_ah',
            'temperature_c',
            'present',
            'power_supply_status',
        ]
        self.writer = csv.DictWriter(self.samples_file, fieldnames=self.fields)
        self.writer.writeheader()
        self.samples_file.flush()

        self.subscription = self.create_subscription(
            BatteryState,
            topic,
            self.battery_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Recording battery samples from {topic} to {output_dir}'
        )

    def battery_callback(self, msg: BatteryState) -> None:
        elapsed = time.monotonic() - self.started_monotonic
        ros_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        if ros_stamp_ns <= 0:
            ros_stamp_ns = self.get_clock().now().nanoseconds

        percentage_fraction = finite_or_none(msg.percentage)
        percentage = (
            percentage_fraction * 100.0
            if percentage_fraction is not None and percentage_fraction >= 0.0
            else None
        )
        sample = {
            'wall_timestamp': iso_now(),
            'elapsed_s': rounded(elapsed),
            'ros_stamp_ns': ros_stamp_ns,
            'percentage': rounded(percentage),
            'voltage_v': rounded(finite_or_none(msg.voltage)),
            'current_a': rounded(finite_or_none(msg.current)),
            'charge_ah': rounded(finite_or_none(msg.charge)),
            'capacity_ah': rounded(finite_or_none(msg.capacity)),
            'temperature_c': rounded(finite_or_none(msg.temperature)),
            'present': bool(msg.present),
            'power_supply_status': int(msg.power_supply_status),
        }
        self.samples.append(sample)
        self.writer.writerow(sample)
        self.samples_file.flush()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.samples_file.close()

        percentage_samples = [
            (float(sample['elapsed_s']), float(sample['percentage']))
            for sample in self.samples
            if sample['elapsed_s'] is not None
            and sample['percentage'] is not None
        ]
        percentages = [sample[1] for sample in percentage_samples]
        endpoint_count = min(5, len(percentages))
        start_percentage = (
            statistics.median(percentages[:endpoint_count])
            if endpoint_count
            else None
        )
        end_percentage = (
            statistics.median(percentages[-endpoint_count:])
            if endpoint_count
            else None
        )
        drop = (
            start_percentage - end_percentage
            if start_percentage is not None and end_percentage is not None
            else None
        )
        slope = linear_rate_per_second(percentage_samples)
        signed_rate_per_hour = slope * 3600.0 if slope is not None else None
        discharge_rate = (
            -signed_rate_per_hour
            if signed_rate_per_hour is not None and signed_rate_per_hour < 0.0
            else 0.0 if signed_rate_per_hour is not None else None
        )
        duration = time.monotonic() - self.started_monotonic
        projected_full_runtime = (
            100.0 / discharge_rate
            if discharge_rate is not None
            and discharge_rate > 0.0
            and duration >= 900.0
            and drop is not None
            and drop >= 2.0
            else None
        )

        summary = {
            'label': self.label,
            'topic': self.topic,
            'started_at': self.started_at,
            'ended_at': iso_now(),
            'monitor_duration_s': rounded(duration),
            'sample_count': len(self.samples),
            'valid_percentage_sample_count': len(percentages),
            'start_percentage': rounded(start_percentage),
            'end_percentage': rounded(end_percentage),
            'observed_drop_percentage_points': rounded(drop),
            'minimum_percentage': rounded(min(percentages)) if percentages else None,
            'maximum_percentage': rounded(max(percentages)) if percentages else None,
            'regression_rate_percentage_points_per_hour': rounded(
                signed_rate_per_hour
            ),
            'discharge_rate_percentage_points_per_hour': rounded(discharge_rate),
            'projected_full_runtime_hours': rounded(projected_full_runtime),
            'projection_available': projected_full_runtime is not None,
            'definitions': {
                'endpoint_values': (
                    'Median of up to the first/last five valid samples.'
                ),
                'regression_rate': (
                    'Least-squares slope across all percentage samples; a '
                    'negative value indicates discharge.'
                ),
                'projected_full_runtime': (
                    'Reported only for runs of at least fifteen minutes with '
                    'at least two percentage points of observed discharge.'
                ),
            },
            'limitations': (
                'The current XGO SDK bridge reports controller battery '
                'percentage only. Voltage, current, energy, and state of '
                'charge accuracy are not available, so short-run battery '
                'comparisons are coarse.'
            ),
            'files': {'samples': self.samples_path.name},
        }
        self.summary_path.write_text(
            json.dumps(summary, indent=2) + '\n', encoding='utf-8'
        )
        self.get_logger().info(
            f'Battery measurements saved to {self.output_dir}'
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--topic', default='/battery_state')
    parser.add_argument('--label', default='')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, stop_requested)
    signal.signal(signal.SIGTERM, stop_requested)
    rclpy.init()
    node = BatteryMonitor(args.topic, args.output_dir, args.label)
    try:
        while RUNNING and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
