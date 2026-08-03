#!/usr/bin/env python3

import argparse
import csv
import math
import statistics
import time
from pathlib import Path

from xgolib import XGO


def shortest_angle_delta_degrees(current: float, previous: float) -> float:
    return math.degrees(
        math.atan2(
            math.sin(math.radians(current - previous)),
            math.cos(math.radians(current - previous)),
        )
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description='Record raw XGO roll, pitch, and yaw register readings.'
    )
    parser.add_argument('--port', default='/dev/ttyAMA0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--duration', type=float, default=40.0)
    parser.add_argument('--rate', type=float, default=5.0)
    parser.add_argument('--baseline-duration', type=float, default=10.0)
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('metrics/xgo_orientation_recording.csv'),
    )
    return parser.parse_args()


def main():
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    dog = XGO(
        port=args.port,
        baud=args.baud,
        version='xgomini',
        verbose=False,
    )
    firmware = dog.read_firmware()
    print(f'Connected to XGO firmware {firmware} on {args.port}.', flush=True)
    print(
        f'Recording {args.duration:.1f} seconds to {args.output}. '
        f'Keep the robot stationary for the first {args.baseline_duration:.1f} '
        'seconds.',
        flush=True,
    )

    samples = []
    previous_raw_yaw = None
    unwrapped_yaw = None
    yaw_reference = None
    start_monotonic = time.monotonic()
    next_sample_time = start_monotonic
    turn_announced = False

    fieldnames = [
        'elapsed_sec',
        'unix_time_sec',
        'roll_deg',
        'pitch_deg',
        'yaw_raw_deg',
        'yaw_relative_unwrapped_deg',
        'read_cycle_sec',
    ]

    try:
        with args.output.open('w', newline='', encoding='utf-8') as output_file:
            writer = csv.DictWriter(output_file, fieldnames=fieldnames)
            writer.writeheader()

            while True:
                elapsed = time.monotonic() - start_monotonic
                if elapsed >= args.duration:
                    break
                if elapsed >= args.baseline_duration and not turn_announced:
                    print(
                        'TURN NOW: rotate the robot, pause, and rotate it back.',
                        flush=True,
                    )
                    turn_announced = True

                read_start = time.monotonic()
                roll = float(dog.read_roll())
                pitch = float(dog.read_pitch())
                yaw_raw = float(dog.read_yaw())
                read_cycle_sec = time.monotonic() - read_start

                if previous_raw_yaw is None:
                    unwrapped_yaw = yaw_raw
                    yaw_reference = yaw_raw
                else:
                    unwrapped_yaw += shortest_angle_delta_degrees(
                        yaw_raw,
                        previous_raw_yaw,
                    )
                previous_raw_yaw = yaw_raw
                yaw_relative = unwrapped_yaw - yaw_reference

                row = {
                    'elapsed_sec': elapsed,
                    'unix_time_sec': time.time(),
                    'roll_deg': roll,
                    'pitch_deg': pitch,
                    'yaw_raw_deg': yaw_raw,
                    'yaw_relative_unwrapped_deg': yaw_relative,
                    'read_cycle_sec': read_cycle_sec,
                }
                writer.writerow(row)
                output_file.flush()
                samples.append(row)

                if turn_announced:
                    print(
                        f't={elapsed:5.1f}s '
                        f'roll={roll:+8.2f} pitch={pitch:+8.2f} '
                        f'yaw_rel={yaw_relative:+9.2f} deg',
                        flush=True,
                    )

                next_sample_time += 1.0 / max(args.rate, 0.1)
                time.sleep(max(0.0, next_sample_time - time.monotonic()))
    finally:
        dog.ser.close()

    if not samples:
        raise RuntimeError('No orientation samples were recorded.')

    baseline = [
        sample
        for sample in samples
        if float(sample['elapsed_sec']) < args.baseline_duration
    ]
    baseline_yaw = [
        float(sample['yaw_relative_unwrapped_deg'])
        for sample in baseline
    ]
    read_times = [float(sample['read_cycle_sec']) for sample in samples]
    yaw_values = [
        float(sample['yaw_relative_unwrapped_deg'])
        for sample in samples
    ]

    print(f'Recorded {len(samples)} samples.', flush=True)
    if len(baseline_yaw) >= 2:
        print(
            'Stationary yaw baseline: '
            f'range={max(baseline_yaw) - min(baseline_yaw):.3f} deg, '
            f'std={statistics.stdev(baseline_yaw):.3f} deg.',
            flush=True,
        )
    print(
        'Full-run relative yaw: '
        f'min={min(yaw_values):.2f} deg, max={max(yaw_values):.2f} deg.',
        flush=True,
    )
    print(
        f'Mean three-register read cycle: {statistics.mean(read_times):.3f}s.',
        flush=True,
    )
    print(f'Saved: {args.output.resolve()}', flush=True)


if __name__ == '__main__':
    main()
