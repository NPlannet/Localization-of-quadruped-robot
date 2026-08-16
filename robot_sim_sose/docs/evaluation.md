# Recording and Localization Evaluation

The evaluation compares SLAM Toolbox, Cartographer, and RTAB-Map using the same
recorded sensor data, with the dynamic LiDAR filter enabled and disabled.

## Data layout

```text
evaluation/
├── datasets/<dataset>/   Bag sidecars and externally stored inputs
├── runs/<run>/           New recordings and temporary evaluations
└── results/<experiment>/ Retained metrics, maps, plots, and summaries
```

Raw bags and RTAB-Map databases are Git-ignored and must be archived
separately.

## Record a robot dataset

Start robot bringup without SLAM so every algorithm can later process the same
original inputs:

```bash
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:=true slam_method:=none \
  start_dynamic_filter:=true start_foxglove:=true
```

In another container shell:

```bash
bash scripts/record_robot_run.sh RUN_NAME
```

The recorder stores the bag, ROS inventory, CPU/RAM samples, and battery
samples under `evaluation/runs/<run_name>/`. Press `Ctrl+C` once to finalize it.
The running sensor stack is not stopped.

Important recorded inputs include `/scan`, `/imu/data`, `/xgo/applied_vel`,
`/tf_static`, camera topics, and `/ground_truth/waypoint`. Derived topics are
recorded for inspection but are excluded from clean benchmark replay.

## Mark ground-truth waypoints

In Lichtblick, create a Publish button:

- Topic: `/ground_truth/waypoint_trigger`
- Type: `std_msgs/msg/Empty`
- Message: `{}`

Press it once while the robot is stationary on each surveyed point. The robot
node writes the current ROS timestamp to `/ground_truth/waypoint` and the bag
records it.

After recording, create `waypoints.json` beside the bag:

```json
{
  "topic": "/ground_truth/waypoint",
  "frame": "map",
  "marks": [
    {"stamp_ns": 1785415036091702448, "index": 0, "label": "1", "x": 0.0, "y": 0.0}
  ]
}
```

The timestamp comes from the bag; `label`, `x`, and `y` come from the surveyed
waypoint layout.

## Run one clean benchmark

Run on the Raspberry Pi to measure performance on the target CPU:

The arguments are the SLAM method (`slam_toolbox`, `cartographer`, or
`rtabmap`), scan variant (`raw` or `filtered`), bag path, and an optional
waypoint file.

Example:

```bash
bash scripts/robot_bag_benchmark.sh \
  cartographer raw \
  evaluation/runs/my_run/bag \
  evaluation/runs/my_run/waypoints.json
```

The benchmark replays only original inputs:

```text
/scan  /imu/data  /xgo/applied_vel  /tf_static
/camera/image_raw/compressed   # RTAB-Map RGB only
```

It rebuilds `/odom`, runs the current filter for a filtered trial, starts a
fresh mapper, measures resources, and evaluates waypoints. Recorded `/odom`,
`/tf`, `/scan_filtered`, `/map`, and old mapper outputs are not replayed.

Legacy bags with overlapping per-beam LiDAR timestamps, including W1, require:

```bash
NORMALIZE_SCAN=true bash scripts/robot_bag_benchmark.sh \
  cartographer raw /path/to/bag /path/to/waypoints.json
```

## Run the complete comparison

Copy and edit the matrix template:

```bash
cp evaluation/templates/benchmark_matrix.example.yaml evaluation/benchmark.yaml
python3 scripts/run_benchmark_matrix.py evaluation/benchmark.yaml --dry-run
python3 scripts/run_benchmark_matrix.py evaluation/benchmark.yaml
```

Running the last command again resumes unfinished work. Check progress with:

```bash
python3 scripts/run_benchmark_matrix.py evaluation/benchmark.yaml --status
```

The standard design is three algorithms × raw/filtered × five repetitions.
The runner randomizes condition order, applies cooldown/temperature checks,
saves configuration snapshots, and writes per-run and aggregate CSV summaries.

Do not edit a matrix configuration after the experiment state has been
created. Use a new experiment name for changed settings.

## Accuracy metric

At every marked timestamp, the evaluator samples `map -> base_link` over a
short stationary window and uses a robust central pose. Because every SLAM map
has an arbitrary origin, the default evaluation fits one rigid 2D SE(2)
rotation and translation to the surveyed points. It never fits scale.

Report:

- mean absolute error (MAE);
- RMSE and median error;
- maximum error;
- successful waypoint count; and
- drift between repeated visits to the same waypoint.

Do not use p95 as a headline metric for a small waypoint set. Compare all
methods on the same common observations and retain individual-run values, not
only their average.

The reconstructed odometry integrates applied velocity and uses IMU
orientation. It is an approximation, not measured displacement or ground
truth, and should be stated as a limitation.

## Fair-comparison rules

- Use the same bag, waypoints, playback rate, and algorithm configuration.
- Start each run with an independent map or RTAB-Map database.
- Keep camera use fixed when comparing RTAB-Map variants.
- Disable Foxglove/RViz for primary CPU measurements.
- Use at least three runs; five are used by the default matrix.
- Save the Git commit and configuration snapshot with every experiment.

For visual desktop inspection, the three replay launches are:

```bash
ros2 launch xgo_description bag_replay.launch.py
ros2 launch xgo_description bag_replay_cartographer.launch.py
ros2 launch xgo_description bag_replay_rtabmap.launch.py use_camera:=true
```
