# Recording and Localization Evaluation

The evaluation layout is documented in [`../evaluation/README.md`](../evaluation/README.md).
The W1 replay launches default to:

```text
evaluation/datasets/w1/bag/
evaluation/datasets/w1/waypoints.json
evaluation/results/w1/
```

## Build

Inside the development container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  xgo_driver_bridge dynamic_scan_filter xgo_description
source install/setup.bash
```

## Replay W1

Run each algorithm with the raw scan and then the filtered scan. The launch
files select distinct default metric/database names, so variants do not
overwrite one another.

```bash
# SLAM Toolbox
ros2 launch xgo_description bag_replay.launch.py use_dynamic_filter:=false
ros2 launch xgo_description bag_replay.launch.py use_dynamic_filter:=true

# Cartographer
ros2 launch xgo_description bag_replay_cartographer.launch.py use_dynamic_filter:=false
ros2 launch xgo_description bag_replay_cartographer.launch.py use_dynamic_filter:=true

# RTAB-Map with RGB loop recognition
ros2 launch xgo_description bag_replay_rtabmap.launch.py \
  use_dynamic_filter:=false use_camera:=true
ros2 launch xgo_description bag_replay_rtabmap.launch.py \
  use_dynamic_filter:=true use_camera:=true
```

For timing/resource measurements, disable visualization. For visual inspection,
enable it explicitly. Relevant arguments include `use_rviz`, `use_image_view`,
and, for RTAB-Map, `use_rtabmap_viz`.

Override any input or output when testing another bag:

```bash
ros2 launch xgo_description bag_replay.launch.py \
  bag_path:=/absolute/path/to/bag \
  waypoints_file:=/absolute/path/to/waypoints.json \
  evaluation_output_path:=/absolute/path/to/result.json
```

## What the waypoint metric measures

Each waypoint sidecar entry contains a bag timestamp and a surveyed global
position. Around that timestamp, the evaluator samples the SLAM `map ->
base_link` transform 11 times over a centered one-second settling window. It
uses the median X/Y and circular-mean heading to reduce single-sample jitter.

The arbitrary SLAM map origin is not ground truth. The default `se2` alignment
therefore fits one rigid 2D rotation and translation from all estimated points
to the surveyed frame. It never changes scale. The JSON retains raw poses and
errors as well as aligned errors, successful/failed observations, the fitted
transform, summary statistics, and revisit drift.

For the presentation, compare mean absolute error (MAE), RMSE, median error,
maximum error, successful observations, and revisit drift. Do not present the
old p95 field for this 12-waypoint sample; one order statistic is not very
informative at that size. Compare all methods on the same common waypoint set.

The W1 bag does not contain wheel/leg odometry. During replay,
`xgo_offline_odom_node` reconstructs `odom -> base_link` by integrating the
recorded applied velocity and using recorded IMU orientation for heading. This
is intentionally an approximation and should be stated as a limitation when
generalizing replay accuracy to live robot operation.

Useful evaluator overrides:

```bash
# Inspect errors without aligning the coordinate systems.
ros2 launch xgo_description bag_replay.launch.py evaluation_alignment_mode:=none

# Widen the stable-pose window.
ros2 launch xgo_description bag_replay.launch.py \
  evaluation_settling_window_sec:=1.5 \
  evaluation_window_sample_count:=15 \
  evaluation_min_window_samples:=7
```

## Loop-closure comparison

SLAM Toolbox replay parameters are in:

```text
src/xgo_description/config/slam_toolbox_lifelong.yaml
src/xgo_description/config/slam_toolbox_lifelong_raw.yaml
```

Set `do_loop_closing` to the same value in both files for a raw-versus-filtered
comparison. It is currently enabled.

Cartographer replay parameters are in:

```text
src/xgo_description/config/cartographer_replay_2d.lua
```

`POSE_GRAPH.optimize_every_n_nodes` controls how frequently the pose graph is
optimized; it is not a direct equivalent of SLAM Toolbox's loop-closure switch.
Cartographer constraint-builder parameters govern candidate loop closures.

RTAB-Map uses geometric scan constraints and can additionally use RGB
appearance for loop recognition. Its launch arguments include `detection_rate`
and `visual_loop_threshold`. Keep camera usage fixed when comparing its scan
variants.

## Plot the existing runs

```bash
python3 scripts/plot_localization_accuracy.py
python3 scripts/plot_localization_accuracy.py --metric rmse
python3 scripts/plot_localization_accuracy.py --metric max
```

The default output is
`evaluation/results/w1/plots/localization_accuracy_mae.png`. Supported metrics
are `mae`, `rmse`, `median`, `max`, and `revisit`.

## Record a physical-robot dataset without SLAM

Start the normal robot bringup with no mapper:

```bash
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:=true \
  start_dynamic_filter:=true \
  start_foxglove:=true \
  slam_method:=none
```

In another shell in the same container:

```bash
bash scripts/record_robot_run.sh sensor_only_run1
```

This records `/scan`, `/scan_filtered`, `/cmd_vel`, `/imu/data`, `/odom`, TF,
`/battery_state`, camera topics, and diagnostics while independently sampling
CPU, RAM, and battery percentage. Press `Ctrl+C` once to finalize the MCAP. It
does not stop the existing bringup. Output is written to
`evaluation/runs/<run_name>/`:

```text
<run_name>/
├── bag/                 MCAP sensor and algorithm topics
├── resources/
│   ├── system.csv       Whole-system CPU, RAM, temperature, and frequency
│   ├── processes.csv    Per-component CPU and resident memory
│   └── summary.json
├── battery/
│   ├── samples.csv      Timestamped BatteryState values
│   └── summary.json     Start/end percentage and observed discharge rate
├── bag_info.txt
├── ros_nodes.txt
├── ros_topics.txt
└── run_config.txt
```

The XGO SDK currently supplies battery percentage but not measured voltage or
current. Consequently, this is a coarse state-of-charge comparison rather than
an energy measurement in watt-hours. Use equal-duration trials with the same
starting charge, route, gait, payload, camera/visualization settings, and idle
warm-up. Short runs may show no percentage change because the controller value
is quantized; longer repeated runs are more useful.

For a controlled live mapper run that starts and owns the entire bringup:

```bash
bash scripts/robot_evaluation_run.sh slam_toolbox filtered
bash scripts/robot_evaluation_run.sh cartographer raw
bash scripts/robot_evaluation_run.sh rtabmap filtered
```

Run directories are allocated automatically and never overwritten, for example
`live_slam_toolbox_filtered_run001` followed by
`live_slam_toolbox_filtered_run002`. Add an optional tag when separating rooms
or protocols:

```bash
bash scripts/robot_evaluation_run.sh cartographer raw room2
```

Run at least three trials per condition, use the same route and speed, start
with an empty/independent map or database, and avoid running RViz/Foxglove when
measuring feasibility on the Raspberry Pi unless visualization overhead is
explicitly part of the test.


## IMPORTANT : Benchmark recorded bags on the Raspberry Pi

The robot benchmark uses only `xgo_driver_bridge` and the installed mapping
packages; it does not build or install `xgo_description`, Gazebo, or RViz.

```bash
# SLAM Toolbox, recomputing odometry and filtering from original inputs
bash scripts/robot_bag_benchmark.sh \
  slam_toolbox filtered \
  evaluation/datasets/my_run/bag \
  evaluation/datasets/my_run/waypoints.json

# Cartographer, raw scan
bash scripts/robot_bag_benchmark.sh \
  cartographer raw \
  evaluation/datasets/my_run/bag \
  evaluation/datasets/my_run/waypoints.json

# RTAB-Map with RGB appearance recognition
bash scripts/robot_bag_benchmark.sh \
  rtabmap filtered \
  evaluation/datasets/my_run/bag \
  evaluation/datasets/my_run/waypoints.json
```

Omit the final waypoint argument for compute-only runs. RTAB-Map uses the
camera by default; set `USE_CAMERA=false` to measure its LiDAR-only variant.
Legacy W1 scans require the normalizer:

```bash
NORMALIZE_SCAN=true bash scripts/robot_bag_benchmark.sh \
  cartographer raw evaluation/datasets/w1/bag \
  evaluation/datasets/w1/waypoints.json
```

Benchmark directories follow the same collision-safe convention, such as
`bag_cartographer_raw_run001`. Set `RUN_TAG=room2` to obtain names such as
`bag_cartographer_raw_room2_run001`.

The player uses an explicit input allowlist:

```text
/scan
/imu/data
/xgo/applied_vel
/tf_static
/camera/image_raw/compressed    # RTAB-Map RGB only
```

Recorded `/odom`, `/tf`, `/scan_filtered`, `/map`, and mapper diagnostics are
never replayed. `/odom` and `odom -> base_link` are rebuilt from the recorded
applied velocity and IMU, and a selected filtered run always executes the
current dynamic filter on the raw `/scan`. Keep the original bag unchanged;
there is no need to create a second bag with topics deleted.

New benchmark results are written to `evaluation/runs/<run_name>/`, including
the input bag inventory, exact configuration, launch log, component/system
resource samples, RTAB-Map database where applicable, and waypoint metrics
when a sidecar was supplied.

### Foxglove during CPU trials

Headless operation is the default and should be used for the primary compute
comparison. To inspect a separately labelled visual run:

```bash
RUN_TAG=visual START_FOXGLOVE=true bash scripts/robot_bag_benchmark.sh \
  slam_toolbox filtered /path/to/bag
```

Connect Lichtblick to `ws://<robot-ip>:8766`. Enabling the bridge for every run
is a valid end-to-end system comparison, but it is not a pure mapper comparison:
network serialization depends on map size, diagnostics, and camera traffic, so
the overhead need not be identical across algorithms. Do not mix headless and
Foxglove-enabled trials in one aggregate.

The benchmark does not replay the historical battery topic because that would
describe the original walk, not current Pi consumption. It also does not start
the live XGO bridge, since that would create conflicting IMU and odometry
publishers. True replay-energy measurement requires an external Pi power meter;
CPU, RAM, temperature, and frequency are measured directly by the script.
