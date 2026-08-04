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
camera topics, and diagnostics while independently sampling CPU and RAM. Press
`Ctrl+C` once to finalize the MCAP. It does not stop the existing bringup.
Output is written to `evaluation/runs/<run_name>/`.

For a controlled live mapper run that starts and owns the entire bringup:

```bash
bash scripts/robot_evaluation_run.sh slam_toolbox filtered st_filtered_run1
bash scripts/robot_evaluation_run.sh cartographer raw cart_raw_run1
bash scripts/robot_evaluation_run.sh rtabmap filtered rtab_filtered_run1
```

Run at least three trials per condition, use the same route and speed, start
with an empty/independent map or database, and avoid running RViz/Foxglove when
measuring feasibility on the Raspberry Pi unless visualization overhead is
explicitly part of the test.
