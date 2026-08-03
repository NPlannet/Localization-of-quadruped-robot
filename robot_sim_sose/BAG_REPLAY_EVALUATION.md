# W1 Bag Replay Evaluation

The `w1/w1_nogt` bag can be replayed with either SLAM Toolbox or Cartographer.
Surveyed waypoint positions and their bag timestamps are stored in
`w1/w1_nogt_waypoints.json`.

The replay launches normalize `/scan` to `/scan_normalized` before either
mapper sees it. The recorded LD19 messages vary between 497 and 507 beams and
sometimes contain overlapping per-beam timestamps; normalization produces a
fixed 503-beam scan with consistent timing. Filtered runs then process
`/scan_normalized` into `/scan_filtered`.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  xgo_driver_bridge dynamic_scan_filter xgo_description
source install/setup.bash
```

## Run the four accuracy variants

For repeatable metric runs, RViz, the camera viewer, camera republishing, and
Nav2 are disabled. They can be enabled separately for visual inspection.

```bash
# SLAM Toolbox, filtered scan
ros2 launch xgo_description bag_replay.launch.py \
  use_dynamic_filter:=true use_rviz:=true use_image_view:=true \
  republish_camera:=true use_nav2:=false

# SLAM Toolbox, raw scan
ros2 launch xgo_description bag_replay.launch.py \
  use_dynamic_filter:=false use_rviz:=true use_image_view:=true \
  republish_camera:=true use_nav2:=false

# Cartographer, filtered scan
ros2 launch xgo_description bag_replay_cartographer.launch.py \
  use_dynamic_filter:=true use_rviz:=true use_image_view:=true \
  republish_camera:=true

# Cartographer, raw scan
ros2 launch xgo_description bag_replay_cartographer.launch.py \
  use_dynamic_filter:=false use_rviz:=true use_image_view:=true \
  republish_camera:=true
```

Results are written without overwriting the other variants:

```text
metrics/w1_slam_toolbox_filtered_waypoint_eval.json
metrics/w1_slam_toolbox_raw_waypoint_eval.json
metrics/w1_cartographer_filtered_waypoint_eval.json
metrics/w1_cartographer_raw_waypoint_eval.json
```

For visualization, RViz uses fixed frame `map`, occupancy grid `/map`, and raw
normalized LiDAR `/scan_normalized`. The filtered scan remains available on
`/scan_filtered` and can be selected in RViz when inspecting the filter.

## Accuracy calculation

For each waypoint, the evaluator takes 11 TF samples across a centered
one-second window. It uses the median X/Y position and circular-mean heading,
which avoids scoring a single noisy transform while the robot is settling.

The SLAM `map` frame and surveyed room frame can have different origins and
headings. By default, the evaluator therefore fits one rigid 2D SE(2)
transformation (rotation and translation, never scale) across the successful
waypoints. The JSON includes both raw errors and aligned errors so the frame
correction remains visible.

Reported position statistics include MAE, RMSE, median, p95, maximum, and
standard deviation. Repeated waypoint labels additionally report position and
heading drift between the first and last visits. Estimated heading is always
recorded; heading error is calculated when a waypoint contains `yaw_deg` or
`yaw`.

Useful overrides:

```bash
# Do not align the SLAM map frame to the surveyed frame.
ros2 launch xgo_description bag_replay.launch.py \
  evaluation_alignment_mode:=none

# Change the pose-settling window and sample count.
ros2 launch xgo_description bag_replay.launch.py \
  evaluation_settling_window_sec:=1.5 \
  evaluation_window_sample_count:=15 \
  evaluation_min_window_samples:=7
```

## Loop closure settings

The bag replay files for SLAM Toolbox are:

```text
src/xgo_description/config/slam_toolbox_lifelong.yaml
src/xgo_description/config/slam_toolbox_lifelong_raw.yaml
```

Both set `do_loop_closing: true` for comparison with Cartographer. Change that
parameter to `false` in both files for a no-loop-closure SLAM Toolbox ablation.

Cartographer's optimization frequency is configured in:

```text
src/xgo_description/config/cartographer_robot_2d.lua
```

with `POSE_GRAPH.optimize_every_n_nodes = 35`. This is not an on/off loop
closure switch; Cartographer's constraint builder performs loop-closure search
using its pose-graph parameters.
