# Bag Replay Evaluation

The `w1 (1)/w1_nogt` bag can now be replayed directly with either SLAM Toolbox or Cartographer.

## What Changed

- The bag replay launches now default to `w1 (1)/w1_nogt`.
- SLAM Toolbox replay uses lifelong mapping by default.
- Replay starts an offline odometry bridge because this bag does not record `/odom` or dynamic `/tf`.
- Replay can score localization against `w1 (1)/w1_nogt_waypoints.json`.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select xgo_driver_bridge dynamic_scan_filter xgo_description
source install/setup.bash
```

## SLAM Toolbox

```bash
ros2 launch xgo_description bag_replay.launch.py
```

Results are written to:

```text
metrics/w1_slam_toolbox_waypoint_eval.json
```

Disable the dynamic filter for an A/B run:

```bash
ros2 launch xgo_description bag_replay.launch.py use_dynamic_filter:=false
```

## Cartographer

```bash
ros2 launch xgo_description bag_replay_cartographer.launch.py
```

Results are written to:

```text
metrics/w1_cartographer_waypoint_eval.json
```

Disable the dynamic filter for an A/B run:

```bash
ros2 launch xgo_description bag_replay_cartographer.launch.py use_dynamic_filter:=false
```

## Useful Overrides

Use a different output file:

```bash
ros2 launch xgo_description bag_replay.launch.py \
  evaluation_output_path:=/tmp/slam_eval.json
```

Disable waypoint scoring:

```bash
ros2 launch xgo_description bag_replay.launch.py use_waypoint_evaluator:=false
```
