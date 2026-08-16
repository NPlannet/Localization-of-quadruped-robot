# Robot Runtime Configuration

These are the source configurations used by the live robot bringup and the
headless bag benchmark. Raw and filtered comparisons use the same SLAM
configuration; only the scan topic changes between `/scan` and
`/scan_filtered`.

| Component | Configuration | Selected by |
| --- | --- | --- |
| SLAM Toolbox | `slam_toolbox_robot.yaml` | `slam_method:=slam_toolbox` |
| Cartographer | `cartographer_robot_2d.lua` | `slam_method:=cartographer` |
| RTAB-Map | Static parameters in `../launch/rtabmap_robot.launch.py` | `slam_method:=rtabmap` |
| Dynamic LiDAR filter | `dynamic_scan_filter_robot.yaml` | `start_dynamic_filter:=true` |
| Camera | `camera_ros_robot.yaml` | `start_camera:=true` |
| Foxglove/Lichtblick bridge | `foxglove_bridge_robot.yaml` | `start_foxglove:=true` |
| Bag playback QoS | `bag_benchmark_play_qos.yaml` | Headless bag benchmark |
| Bag recording QoS | `bag_record_qos.yaml` | Robot recording script |

The main live entry point is
`xgo_driver_bridge/launch/robot_sensor_bringup.launch.py`. Its launch arguments
select the algorithm, scan topic, and optional sensors. The headless evaluator
uses `xgo_driver_bridge/launch/headless_bag_benchmark.launch.py` and defaults to
the same SLAM and filter configurations.

RTAB-Map currently keeps its static settings in its launch file because several
values are launch-time choices, including camera use, database path, scan QoS,
detection rate, and loop-closure threshold.

For reproducibility, `scripts/run_benchmark_matrix.py` copies these files into
each experiment's `configuration_snapshot/` directory and records their SHA-256
hashes in `experiment_metadata.yaml`. Every individual run also receives a
`run_config.txt` with the selected algorithm and runtime arguments.

After changing a source configuration, rebuild and source the workspace before
starting a new run. Do not change configuration files during an experiment
matrix.
