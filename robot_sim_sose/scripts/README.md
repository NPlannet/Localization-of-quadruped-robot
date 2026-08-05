# Project Scripts

Run these commands from the `robot_sim_sose/` workspace root unless noted.

| Script | Purpose |
| --- | --- |
| `sync_robot_runtime.sh` | Copy the minimal runtime tree to a robot with rsync |
| `robot_build_workspace.sh` | Resolve/build the ROS workspace inside the robot container |
| `robot_stack.sh` | Small commands for sensor bringup, checks, mapping, and map saving |
| `record_robot_run.sh` | Attach to an existing bringup and record MCAP, CPU/RAM, and battery data |
| `robot_bag_benchmark.sh` | Replay original bag inputs through a fresh headless SLAM pipeline on the Pi |
| `run_benchmark_matrix.py` | Resume an all-algorithm, raw/filtered, repeated benchmark matrix for one bag |
| `monitor_robot_resources.py` | Sample process/system CPU and memory and write a summary |
| `monitor_robot_battery.py` | Record `/battery_state` samples and summarize observed discharge |
| `plot_localization_accuracy.py` | Create the presentation accuracy comparison PNG |
| `record_xgo_orientation.py` | Diagnostic logger for raw XGO orientation reads |
| `waypoint_marker_node` | Timestamp Lichtblick waypoint-button presses with the robot ROS clock |
| `slam_failure_monitor.py` | Diagnostic monitor for common SLAM/TF failures |
| `explore.py` | Legacy interactive Nav2 exploration prototype |
| `image_saver.py` | Legacy raw-camera image capture prototype |

`record_robot_run.sh` attaches to an existing sensor, teleoperation, or mapper
bringup. Stopping the recorder does not stop bringup or send motion commands.
