# Project Scripts

Run these commands from the `robot_sim_sose/` workspace root unless noted.

| Script | Purpose |
| --- | --- |
| `sync_robot_runtime.sh` | Copy the minimal runtime tree to a robot with rsync |
| `robot_build_workspace.sh` | Resolve/build the ROS workspace inside the robot container |
| `robot_stack.sh` | Small commands for sensor bringup, checks, mapping, and map saving |
| `record_robot_run.sh` | Attach to an existing bringup and record MCAP, CPU/RAM, and battery data |
| `robot_evaluation_run.sh` | Start a complete mapper evaluation with resource and battery recording |
| `robot_bag_benchmark.sh` | Replay original bag inputs through a fresh headless SLAM pipeline on the Pi |
| `monitor_robot_resources.py` | Sample process/system CPU and memory and write a summary |
| `monitor_robot_battery.py` | Record `/battery_state` samples and summarize observed discharge |
| `plot_localization_accuracy.py` | Create the presentation accuracy comparison PNG |
| `record_xgo_orientation.py` | Diagnostic logger for raw XGO orientation reads |
| `slam_failure_monitor.py` | Diagnostic monitor for common SLAM/TF failures |
| `explore.py` | Legacy interactive Nav2 exploration prototype |
| `image_saver.py` | Legacy raw-camera image capture prototype |

`record_robot_run.sh` is the safer choice when the sensors, teleoperation, and
mapper are already running: stopping the recorder does not stop bringup or send
motion commands. `robot_evaluation_run.sh` owns the complete evaluation process
and is intended for controlled, repeatable trials.
