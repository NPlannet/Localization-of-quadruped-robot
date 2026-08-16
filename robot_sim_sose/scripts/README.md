# Project Scripts

Run scripts from the `robot_sim_sose/` workspace root.

| Script | Purpose |
| --- | --- |
| `demo_simulation.sh` | Start the maintained Gazebo, filter, SLAM Toolbox, and Nav2 demo |
| `sync_robot_runtime.sh` | Copy the robot runtime to a Raspberry Pi |
| `robot_build_workspace.sh` | Build the robot ROS packages |
| `robot_stack.sh` | Sensor checks, individual nodes, motion test, and map saving |
| `record_robot_run.sh` | Record MCAP, system resources, and battery data |
| `robot_bag_benchmark.sh` | Run one clean headless bag evaluation |
| `run_benchmark_matrix.py` | Run and resume the complete repeated comparison |
| `monitor_robot_resources.py` | Record CPU, memory, temperature, and process use |
| `monitor_robot_battery.py` | Record and summarize `/battery_state` |
| `plot_localization_accuracy.py` | Generate accuracy comparison PNGs |
| `record_xgo_orientation.py` | Diagnose raw XGO roll, pitch, and yaw readings |

`record_robot_run.sh` attaches to an existing bringup. Stopping the recorder
does not stop the robot nodes or send motion commands.
