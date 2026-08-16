# Nav2 Wavefront Frontier Explorer

This ROS 2 node finds reachable free cells bordering unknown space in `/map`
and sends one goal at a time to Nav2's `/navigate_to_pose` action. It supports
binary occupancy grids from SLAM Toolbox and RTAB-Map as well as Cartographer's
probability-valued occupancy grids.

The implementation is based on wavefront frontier detection described by
[Quin et al.](https://arxiv.org/abs/1806.03581).

## Build and run

```bash
colcon build --symlink-install --packages-select nav2_wfd
source install/setup.bash
ros2 run nav2_wfd explore --ros-args -p use_sim_time:=true
```

Nav2, a live `/map`, and a connected `map -> base_link` TF tree must be
available. The node waits while Nav2 and the mapper start. Do not run another
explorer at the same time because both processes would send competing goals.

Useful parameters:

- `free_threshold` (default `49`): highest occupancy value treated as free
- `occupied_threshold` (default `65`): occupancy value treated as occupied
- `minimum_frontier_size` (default `5`): minimum connected frontier cells
- `minimum_goal_distance` (default `0.5` m): ignore targets already at the robot
- `empty_frontier_retries` (default `10`): map-update retries before finishing
