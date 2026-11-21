# navigation_pkg

Path planner for the Unity mobile robot in MobileRobotEnv.

`planner.py` builds an occupancy grid from the obstacle point cloud. Cell size is 0.25 m. Occupied cells are inflated by 0.40 m. The search is A* over 8 neighbours. A diagonal step is not taken if it cuts a blocked corner. The heuristic is Euclidean distance. An empty point cloud gives a straight line from start to goal.

`navigation_node.py` reads the robot pose from the TF transform `map` -> `Robot` and publishes `nav_msgs/Path` on `/path`.

The test node sends `(-3, 3)`, then `(7, -10)`, then `(6, -6)`. A goal is reached at a distance of 0.25 m or less.

`results/run_log.txt`:

- `(-3, 3)`: 13 points, goal reached
- `(7, -10)`: 61 points, goal reached
- `(6, -6)`: 122 points, goal reached

Figures: `unity.png`, `rviz.png`, `rviz_window.png`.

## Run

Ubuntu, ROS 2 Humble, `MobileRobotSim.x86_64`, and ROS-TCP-Endpoint branch `main-ros2`.

```bash
export ROS_LOCALHOST_ONLY=1
cd RoboticsLesson_ws
colcon build
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint
ros2 run navigation_pkg navigation_node
ros2 run navigation_pkg test
rviz2 -d rviz/RoboticsLesson_viz.rviz
```

```bash
python3 check_planner.py
```
