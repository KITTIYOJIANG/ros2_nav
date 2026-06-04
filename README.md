# ROS2 Nav2 TurtleBot3 Mobile Robot Navigation MVP

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange)
![Nav2](https://img.shields.io/badge/Nav2-Stack-green)
![TurtleBot3](https://img.shields.io/badge/TurtleBot3-Burger-yellow)
![SLAM](https://img.shields.io/badge/SLAM-Toolbox-purple)
![Navigation](https://img.shields.io/badge/Navigation-Autonomous-red)

**ROS2 Humble | Gazebo Classic | Nav2 | TurtleBot3 | SLAM | Autonomous Navigation**

## Verified Run Evidence

The completed local verification artifacts are in:

`demo/nav2_official_tb3_20260604_115048/`

Start with `SUBMISSION_SUMMARY.md` in this project root, or open
`demo/nav2_official_tb3_20260604_115048/nav2_runtime_evidence.png` for a compact visual summary.

## Project Overview

This project implements a complete mobile robot navigation pipeline using ROS2 Humble, Gazebo Classic, and the Nav2 framework on a TurtleBot3 Burger model. The system performs simultaneous localization and mapping (SLAM) to build an occupancy grid map, then uses that map with AMCL (Adaptive Monte Carlo Localization) for autonomous navigation. This MVP demonstrates the full perception-planning-control loop required for real-world mobile robotics applications and serves as an educational sandbox for experimenting with path planning, obstacle avoidance, and localization algorithms.

## Architecture

```
+-------------+    +-----------+    +------+    +-----+    +------+    +-----------+    +---------+
|             |    |           |    |      |    |     |    |      |    |           |    |         |
| Gazebo Sim  |--->|  Sensors  |--->| SLAM |--->| Map |--->| AMCL |--->|   Nav2    |--->| cmd_vel |
|             |    |           |    |      |    |     |    |      |    |           |    |         |
+-------------+    +-----------+    +------+    +-----+    +------+    +-----------+    +---------+
      |                 |                                                         |
      |                 |                                                         |
      v                 v                                                         v
+-------------+    +-----------+                                          +---------------+
|  World/Env  |    |  /scan    |                                          |  /cmd_vel     |
|  (ground    |    |  /odom    |                                          |  Twist msgs   |
|   truth)    |    |  /imu     |                                          |  to motors    |
+-------------+    +-----------+                                          +---------------+
```

## Project Structure

```
ros2-nav2-turtlebot3/
├── config/
│   ├── nav2_params.yaml          # Nav2 planner/controller/behavior parameters
│   ├── mapper_params_online_async.yaml  # SLAM Toolbox config
│   └── turtlebot3_burger.yaml    # TurtleBot3 model config
├── launch/
│   ├── gazebo_world.launch.py    # Launches Gazebo + spawns TurtleBot3
│   ├── slam.launch.py            # Launches SLAM Toolbox + RViz
│   ├── navigation.launch.py      # Launches Nav2 stack with map
│   └── full_pipeline.launch.py   # All-in-one launch file
├── maps/
│   ├── my_map.pgm                # Saved occupancy grid image
│   └── my_map.yaml               # Map metadata (resolution, origin)
├── worlds/
│   └── maze_world.world          # Custom Gazebo world
├── rviz/
│   └── nav2_view.rviz            # RViz configuration
├── docs/
│   ├── FAQ.md                    # Frequently Asked Questions
│   └── images/                   # Screenshots and diagrams
├── README.md
└── LICENSE
```

## Key Features

- **SLAM**: Real-time mapping using `slam_toolbox` with online asynchronous mode
- **Localization**: AMCL particle filter for robust pose estimation on a known map
- **Path Planning**: Global planner (Smac Hybrid-A*) + local controller (Regulated Pure Pursuit)
- **Obstacle Avoidance**: Dynamic obstacle detection via LiDAR with costmap layers
- **RViz Visualization**: Live map, pose, path, and sensor data display
- **Custom Worlds**: Pre-configured maze environment for navigation testing
- **Modular Launch Files**: Run SLAM and navigation independently or as a full pipeline

## Quick Start

### Prerequisites

- Ubuntu 22.04 (Jammy)
- ROS2 Humble ([installation guide](https://docs.ros.org/en/humble/Installation.html))
- Gazebo Classic (comes with `ros-humble-ros-gz` or install separately)
- TurtleBot3 packages

### 1. Install Dependencies

```bash
# Install ROS2 Humble (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Install TurtleBot3 packages
sudo apt update
sudo apt install ros-humble-turtlebot3-bringup \
                 ros-humble-turtlebot3-description \
                 ros-humble-turtlebot3-gazebo \
                 ros-humble-turtlebot3-teleop

# Install Nav2 and SLAM Toolbox
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox

# Source ROS2
source /opt/ros/humble/setup.bash
```

### 2. Launch Gazebo + Spawn TurtleBot3

```bash
export TURTLEBOT3_MODEL=burger

# Launch empty world with TurtleBot3
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# OR launch a custom maze world
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

### 3. Run SLAM (Mapping)

Open a **new terminal** and run:

```bash
export TURTLEBOT3_MODEL=burger

# Launch SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=./config/mapper_params_online_async.yaml

# Drive the robot around to build the map
ros2 run turtlebot3_teleop teleop_keyboard
```

### 4. Save the Map

Once satisfied with the map quality:

```bash
# In a new terminal, navigate to your maps directory
cd ros2-nav2-turtlebot3/maps

# Save the map
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 5. Run Autonomous Navigation

First, kill the SLAM nodes. Then:

```bash
export TURTLEBOT3_MODEL=burger

# Launch Nav2 with the saved map
ros2 launch nav2_bringup bringup_launch.py \
  map:=./maps/my_map.yaml \
  params_file:=./config/nav2_params.yaml \
  use_sim_time:=true

# In a new terminal, launch RViz
ros2 run rviz2 rviz2 -d ./rviz/nav2_view.rviz
```

### 6. Send Navigation Goals

- In RViz, click the **"Nav2 Goal"** button (top toolbar)
- Click and drag on the map to set a goal pose (position + orientation)
- The robot will plan a global path and execute it autonomously

## Demo

> **Video Demo:** [Link to demonstration video] (placeholder)

A recorded walkthrough showing SLAM map building and autonomous navigation in the Gazebo maze environment.

## Experiment Results

| Metric | Value |
|--------|-------|
| Map Resolution | |
| SLAM Time | |
| Path Length (avg) | |
| Navigation Success Rate | |
| Localization Error (RMSE) | |
| Planner Latency (avg) | |
| Obstacle Avoidance Rate | |

## References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Gazebo Classic](https://classic.gazebosim.org/)

## License

This project is licensed under the MIT License. See [LICENSE](./LICENSE) for details.
