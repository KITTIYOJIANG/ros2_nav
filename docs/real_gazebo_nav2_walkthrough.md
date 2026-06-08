# Real Gazebo + Nav2 Walkthrough

This note documents the real simulation path used for the final evidence run on June 6, 2026. The important change from the earlier archive is that `/odom` and `/scan` came from `turtlebot3_gazebo`, not from a fake simulator.

## Mental Model

Nav2 is not the robot. Nav2 is the navigation brain:

- Gazebo simulates the world, robot body, wheels, laser scan, odometry, and physics.
- TurtleBot3 publishes robot state such as `/odom`, `/scan`, `/joint_states`, `/tf`, and `/tf_static`.
- AMCL estimates `map -> odom` after you give `/initialpose`.
- Nav2 plans from the current pose to the goal and publishes `/cmd_vel`.
- Gazebo receives `/cmd_vel`, moves the robot, and publishes the next `/odom` and `/scan`.

That loop is the core navigation pipeline:

```text
Gazebo robot/sensors -> /odom + /scan -> AMCL/Nav2 -> /plan + /cmd_vel -> Gazebo robot motion
```

## Reproduce The Run

In WSL Ubuntu 22.04:

```bash
cd /home/zexu/ros2-nav2-turtlebot3
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=52
```

From this repository, run:

```bash
/mnt/f/Work/Portfolio/ros2_nav/src/turtlebot3_nav2_demo/scripts/run_real_gazebo_nav2.sh \
  /home/zexu/ros2-nav2-turtlebot3/real_nav2_gazebo_repeat
```

The script does five things:

1. Starts `turtlebot3_gazebo turtlebot3_world.launch.py`.
2. Waits for real Gazebo `/odom` and `/scan`.
3. Starts `nav2_bringup bringup_launch.py` with the TurtleBot3 world map.
4. Publishes `/initialpose` so AMCL can create `map -> odom`.
5. Sends `/navigate_to_pose` and records a rosbag.

## How To Read The Evidence

The final evidence directory is:

```text
demo/real_nav2_gazebo_20260606_220310/
```

Useful files:

- `odom_once_gazebo.txt`: proves Gazebo published robot odometry before Nav2 was involved.
- `scan_once_gazebo.txt`: proves the simulated laser scan was active.
- `amcl_pose_once_manual.txt`: proves AMCL localized the robot after `/initialpose`.
- `navigate_to_pose_goal_result_tail.txt`: shows the Nav2 action feedback and `SUCCEEDED` result.
- `real_nav2_bag_manual_info.txt`: summarizes the recorded rosbag.
- `bag_sample_summary.txt`: extracts start/end odometry, first command velocity, first plan, and first scan statistics from the bag.

Key result:

```text
/navigate_to_pose: SUCCEEDED
rosbag: 6753 messages over 69.6 seconds
/odom: 1906 messages, x/y moved from about (-1.99, -0.50) to (0.37, 0.47)
/scan: 324 messages
/cmd_vel: 331 messages
/plan: 15 messages
```

## Common Pitfalls

- If AMCL says `Please set the initial pose`, publish `/initialpose` before sending the goal.
- If `/spawn_entity` times out in a one-shot launch, start Gazebo first and wait for sensor topics before starting Nav2.
- If a topic echo after the goal is empty, check the rosbag. Topics such as `/cmd_vel` may stop publishing once the robot has already reached the goal.
- WSLg root-window recording can produce a black screen. The committed video is therefore rendered from the real recorded rosbag instead of a hand-made animation.
