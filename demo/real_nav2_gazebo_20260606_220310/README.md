# Real Gazebo + Nav2 Evidence

This directory contains the final real simulation run captured in WSL Ubuntu 22.04 on June 6, 2026.

## Result

- Gazebo Classic ran `turtlebot3_gazebo/turtlebot3_world.launch.py`.
- Nav2 ran `nav2_bringup/bringup_launch.py` with the TurtleBot3 world map.
- AMCL accepted the initial pose and published `/amcl_pose`.
- `/navigate_to_pose` finished with `SUCCEEDED`.
- The recorded rosbag contains 6753 messages over 69.6 seconds.

## Key Evidence

| Check | File |
|---|---|
| Gazebo odometry | `odom_once_gazebo.txt` |
| Gazebo laser scan | `scan_once_gazebo.txt` |
| AMCL pose after initial pose | `amcl_pose_once_manual.txt` |
| Nav2 action result | `navigate_to_pose_goal_result_tail.txt` |
| Bag topic counts | `real_nav2_bag_manual_info.txt` |
| Bag samples | `bag_sample_summary.txt` |
| Nodes / topics / actions | `nodes_after_goal_manual.txt`, `topics_after_goal_manual.txt`, `actions_after_goal_manual.txt` |
| Rosbag | `real_nav2_bag_manual/` |

## Bag Summary

```text
/odom: 1906 messages
/scan: 324 messages
/cmd_vel: 331 messages
/plan: 15 messages
/tf: 3489 messages
/map: 1 message
```

The robot moved from approximately `x=-1.99, y=-0.50` to `x=0.37, y=0.47` during the run.
