# ROS2 / Nav2 Demo Submission Summary

This folder contains the ROS2/Nav2 TurtleBot3 project files plus the local run evidence produced in WSL Ubuntu 22.04.

## Final Evidence Directory

`demo/real_nav2_gazebo_20260606_220310/`

## Acceptance Mapping

- Nodes: `demo/real_nav2_gazebo_20260606_220310/nodes_after_goal_manual.txt`
- Topics: `demo/real_nav2_gazebo_20260606_220310/topics_after_goal_manual.txt`
- Actions: `demo/real_nav2_gazebo_20260606_220310/actions_after_goal_manual.txt`
- TF: recorded in `demo/real_nav2_gazebo_20260606_220310/real_nav2_bag_manual/`
- Map: `demo/real_nav2_gazebo_20260606_220310/map_turtlebot3_world.yaml` and `.pgm`
- Navigation run: `navigate_to_pose_goal_result_tail.txt`
- Local recording: `demo/real_nav2_gazebo_20260606_220310/real_nav2_bag_manual/`, summarized by `real_nav2_bag_manual_info.txt`
- Video evidence: `docs/videos/real_gazebo_nav2_run.mp4`

## Result

- ROS2 Humble environment found in WSL Ubuntu 22.04.
- Gazebo Classic published real TurtleBot3 `/odom` and `/scan`.
- AMCL published `/amcl_pose` after `/initialpose`.
- `/navigate_to_pose` finished with `SUCCEEDED`.
- `/plan`, `/cmd_vel`, `/odom`, `/scan`, `/tf`, `/tf_static`, `/map`, `/amcl_pose`, and `/particle_cloud` were recorded.
- ROS bag captured 6753 messages over about 69.6 seconds.

## Note

The earlier archive in `demo/nav2_official_tb3_20260604_115048/` is kept as historical evidence from the first attempt. The final evidence above is the real Gazebo run: `/odom` and `/scan` came from `turtlebot3_gazebo`, and the video was rendered from the recorded rosbag rather than hand-keyframed.
