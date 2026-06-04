# ROS2 / Nav2 Demo Submission Summary

This folder contains the ROS2/Nav2 TurtleBot3 project files plus the local run evidence produced in WSL Ubuntu 22.04.

## Final Evidence Directory

`demo/nav2_official_tb3_20260604_115048/`

## Acceptance Mapping

- Nodes: `demo/nav2_official_tb3_20260604_115048/nodes.txt`
- Topics: `demo/nav2_official_tb3_20260604_115048/topics.txt`
- Actions: `demo/nav2_official_tb3_20260604_115048/actions.txt`
- Lifecycle states: `demo/nav2_official_tb3_20260604_115048/lifecycle_after_initialpose.txt`
- TF: `demo/nav2_official_tb3_20260604_115048/runtime_tf_tree.png`, `tf_dynamic_after_goal.txt`, `tf_static_once.txt`
- Map: `demo/nav2_official_tb3_20260604_115048/map_turtlebot3_world.yaml`, `.pgm`, `.png`, and `map_info.txt`
- Navigation run: `navigate_to_pose_goal.log`, `plan_once_after_goal.txt`, `local_plan_once_after_goal.txt`, `cmd_vel_once_after_goal.txt`
- Local recording: `demo/nav2_official_tb3_20260604_115048/nav2_goal_bag/`, summarized by `bag_info.txt`
- Screenshot/evidence image: `demo/nav2_official_tb3_20260604_115048/nav2_runtime_evidence.png`

## Result

- ROS2 Humble environment found in WSL Ubuntu 22.04.
- Nav2 lifecycle nodes reached `active [3]` after publishing the initial pose.
- `/navigate_to_pose` goal was accepted.
- `/plan`, `/local_plan`, `/cmd_vel`, `/cmd_vel_nav`, `/odom`, `/scan`, `/tf`, `/tf_static`, `/map`, and `/amcl_pose` were recorded.
- ROS bag captured 1169 messages over about 29.4 seconds.

## Note

Gazebo's factory spawn service did not become available in this WSL session, even though the factory plugin was loaded. To complete the Nav2 verification path, `fake_turtlebot_sim` supplied `/odom`, `/scan`, and `odom -> base_footprint` TF while the official Nav2 launch, map server, AMCL, planner, controller, BT navigator, and RViz were running.

