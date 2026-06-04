# ROS2 / Nav2 Demo Submission Summary

Run directory:
/home/zexu/ros2-nav2-turtlebot3/demo/nav2_official_tb3_20260604_115048

## Acceptance Mapping

- Nodes: nodes.txt
- Topics: topics.txt
- Actions: actions.txt
- Lifecycle states: lifecycle_after_initialpose.txt
- TF: runtime_tf_tree.png, runtime_tf_tree.pdf, tf_dynamic_after_goal.txt, tf_static_once.txt, frames_2026-06-04_12.02.32.pdf
- Map: map_turtlebot3_world.yaml, map_turtlebot3_world.pgm, map_turtlebot3_world.png, map_info.txt
- Navigation run: navigate_to_pose_goal.log, plan_once_after_goal.txt, local_plan_once_after_goal.txt, cmd_vel_once_after_goal.txt, cmd_vel_nav_once_after_goal.txt
- Local recording: nav2_goal_bag/, summarized by bag_info.txt
- Screenshot/evidence image: nav2_runtime_evidence.png

## Result

- ROS2 Humble environment found in WSL Ubuntu 22.04.
- Relevant project found at /home/zexu/ros2-nav2-turtlebot3.
- Nav2 lifecycle nodes reached active [3] after publishing initial pose.
- /navigate_to_pose goal was accepted.
- /plan, /local_plan, /cmd_vel, /cmd_vel_nav, /odom, /scan, /tf, /tf_static, /map, and /amcl_pose were recorded.
- ROS bag captured 1169 messages over about 29.4 seconds.

## Note

Gazebo's factory spawn service did not become available in this WSL session, even though the factory plugin was loaded. To complete the Nav2 verification path, fake_turtlebot_sim supplied /odom, /scan, and odom -> base_footprint TF while the official Nav2 launch, map server, AMCL, planner, controller, BT navigator, and RViz were running.
