# Frequently Asked Questions (FAQ)

## 1. Gazebo Crashes on Startup

**Symptom:** Gazebo freezes or crashes immediately when launching the simulation.

**Solution:**

- Ensure your GPU drivers are properly installed. Run `glxinfo | grep "OpenGL renderer"` to verify.
- Try launching Gazebo with software rendering:

  ```bash
  export LIBGL_ALWAYS_SOFTWARE=1
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  ```

- If using WSL or a VM, software rendering is often required. Add `LIBGL_ALWAYS_SOFTWARE=1` to your `.bashrc` for convenience.
- Kill any stale Gazebo processes: `killall gzserver gzclient`.

---

## 2. Robot Doesn't Move with Nav2

**Symptom:** Nav2 is running and a goal is set, but the TurtleBot3 does not move.

**Solution:**

- Check if the controller server is active:

  ```bash
  ros2 node list | grep controller_server
  ```

- Verify `/cmd_vel` is being published:

  ```bash
  ros2 topic echo /cmd_vel
  ```

  You should see non-zero `linear.x` and `angular.z` values when a goal is active.

- Confirm the robot is receiving velocity commands:

  ```bash
  ros2 topic echo /cmd_vel --once
  ```

- Ensure `use_sim_time:=true` is set in all launch commands.
- Check that the costmap is not blocking motion — inflated obstacle costs may prevent the local planner from finding a feasible path.

---

## 3. Map Doesn't Appear in RViz

**Symptom:** RViz shows a gray background with no map.

**Solution:**

- In RViz, under the **Displays** panel, check the **Fixed Frame** is set to `map`. If `map` is not available in the dropdown, the map TF is not being published.
- Verify the `/map` topic is publishing:

  ```bash
  ros2 topic echo /map --once
  ```

  You should see occupancy grid data. If the topic does not exist, the map server is not running.

- In RViz, click **Add** → **By topic** → select `/map` → **Map**. Also ensure the **Map** display has the correct **Topic** set to `/map`.

---

## 4. AMCL Doesn't Localize Properly

**Symptom:** The robot's estimated pose in RViz drifts or does not converge to the real pose.

**Solution:**

- Provide an accurate **initial pose estimate** using RViz. Click the **"2D Pose Estimate"** button and set the approximate position and orientation of the robot on the map. If the estimate is too far from the true pose, AMCL may fail to converge.
- Ensure the LiDAR is publishing scan data:

  ```bash
  ros2 topic echo /scan --once
  ```

- Check that the TF tree is complete. Run:

  ```bash
  ros2 run tf2_tools view_frames
  ```

  Look at the generated `frames.pdf` — you should see `map → odom → base_footprint → base_scan`.

- Increase AMCL particle count in `nav2_params.yaml` under `amcl` → `max_particles` for more robust localization at the cost of CPU.

---

## 5. TF Errors in Terminal

**Symptom:** Terminal floods with warnings like `"Lookup would require extrapolation..."` or `"No transform from [X] to [Y]"`.

**Solution:**

- Ensure `robot_state_publisher` is running:

  ```bash
  ros2 node list | grep robot_state_publisher
  ```

  If missing, launch it manually:

  ```bash
  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(xacro path/to/turtlebot3_burger.urdf.xacro)"
  ```

- Verify the URDF file path is correct and the model file exists.
- Run `ros2 run tf2_tools view_frames` to visualize and debug the full transform tree.
- Make sure all nodes share the same `use_sim_time` setting (all `true` or all `false`).

---

## 6. SLAM Toolbox Not Building a Map

**Symptom:** SLAM Toolbox is running but the map remains empty or does not update.

**Solution:**

- Verify that LiDAR scan data is being published:

  ```bash
  ros2 topic hz /scan
  ```

  A typical rate is ~5-10 Hz for TurtleBot3. If no messages appear, the Gazebo LiDAR plugin is not working.

- Confirm the `/scan` topic is connected:

  ```bash
  ros2 topic info /scan
  ```

  Note the publisher and subscriber counts.

- Move the robot slowly using `teleop_keyboard` to give SLAM enough feature data to correlate scans.
- Check the SLAM Toolbox parameters: `minimum_travel_distance` and `minimum_travel_heading` in `mapper_params_online_async.yaml` control how often a new node is added to the pose graph. Lower values create more nodes.

---

## 7. Navigation Fails Near Obstacles

**Symptom:** The robot plans a path but gets stuck or stops far away from walls and obstacles.

**Solution:**

- The **inflation layer** creates a cost gradient around obstacles. If the inflation radius is too large, the robot may avoid narrow passages entirely. Adjust in `nav2_params.yaml`:

  ```yaml
  local_costmap:
    ros__parameters:
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.35   # Reduce this value (default ~0.55m)
        cost_scaling_factor: 5.0 # Increase for sharper falloff
  ```

- Also check the global costmap's inflation settings under `global_costmap`.
- Ensure the **footprint** size in the costmap parameters matches the TurtleBot3 Burger dimensions (`[[-0.105, -0.105], [-0.105, 0.105], [0.105, 0.105], [0.105, -0.105]]`).

---

## 8. `ros2: command not found`

**Symptom:** After opening a new terminal, `ros2` commands are not recognized.

**Solution:**

- You need to source the ROS2 setup script in every new terminal:

  ```bash
  source /opt/ros/humble/setup.bash
  ```

- To avoid doing this manually, add it to your `.bashrc`:

  ```bash
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
  echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
  source ~/.bashrc
  ```

- If you are using a workspace, also source its setup:

  ```bash
  source ~/ros2-nav2-turtlebot3/install/setup.bash
  ```

---

## 9. Robot Spins in Place

**Symptom:** When a navigation goal is set, the robot rotates continuously without translating.

**Solution:**

- This usually indicates an **odometry drift** problem. Check the odometry topic:

  ```bash
  ros2 topic echo /odom
  ```

  Look for unrealistic jumps or NaN values in `pose.pose.position` or `twist.twist`.

- Reset the simulation by killing and restarting Gazebo.
- Verify that the `/odom` → `/base_footprint` transform is being published:

  ```bash
  ros2 run tf2_ros tf2_echo odom base_footprint
  ```

- If using AMCL, re-set the initial pose estimate (see FAQ #4). A poor initial estimate can cause the robot to spin while trying to localize.
- Reduce the local planner's `max_vel_theta` in `nav2_params.yaml` to limit rotation speed.

---

## 10. How to Add Custom Obstacles

**Symptom:** You want to test navigation against specific obstacles not present in the default world.

**Solution:**

- **Option A — Insert in Gazebo GUI:**
  - In the Gazebo window, click the **"Insert"** tab on the left panel.
  - Browse available models (cubes, cylinders, walls).
  - Click a model, then click in the simulation world to place it.
  - Obstacles are detected automatically via LiDAR — Nav2 costmaps will update.

- **Option B — Add to World File:**
  - Edit your `.world` file and add a `<model>` block inside `<world>`:

    ```xml
    <model name="my_box">
      <pose>2.0 1.0 0.1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.3 0.3 0.3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.3 0.3 0.3</size></box></geometry>
        </visual>
      </link>
    </model>
    ```

  - Relaunch Gazebo with the modified world file.
  - The new obstacle will appear in the LiDAR scans and costmaps automatically.

- Use `ros2 topic echo /local_costmap/costmap` to verify the costmap registers the new obstacle.
