#!/usr/bin/env python3

import math
import os
import signal
import sys
import time
import yaml

from geometry_msgs.msg import PoseStamped, Quaternion
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class WaypointFollower(Node):
    """ROS2 node that navigates a TurtleBot3 through a sequence of waypoints
    using Nav2's Simple Commander API (BasicNavigator).
    """

    def __init__(self):
        super().__init__('waypoint_follower')

        # ---- Declare parameters ----
        self.declare_parameter('waypoints_file', 'waypoints.yaml')
        self.declare_parameter('set_initial_pose', False)
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('frame_id', 'map')

        self.shutdown_requested = False

        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # Initialize the BasicNavigator
        self.get_logger().info('Initializing BasicNavigator...')
        self.navigator = BasicNavigator()

        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active.')

        # Load waypoints (from YAML or hardcoded fallback)
        self.waypoints = self._load_waypoints()
        if not self.waypoints:
            self.get_logger().error('No waypoints available. Exiting.')
            sys.exit(1)

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')

        # Optionally set the initial pose
        if self.get_parameter('set_initial_pose').value:
            self._set_initial_pose()

    # ------------------------------------------------------------------
    # Signal handling
    # ------------------------------------------------------------------

    def _signal_handler(self, signum, frame):
        """Set the shutdown flag when SIGINT/SIGTERM is received."""
        self.get_logger().info('Shutdown signal received. Requesting graceful stop.')
        self.shutdown_requested = True

    # ------------------------------------------------------------------
    # Pose helpers
    # ------------------------------------------------------------------

    def _yaw_to_quaternion(self, yaw):
        """Convert a yaw angle (radians) to a geometry_msgs/Quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = float(math.sin(yaw / 2.0))
        q.w = float(math.cos(yaw / 2.0))
        return q

    def _create_pose_stamped(self, x, y, yaw, frame_id=None):
        """Build a PoseStamped message for a single waypoint."""
        if frame_id is None:
            frame_id = self.get_parameter('frame_id').value

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = self._yaw_to_quaternion(yaw)
        return pose

    # ------------------------------------------------------------------
    # Waypoint loading
    # ------------------------------------------------------------------

    def _resolve_file_path(self, filename):
        """Try to resolve *filename* to an absolute path.

        Search order:
          1. Absolute path (returned as-is)
          2. Package share directory (ament_index)
          3. Current working directory
          4. Give up  => return None
        """
        if os.path.isabs(filename):
            return filename

        # Try inside the package's share directory
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('turtlebot3_nav2_demo')
            candidate = os.path.join(share_dir, filename)
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass

        # Try relative to the current working directory
        candidate = os.path.join(os.getcwd(), filename)
        if os.path.exists(candidate):
            return candidate

        return None

    def _load_waypoints(self):
        """Return a list of (x, y, yaw) tuples.

        Attempts to load from the YAML file given by the 'waypoints_file'
        parameter.  On any failure the method falls back to a small set of
        hardcoded waypoints.
        """
        waypoints_file_param = self.get_parameter('waypoints_file').value
        resolved = self._resolve_file_path(waypoints_file_param)

        if resolved:
            try:
                with open(resolved, 'r') as f:
                    data = yaml.safe_load(f)
                raw = data.get('waypoints', [])
                if raw:
                    waypoints = [
                        (wp['x'], wp['y'], wp.get('yaw', 0.0)) for wp in raw
                    ]
                    self.get_logger().info(
                        f'Loaded {len(waypoints)} waypoints from {resolved}'
                    )
                    return waypoints
            except Exception as e:
                self.get_logger().warn(f'Failed to parse {resolved}: {e}')

        # ---- Hardcoded fallback waypoints ----
        self.get_logger().warn(
            f'Could not load waypoints file. Using hardcoded fallback waypoints.'
        )
        return [
            (1.0, 0.0, 0.0),
            (2.0, 0.5, math.radians(30)),
            (3.0, 0.0, 0.0),
            (2.0, -0.5, math.radians(-30)),
            (1.0, 0.0, math.radians(180)),
        ]

    # ------------------------------------------------------------------
    # Initial pose
    # ------------------------------------------------------------------

    def _set_initial_pose(self):
        """Publish the initial pose to AMCL using the node parameters."""
        x = self.get_parameter('initial_pose_x').value
        y = self.get_parameter('initial_pose_y').value
        yaw = self.get_parameter('initial_pose_yaw').value

        initial_pose = self._create_pose_stamped(x, y, yaw)
        self.get_logger().info(
            f'Setting initial pose: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg'
        )
        self.navigator.setInitialPose(initial_pose)

        # Give AMCL a moment to converge on the new pose estimate
        time.sleep(2.0)

    # ------------------------------------------------------------------
    # Navigation mission
    # ------------------------------------------------------------------

    def _get_eta_seconds(self, feedback):
        """Extract estimated time remaining (seconds) from navigation feedback.

        Returns -1.0 if the feedback does not contain a valid ETA.
        """
        try:
            eta = feedback.estimated_time_remaining
            return eta.sec + eta.nanosec * 1e-9
        except AttributeError:
            return -1.0

    def navigate_waypoints(self):
        """Drive the robot through every waypoint in sequence.

        For each waypoint the node will:
          - attempt navigation up to *max_retries* times,
          - log progress feedback (distance remaining, ETA),
          - clear costmaps between retries,
          - skip the waypoint after exhausting all retries.
        """
        frame_id = self.get_parameter('frame_id').value
        max_retries = self.get_parameter('max_retries').value
        total = len(self.waypoints)

        for idx, (wx, wy, wyaw) in enumerate(self.waypoints):
            if self.shutdown_requested:
                self.get_logger().info('Shutdown requested. Aborting mission.')
                break

            self.get_logger().info(
                f'--- Waypoint {idx + 1}/{total}: '
                f'({wx:.2f}, {wy:.2f}, {math.degrees(wyaw):.1f} deg) ---'
            )

            goal_pose = self._create_pose_stamped(wx, wy, wyaw, frame_id)
            reached = False

            for attempt in range(1, max_retries + 1):
                if self.shutdown_requested:
                    break

                self.get_logger().info(f'  Attempt {attempt}/{max_retries}')

                # Send the goal to Nav2
                self.navigator.goToPose(goal_pose)

                # Monitor feedback until the task finishes or shutdown is requested
                while not self.navigator.isTaskComplete():
                    if self.shutdown_requested:
                        self.navigator.cancelTask()
                        break

                    feedback = self.navigator.getFeedback()
                    if feedback:
                        dist = feedback.distance_remaining
                        eta = self._get_eta_seconds(feedback)
                        self.get_logger().info(
                            f'    Waypoint {idx + 1}: '
                            f'distance remaining = {dist:.2f} m, '
                            f'ETA = {eta:.1f} s',
                            throttle_duration_sec=2.0,
                        )
                    time.sleep(0.1)

                if self.shutdown_requested:
                    break

                # Evaluate the outcome of this attempt
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(
                        f'  Waypoint {idx + 1} reached successfully!'
                    )
                    reached = True
                    break
                elif result == TaskResult.CANCELED:
                    self.get_logger().warn(
                        f'  Waypoint {idx + 1} navigation was canceled.'
                    )
                else:
                    self.get_logger().warn(
                        f'  Waypoint {idx + 1} navigation failed.'
                    )

                # Between retries: clear costmaps to help the planner recover
                if attempt < max_retries and not self.shutdown_requested:
                    self.get_logger().info('  Clearing costmaps before retry...')
                    self.navigator.clearAllCostmaps()
                    time.sleep(1.0)

            if not reached:
                self.get_logger().error(
                    f'  Waypoint {idx + 1} failed after {max_retries} '
                    f'attempt(s). Skipping to the next waypoint.'
                )

            # Brief pause between waypoints
            time.sleep(0.5)

        self.get_logger().info('Waypoint mission complete.')

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def shutdown(self):
        """Cancel any active task, shut down Nav2 lifecycle nodes, and
        destroy this ROS2 node."""
        self.get_logger().info('Shutting down waypoint follower...')
        try:
            self.navigator.cancelTask()
        except Exception:
            pass
        try:
            self.navigator.lifecycleShutdown()
        except Exception:
            pass
        self.destroy_node()


# ======================================================================
# Entry point
# ======================================================================

def main(args=None):
    rclpy.init(args=args)

    follower = WaypointFollower()

    try:
        follower.navigate_waypoints()
    except KeyboardInterrupt:
        follower.get_logger().info('KeyboardInterrupt received.')
    except Exception as e:
        follower.get_logger().error(f'Unexpected error during navigation: {e}')
    finally:
        follower.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
