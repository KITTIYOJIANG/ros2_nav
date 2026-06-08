#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "Usage: $(basename "$0") [OUTPUT_DIR]"
    echo ""
    echo "Run a real Gazebo Classic + TurtleBot3 + Nav2 navigation demo."
    echo ""
    echo "Environment overrides:"
    echo "  ROS_DOMAIN_ID     Default: 52"
    echo "  TURTLEBOT3_MODEL  Default: waffle"
    echo "  GOAL_X, GOAL_Y    Default: 0.6, 0.4"
    echo "  KEEP_RUNNING=1    Keep Gazebo/Nav2 alive after the goal finishes"
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

if [[ $# -gt 1 ]]; then
    usage
    exit 1
fi

source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-52}"
export GAZEBO_MODEL_PATH="/opt/ros/humble/share/turtlebot3_gazebo/models:/opt/ros/humble/share/nav2_bringup/worlds:${GAZEBO_MODEL_PATH:-}"

OUTPUT_DIR="${1:-$HOME/ros2-nav2-real-gazebo-$(date +%Y%m%d_%H%M%S)}"
MAP_FILE="/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml"
PARAMS_FILE="/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml"
GOAL_X="${GOAL_X:-0.6}"
GOAL_Y="${GOAL_Y:-0.4}"
mkdir -p "$OUTPUT_DIR"

cleanup() {
    set +e
    if [[ "${KEEP_RUNNING:-0}" != "1" ]]; then
        [[ -n "${BAG_PID:-}" ]] && kill -INT "$BAG_PID" 2>/dev/null || true
        sleep 2
        [[ -n "${NAV_PID:-}" ]] && kill "$NAV_PID" 2>/dev/null || true
        [[ -n "${GZ_PID:-}" ]] && kill "$GZ_PID" 2>/dev/null || true
        pkill -f "ros2 launch nav2_bringup" 2>/dev/null || true
        pkill -f "ros2 launch turtlebot3_gazebo" 2>/dev/null || true
        pkill -f gzserver 2>/dev/null || true
        pkill -f gzclient 2>/dev/null || true
    fi
}
trap cleanup EXIT

echo "Output directory: $OUTPUT_DIR"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"

ros2 daemon stop >/dev/null 2>&1 || true

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py \
    > "$OUTPUT_DIR/gazebo_launch.log" 2>&1 &
GZ_PID=$!

echo "Waiting for Gazebo /odom and /scan..."
for _ in $(seq 1 60); do
    ros2 topic list > "$OUTPUT_DIR/topics_gazebo_wait.txt" 2>/dev/null || true
    if grep -qx "/odom" "$OUTPUT_DIR/topics_gazebo_wait.txt" && grep -qx "/scan" "$OUTPUT_DIR/topics_gazebo_wait.txt"; then
        break
    fi
    sleep 1
done
timeout 10 ros2 topic echo /odom --once > "$OUTPUT_DIR/odom_once_gazebo.txt" 2>&1
timeout 10 ros2 topic echo /scan --once > "$OUTPUT_DIR/scan_once_gazebo.txt" 2>&1

ros2 launch nav2_bringup bringup_launch.py \
    map:="$MAP_FILE" \
    params_file:="$PARAMS_FILE" \
    use_sim_time:=true \
    use_composition:=False \
    autostart:=True \
    > "$OUTPUT_DIR/nav2_launch.log" 2>&1 &
NAV_PID=$!

echo "Waiting for Nav2 action server..."
for _ in $(seq 1 80); do
    ros2 action list > "$OUTPUT_DIR/actions_wait.txt" 2>/dev/null || true
    if grep -qx "/navigate_to_pose" "$OUTPUT_DIR/actions_wait.txt"; then
        break
    fi
    sleep 1
done

INITIAL_POSE='{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}'
for _ in $(seq 1 5); do
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$INITIAL_POSE" \
        >> "$OUTPUT_DIR/initialpose_pub.log" 2>&1
    sleep 1
done
timeout 10 ros2 topic echo /amcl_pose --once > "$OUTPUT_DIR/amcl_pose_once.txt" 2>&1

ros2 bag record -o "$OUTPUT_DIR/real_nav2_bag" \
    /clock /odom /scan /tf /tf_static /cmd_vel /plan /map /amcl_pose /particle_cloud \
    > "$OUTPUT_DIR/bag_record.log" 2>&1 &
BAG_PID=$!

GOAL="{pose: {header: {frame_id: map}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
timeout 90 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$GOAL" --feedback \
    > "$OUTPUT_DIR/navigate_to_pose_goal.log" 2>&1

sleep 3
kill -INT "$BAG_PID" 2>/dev/null || true
sleep 4

ros2 node list | sort > "$OUTPUT_DIR/nodes_after_goal.txt" 2>&1
ros2 topic list | sort > "$OUTPUT_DIR/topics_after_goal.txt" 2>&1
ros2 action list | sort > "$OUTPUT_DIR/actions_after_goal.txt" 2>&1
ros2 bag info "$OUTPUT_DIR/real_nav2_bag" > "$OUTPUT_DIR/real_nav2_bag_info.txt" 2>&1
tail -n 160 "$OUTPUT_DIR/navigate_to_pose_goal.log" > "$OUTPUT_DIR/navigate_to_pose_goal_result_tail.txt"

echo ""
echo "Done. Key files:"
echo "  $OUTPUT_DIR/navigate_to_pose_goal_result_tail.txt"
echo "  $OUTPUT_DIR/real_nav2_bag_info.txt"
echo "  $OUTPUT_DIR/real_nav2_bag/"
