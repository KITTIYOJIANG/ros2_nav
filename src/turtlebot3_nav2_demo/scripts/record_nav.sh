#!/bin/bash
set -e

usage() {
    echo "Usage: $(basename "$0") [OUTPUT_DIR]"
    echo ""
    echo "Record all key navigation topics for TurtleBot3 Nav2."
    echo ""
    echo "Arguments:"
    echo "  OUTPUT_DIR    Directory to save the ROS2 bag (optional)"
    echo "                Default: ~/ros2_bags/nav_YYYYMMDD_HHMMSS"
    echo ""
    echo "Topics recorded:"
    echo "  /cmd_vel         - Velocity commands"
    echo "  /odom             - Odometry"
    echo "  /scan             - Laser scan"
    echo "  /tf               - Transforms"
    echo "  /tf_static        - Static transforms"
    echo "  /plan             - Global plan"
    echo "  /local_plan       - Local plan"
    echo "  /amcl_pose        - AMCL estimated pose"
    echo "  /goal_pose        - Navigation goal"
    echo "  /robot_description - Robot model"
    echo ""
    echo "Press Ctrl+C to stop recording."
}

TOPICS=(
    /cmd_vel
    /odom
    /scan
    /tf
    /tf_static
    /plan
    /local_plan
    /amcl_pose
    /goal_pose
    /robot_description
)

if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    usage
    exit 0
fi

if [[ $# -gt 1 ]]; then
    echo "ERROR: Too many arguments."
    echo ""
    usage
    exit 1
fi

if [[ -n "$1" ]]; then
    BAG_DIR="$1"
else
    BAG_DIR="$HOME/ros2_bags/nav_$(date +%Y%m%d_%H%M%S)"
fi

mkdir -p "$BAG_DIR"

echo "=============================================="
echo "  TurtleBot3 Nav2 Bag Recording"
echo "=============================================="
echo "Output directory: $BAG_DIR"
echo "Topics:"
for topic in "${TOPICS[@]}"; do
    echo "  - $topic"
done
echo ""
echo "Starting ros2 bag record..."
echo "Press Ctrl+C to stop."
echo "=============================================="
echo ""

ros2 bag record -o "$BAG_DIR" "${TOPICS[@]}"

echo ""
echo "Recording stopped. Bag info:"
ros2 bag info "$BAG_DIR"
