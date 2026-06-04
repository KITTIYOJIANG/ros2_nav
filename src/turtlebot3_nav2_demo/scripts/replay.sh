#!/bin/bash
set -e

usage() {
    echo "Usage: $(basename "$0") [OPTIONS] BAG_DIR"
    echo ""
    echo "Replay a ROS2 bag with clock for TurtleBot3 Nav2."
    echo ""
    echo "Arguments:"
    echo "  BAG_DIR         Path to the recorded bag directory"
    echo ""
    echo "Options:"
    echo "  --no-loop       Play once without looping (default: loop)"
    echo ""
    echo "Example:"
    echo "  $(basename "$0") ~/ros2_bags/nav_20250101_120000"
    echo "  $(basename "$0") --no-loop ~/ros2_bags/nav_20250101_120000"
    echo ""
    echo "To visualize with RViz while replaying:"
    echo "  1. In a second terminal, launch your Nav2 bringup:"
    echo "     ros2 launch turtlebot3_nav2_demo nav2_bringup.launch.py use_sim_time:=true"
    echo "  2. Or run RViz directly:"
    echo "     rviz2 -d <path_to_config>.rviz"
}

LOOP=true
BAG_DIR=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --help|-h)
            usage
            exit 0
            ;;
        --no-loop)
            LOOP=false
            shift
            ;;
        -*)
            echo "ERROR: Unknown option: $1"
            echo ""
            usage
            exit 1
            ;;
        *)
            if [[ -z "$BAG_DIR" ]]; then
                BAG_DIR="$1"
            else
                echo "ERROR: Multiple bag directories provided."
                echo ""
                usage
                exit 1
            fi
            shift
            ;;
    esac
done

if [[ -z "$BAG_DIR" ]]; then
    echo "ERROR: No bag directory specified."
    echo ""
    usage
    exit 1
fi

if [[ ! -d "$BAG_DIR" ]]; then
    echo "ERROR: Bag directory does not exist: $BAG_DIR"
    exit 1
fi

if [[ ! -f "$BAG_DIR/metadata.yaml" ]]; then
    echo "ERROR: $BAG_DIR does not appear to be a valid ROS2 bag (missing metadata.yaml)."
    exit 1
fi

ROS2_BAG_ARGS=("play" "$BAG_DIR" "--clock")
if $LOOP; then
    ROS2_BAG_ARGS+=("--loop")
fi

echo "=============================================="
echo "  TurtleBot3 Nav2 Bag Replay"
echo "=============================================="
echo "Bag directory: $BAG_DIR"
echo "Loop mode:     $( $LOOP && echo "enabled" || echo "disabled" )"
echo ""
echo "Playing back bag with simulation clock..."
echo "Press Ctrl+C to stop."
echo ""
echo "--- RViz Instructions ---"
echo "Run in another terminal to visualize:"
echo "  ros2 launch turtlebot3_nav2_demo nav2_bringup.launch.py use_sim_time:=true"
echo "  or: rviz2"
echo "=============================================="
echo ""

ros2 bag "${ROS2_BAG_ARGS[@]}"
