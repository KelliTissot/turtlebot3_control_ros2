#!/bin/bash
set -e
source /app/source.bash
cd /turtlebot3_ws

parallel --halt now,fail=1 --lb ::: \
    "ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py" \
    "ros2 run turtlebot3_control_ros2 turtlebot_ctrl"