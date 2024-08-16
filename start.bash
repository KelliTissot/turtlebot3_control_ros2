#!/bin/bash
set -e
source /app/source.bash
cd /turtlebot3_ws
ros2 run turtlebot3_control_ros2 turtlebot_ctrl

