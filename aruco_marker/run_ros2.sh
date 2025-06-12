#!/bin/bash
source install/setup.bash
colcon build --packages-select aruco_marker
source install/setup.bash
ros2 launch aruco_marker duo_nodes.launch.py
