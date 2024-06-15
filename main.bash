#in the beginning

source /opt/ros/iron/setup.bash
ros2 pkg create --build-type ament_python ur5e_navigation
cd ur5e_navigation

#in the end

colcon build
source install/setup.bash
ros2 launch ur5e_navigation ur5e_navigation_launch.py
