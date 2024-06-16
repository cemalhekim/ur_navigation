# Basic Navigation Node

## Step 1: Set Up Your ROS 2 Environment

First, ensure you have ROS 2 Iron and MoveIt installed on your system. Follow the official ROS 2 installation guide and MoveIt installation instructions.

## Step 2: Create a New ROS 2 Package

Create a new package for your project. We'll name it ur5e_navigation.

```bash
source /opt/ros/iron/setup.bash
ros2 pkg create --build-type ament_python ur5e_navigation
cd ur5e_navigation
```

## Step 3: Define Dependencies

Edit the package.xml file to include the necessary dependencies

## Step 4: Create the Basic Navigation Node

Create a Python file named basic_navigation_node.py inside the ur5e_navigation/ur5e_navigation/ directory. This file will handle the navigation logic.

File: basic_navigation_node.py

## Step 5: Update Setup.py

Update setup.py to include the new node.

## Step 6: Create a Launch File

Create a directory named launch in ur5e_navigation and create a file basic_navigation_launch.py.

File: basic_navigation_launch.py

## Step 7: Build and Run

Build the package and run the launch file.

```bash
colcon build
source install/setup.bash
ros2 launch ur5e_navigation basic_navigation_launch.py
```
