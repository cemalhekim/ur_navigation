# ur_navigation

## Step 1: Set Up Your ROS 2 Environment

First, ensure you have ROS 2 Iron and MoveIt installed on your system. Follow the official ROS 2 installation guide and MoveIt installation instructions.

## Step 2: Create a New ROS 2 Package

Create a new package for your project. We'll name it ur5e_navigation.

to the bash ->

source /opt/ros/iron/setup.bash
ros2 pkg create --build-type ament_python ur5e_navigation
cd ur5e_navigation

## Step 3: Define Dependencies

Edit the package.xml file to include the necessary dependencies

## Step 4: Create a Lifecycle Node

Create a directory for your nodes and implement a Lifecycle Node. This node will manage the lifecycle states of your application.

Create a file lifecycle_node.py inside ur5e_navigation/ur5e_navigation/.

## Step 5: Implement a Service

Now, let's implement a service that the node can provide. This service can be used to control the robot's movements.

Create a file navigation_service.py inside ur5e_navigation/ur5e_navigation/.

## Step 6: Implement an Action

Actions are used for long-running tasks. Let's implement an action for reaching a specific pose.

Create a file reach_pose_action.py inside ur5e_navigation/ur5e_navigation/.

## Step 7: Integrate Everything and Launch

Now, integrate everything in a launch file. Create a directory launch in ur5e_navigation and create a file ur5e_navigation_launch.py.

## Step 8: Build and Run

Make sure to add the necessary entry points in setup.py.

Finally, build the package ->

colcon build
source install/setup.bash
ros2 launch ur5e_navigation ur5e_navigation_launch.py
