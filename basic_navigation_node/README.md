# Basic Navigation Node

## Step 4: Create the Basic Navigation Node

Create a Python file named basic_navigation_node.py inside the ur5e_navigation/ur5e_navigation/ directory. This file will handle the navigation logic.

File: basic_navigation_node.py

This file contains the implementation of the basic navigation node. This node handles receiving a target pose via a ROS 2 service, plans the movement using MoveIt, and executes the motion to move the UR5e robot's gripper to the desired pose.

Purpose:
To provide a service (navigate_to_pose) that allows for external commands to navigate the robot to a specific pose.
To initialize the MoveIt components necessary for motion planning and execution.
To handle the motion planning and execution using MoveIt when the service is called.

Explanation:

BasicNavigationNode: A class inheriting from Node, representing the navigation node.

create_service: Creates a service named navigate_to_pose using the Trigger service type.

navigate_to_pose_callback: The callback function that handles the service request. It sets a target pose, plans the motion using MoveIt, and executes the plan.

MoveIt Components:
RobotCommander: Provides information about the robot.
PlanningSceneInterface: Represents the planning scene.
MoveGroupCommander: Interfaces with a planning group (in this case, the robot's manipulator).

## Step 5: Update Setup.py

Update setup.py to include the new node.

The setup script for the ur5e_navigation package. This script defines the package metadata and the installation details, including dependencies and entry points for executable scripts.

Purpose:
To provide metadata about the package (name, version, description, maintainer, etc.).
To define the package dependencies that need to be installed.
To specify the entry points for the executables included in the package.

## Step 6: Create a Launch File

Create a directory named launch in ur5e_navigation and create a file basic_navigation_launch.py.

File: basic_navigation_launch.py

This is the launch file used to start the basic navigation node. The launch file is responsible for initializing and running the basic_navigation_node, which handles the navigation logic.

Purpose:
To launch the basic_navigation_node and ensure it starts correctly.
To specify the package and executable details for launching the node.
To provide configuration for the node's name and output settings.

Explanation:
LaunchDescription: A container for the launch actions.
Node: Represents a ROS 2 node to be launched. Specifies the package name (ur5e_navigation), the executable (basic_navigation_node), and configures the output to be shown on the screen.

## Step 7: Build and Run

Build the package and run the launch file.

```bash
colcon build
source install/setup.bash
ros2 launch ur5e_navigation basic_navigation_launch.py
```
