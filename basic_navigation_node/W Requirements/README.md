# UR5e Navigation Package

This package provides navigation capabilities for the UR5e robot using ROS 2 Iron and MoveIt. It includes lifecycle management, service, and action implementations to navigate the robot's end effector to specified poses.

## Overview

The `ur5e_navigation` package consists of several key components:
- **Lifecycle Node**: Manages the node lifecycle states.
- **Service**: Provides a service to navigate the robot to a specified pose.
- **Action**: Implements an action server for handling long-running pose navigation tasks.

## Files

### 1. `launch_lifecycle_navigation.py`

This launch file starts the lifecycle navigation node. The lifecycle node manages the state of the node, ensuring it goes through proper lifecycle states (configure, activate, deactivate, cleanup, shutdown).

#### Usage

```bash
ros2 launch ur5e_navigation launch_lifecycle_navigation.py
```
### 2. launch_reach_pose_action.py

This launch file starts the reach pose action server. The action server handles long-running tasks such as reaching a specific pose.

Usage

```bash
ros2 launch ur5e_navigation launch_reach_pose_action.py
```
### 3. lifecycle_navigation_node.py

This file defines the UR5eLifecycleNode class, which is a ROS 2 Lifecycle Node. The node manages lifecycle states and provides a service to navigate the robot to a specified pose using MoveIt.

Key Components
Lifecycle Callbacks: Handles configuring, activating, deactivating, cleaning up, and shutting down the node.
Service: navigate_to_pose service that takes a target pose and navigates the robot to that pose using MoveIt.

### 4. reach_pose_action.py

This file defines the ReachPoseActionServer class, which implements a ROS 2 Action Server. The action server receives a target pose and uses MoveIt to plan and execute the motion to reach the specified pose.

Key Components
Action Server: Handles MoveGroup action requests to navigate the robot to the specified pose.
MoveIt Integration: Uses MoveIt to plan and execute the robot's motion.

```bash
source /opt/ros/iron/setup.bash
colcon build
source install/setup.bash
```

```bash
colcon build
source install/setup.bash
ros2 launch ur5e_navigation launch_lifecycle_navigation.py
ros2 launch ur5e_navigation launch_reach_pose_action.py
```
Launch Lifecycle Navigation Node
To launch the lifecycle navigation node, use the following command:

```bash
ros2 launch ur5e_navigation launch_lifecycle_navigation.py

```

Launch Reach Pose Action Server
To launch the reach pose action server, use the following command:

bash

```bash
ros2 launch ur5e_navigation launch_reach_pose_action.py

```

Interact with the Service
You can interact with the navigate_to_pose service using the following command:

```bash
ros2 service call /navigate_to_pose std_srvs/srv/Trigger

```


Interact with the Action Server
You can interact with the reach_pose action server using the following command:

```bash
ros2 action send_goal /reach_pose moveit_msgs/action/MoveGroup "{target_pose: {pose: {position: {x: 0.4, y: 0.1, z: 0.4}, orientation: {w: 1.0}}}}"
```
