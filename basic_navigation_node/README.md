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
