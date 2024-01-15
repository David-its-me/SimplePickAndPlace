# SimplePickAndPlace
This Repository contains a simple scripts that performes a very simple pick and place task. For that it publishes some target poses to moveit. For opening and closing the gripper, the script directly sends commands to the io ports of a universal robot.

The positions for picking/aproaching an item are hardcoded into [/moveit2_scripts/src/pick_and_place.cpp](/moveit2_scripts/src/pick_and_place.cpp).

## Installation

Please make shure that the following ROS nodes already installed an running:
- Moveit
- UR_Driver

Install this package:
```
cd YOUR_ROS2_WS/src
git clone git@github.com:David-its-me/SimplePickAndPlace.git
cd YOUR_ROS2_WS
colcon build
```

## Run the package
```
ros2 launch moveit2_scripts pick_and_place.launch.py  
```
