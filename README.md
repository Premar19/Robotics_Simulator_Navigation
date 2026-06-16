
# Autonomous Corridor Navigation, Parking and Cube Localisation

## Overview
This ROS 2 package implements an autonomous robotic system for the Husky mobile robot.
The robot is capable of:

- Navigating safely along a corridor using LIDAR
- Detecting when it enters a room
- Searching for and parking fully on a green floor target
- Detecting a coloured cube on a table
- Reporting the cube position relative to the odom frame

The entire task is completed autonomously without any user input and is controlled using a finite state machine (FSM).

---

## Package Name
prs49_ros366_cpp

---

## Main Node
assignment.cpp

---

## How to Build
From your ROS 2 workspace:

colcon build --packages-select prs49_ros366_cpp

source install/setup.bash

## How to Run:
ros2 run prs49_ros366_cpp assignment --ros-args -r __ns:=/husky -r /tf:=tf -r /tf_static:=tf_static


