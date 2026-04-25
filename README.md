# rt2_assignment

This repository provides a ROS2 navigation framework enabling target-based robot motion in simulation. 
It contains three ROS2 packages:

- The simulation environment (GAZEBO/RVIZ)
- Assigment nodes that allow to perform the requested tasks
- Custom interfaces that contains messages, services and actions crated 

---

## 1. Package Assignment Overview

The package `assignment` contains:

- `node1`: **nav_interface**  
  This node provides a simple user interface that allows the user to set a target pose (x, y, θ) for the robot and to cancel an active goal.

- `node2`: **nav_client**  
  This node acts as an action client, publishing goal or cancel requests and receiving continuous feedback from the navigation server.

- `node3`: **nav_server**  
  This node implements the action server responsible for moving the robot toward the desired target pose, managing the navigation logic and providing feedback during execution.


---

## 2. Directory Structure

```bash
.assignment/
├── CMakeLists.txt  
├── package.xml        
├── launch/  
│   └── launch.py
├── scripts/  
│   └── nav_interface.py
└── src/  
    ├── nav_client.cpp
    └── nav_server.cpp
bme_gazebo_sensors/  
custom_interfaces/  

---
## 3. Installation

Clone this repository into your ROS2 workspace, inside the /src folder:

```bash
cd ~/ws/src
git clone https://github.com/Pedemontemarina/rt2_assignment.git 
cd ..
colcon build 
```
Source your workspace:
```bash
source install/local_setup.bash
```
---
## 4. Usage

A ROS2 launch file has been created to automatically start the simulation environment together with all the nodes required for the assignment:

```bash
ros2 launch assignment launch.py

```
Once launched, the Gazebo/RViz simulation will start and a terminal interface will appear, allowing the user to send commands to the robot.
The communication between the action client and the action server is fully visible in the same terminal where the launch file is executed, showing goal dispatching, feedback updates, and result messages.

---
## 5. Nodes Description 

**Nav_interface Node**

The **nav_interface** is a Python node responsible for managing user interaction and forwarding commands to the navigation system.  
It runs an internal loop that continuously presents a menu with two options:

- **1 — Send a goal**  
- **2 — Cancel the current goal**

When the user selects **option 1**, the node requests three parameters: **x**, **y**, and **θ** (in degrees).  
Since the robot operates in a planar environment, these values fully define the target pose.  
The orientation is converted into a quaternion, and a `PoseStamped` message is published on the `/goal_pose` topic.

If the user selects **option 2**, the node publishes an `Empty()` message on the `/cancel_goal` topic to request cancellation of the current goal.

To ensure consistent behavior, if a goal is already active and the user attempts to send a new one, the node asks whether the existing one should be overwritten before proceeding.

In addition to handling user commands, the node also subscribes to the action status topic and detects when the navigation task has been completed, allowing the interface to notify the user when the robot reaches the target.

Both `/goal_pose` and `/cancel_goal` are monitored by the navigation action client, which forwards the requests to the action server.


**Nav_client Node**



**Nav_Server Node**


---
## 6. Requirements

ROS2
XLaunch (for graphical visualization)

---
## 7. Author

Pedemonte Marina

Research Track I – Assignment 2

