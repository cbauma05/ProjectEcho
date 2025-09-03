# Project Echo

## Overview
Project Echo is a ROS 2-based robotics simulation project featuring a 3D-modeled robot integrated with Gazebo Classic and RViz for realistic simulation of perception and motion. The robot is equipped with a LiDAR scanner, enabling mapping with SLAM Toolbox and autonomous navigation using the Nav2 stack. This project showcases a complete simulation workflow, including teleoperation via keyboard control, SLAM-based mapping, and goal-directed navigation.

I developed this project to serve as a learning platform for:
- 3D robot modelling/urdf and xacro development
- Structuring effective launch files and ROS folder structure
- Integration of various ROS - Gazebo plugins for control methods and sensor perception
- Simulation of sensors for mapping and localization


<table>
  <tr>
    <td><img src="assets/Screenshot from 2025-08-16 23-43-31.png" height = "700" width = "540"></td>
    <td><img src="assets/Screenshot from 2025-09-02 12-32-51.png" height = "700" width = "540"></td>
  </tr>
</table>


## Features
✅ 3D robot model visualized in RViz and Gazebo Classic

✅ LiDAR integration for environmental sensing

✅ SLAM Toolbox for real-time map building

✅ Nav2 for autonomous goal-based navigation

✅ Teleoperation via teleop_twist_keyboard

✅ Configurable launch system for simulation and RViz

## Dependencies
Please ensure you have installed the necessary dependancies before getting started.
- Ubuntu 22.04
- ROS 2 Humble Hawksbill
- gazebo_ros
- xacro
- Slam ToolBox
- Nav2 Stack
- teleop_twist_keyboard
- xterm

## Repository Structure
The following folders within the "echo_sim_v1" package are essential.
```plaintext
echo_sim_v1/
├── launch/           # Launch files (main, robot spawn, RViz)
├── urdf/             # Robot URDF and XACRO files
├── worlds/           # Gazebo world files
├── rviz/             # RViz configuration files
├── config/           # Navigation, SLAM, and sensor configs
├── meshes/           # 3D model meshes (STL)
└── README.md
```
## Getting Started
#### 1. Clone the Repo
```plaintext
git clone <your-repo-url>
cd echo_sim_v1
```

#### 2. Build Workspace
```plaintext
colcon build
source install/setup.bash
```
#### 3. Launch Simulation
Note: Although there are several launch files in the launch directory (for initial testing purposes), the one that will be used is "gazebo.launch.py" which inherently calls "robot_spawn.launch.py"
The command below starts gazebo, spawns the robot in the world specified, opens Rviz with the saved config settings for laserscan messages and map saving, and launches teleop control:
```plaintext
ros2 launch echo_sim_v1 gazebo.launch.py   world:=/home/cameron/Project_Echo/Educational/amr_ws/src/echo_sim_v1/worlds/obstacles_1.world use_sim_time:=true
```
In a new terminal window, source the terminal and run the following command. This will launch the slam_toolbox and enable mapping of the world with the Lidar sensor. You can drive the robot around manually and begin mapping the virtual environment. 
```plainttext
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/echo_sim_v1/config/mapper_params_online_async.yaml use_sim_time:=true
```
In one last terminal window, source the terminal and run this command. This will launch Nav2 and allow you to set positinal goals in Rviz and let Nav2 plan a path for the robot autonomously.
```plaintext
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

### Notes:
For teleop control, ensure your cursor is clicked on the xterm window before you use the keystrokes listed to control the robot.

