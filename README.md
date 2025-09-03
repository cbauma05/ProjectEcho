# Project Echo

## Overview
Project Echo is a ROS 2-based robotics simulation project featuring a 3D-modeled robot integrated with Gazebo Classic and RViz for realistic simulation of perception and motion. The robot is equipped with a LiDAR scanner, enabling mapping with SLAM Toolbox and autonomous navigation using the Nav2 stack. This project showcases a complete simulation workflow, including teleoperation via keyboard control, SLAM-based mapping, and goal-directed navigation.

I developed this project to serve as a learning platform for:
- 3D robot modelling/urdf and xacro development\
- Structuring effective launch files and ROS folder structure
- Integration of various ROS - Gazebo plugins for control methods and sensor perception
- Simulation of sensors for mapping and localization

Example visualization in RViz

Features

✅ 3D robot model visualized in RViz and Gazebo Classic

✅ LiDAR integration for environmental sensing

✅ SLAM Toolbox for real-time map building

✅ Nav2 for autonomous goal-based navigation

✅ Teleoperation via teleop_twist_keyboard

✅ Configurable launch system for simulation and RViz

Dependencies

ROS 2 Humble

gazebo_ros

nav2_bringup

slam_toolbox

teleop_twist_keyboard

Make sure you have ROS 2 Humble installed and sourced before running the simulation.

Repository Structure
