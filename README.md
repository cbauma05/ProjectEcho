# ProjectEcho
Collection of ROS2 and Gazebo packages that will be used for an AMR simulation.
<<<<<<< HEAD
=======



LAUNCH FILES
-> echo_v1_rviz.launch.py
    This launches the echo robot model in rviz with options to enable the jointstate publisher gui and other parameters

-> echo_v1_world.launch.py
    This launches a simpler robot model in gazebo ignition with gazebo keystroke plugins enables mapped to arrow keys to move the robot.
    It also launches Rviz to visualize sensor data like point clouds from LIDAR

-> gazebo_ign.launch.py 
    This launches the echo robot model in gazebo ignition.
    It calls the robot_spawn_ign.launch.py file to launch the robot with robot description in an empty world.

-> gazebo.launch.py
    This launches the echo robot model in gazbeo classic.
    It calls the robot_spawn.launch.py file to launch the robot with robot description in an empty world.



To Launch robot in gazebo with custom world, rviz2, and teleop control run:
ros2 launch echo_sim_v1 gazebo.launch.py   world:=/home/cameron/Project_Echo/Educational/amr_ws/src/echo_sim_v1/worlds/obstacles.world



Important xacros are:

- echo_v1.urdf.xacro
- gazebo_control
- materials
- lidar
>>>>>>> 1b062ec (lots of changes)
