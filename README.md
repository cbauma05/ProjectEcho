# ProjectEcho
Collection of ROS2 and Gazebo packages that will be used for an AMR simulation.

TODO:
- Implement obstacles in gazebo world and have them displayed in rviz
- Setup LIDAR functionality in gazebo world and publish data to create map in RVIZ

HOLDUPS:
- ibignition-gazebo6-keyboard-control-system was unavailable for jammy jellyfish
- ign odom plugin was unavailble
- For these reasons, ros2 control was unable to be established and the robot model from gazebo using the ign native keypress plugin was not able to be published for rviz to see.

