import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# How to run (where input can be True, False or an absolute path).
# $ ros2 launch echo_sim_v1 echo_v1_rviz.launch.py <launch-argument>:=<input>
#
# Examples:
#   $ ros2 launch echo_sim_v1 echo_v1_rviz.launch.py gui:=False
#   $ ros2 launch echo_sim_v1 echo_v1_rviz.launch.py gui:=True urdf_model:=/home/vboxuser/Desktop/amr_ws/src/echo_sim_v1/urdf/turtlebot3_waffle.urdf

def generate_launch_description():
    # Define environment variables
    pkg_share = FindPackageShare(package='echo_sim_v1').find('echo_sim_v1')                 # Set the path to this package.
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/echo_v1.urdf.xacro')        # Set the path to the xacro file
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'echo_v1_gazebo.rviz')            # Set the path to the RViz configuration settings

    # Define launch arguments that can entered into the terminal.
    declare_urdf_model_path_cmd = DeclareLaunchArgument(    
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(   
       name='rviz_config_file',
       default_value=default_rviz_config_path, 
       description='Full path to the RVIZ config file to use')

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(  
        name='gui',
        default_value='False',
        description='Flag to enable joint_state_publisher_gui')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(    
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation clock if true')

    
    # Store users inputs into variables corresponding to previously defined arguments.
    gui = LaunchConfiguration('gui')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    #Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # A GUI to manipulate the joint state values
    start_joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(Command(['xacro ', urdf_model]), value_type=str) # Parameters within robot_state_publisher package 
        }],
        arguments=[default_urdf_model_path] # could potentially remove this line
    )

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Run execute nodes with launch options.
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_node)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
