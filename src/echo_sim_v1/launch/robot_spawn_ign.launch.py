# robot_spawn_ign.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare('echo_sim_v1'),
        'urdf',
        'echo_v1.urdf.xacro'
    ])

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            name='create',
            arguments=[
                '-name', 'echo_v1',
                '-topic', 'robot_description'
            ],
            output='screen'
        )
    ])
