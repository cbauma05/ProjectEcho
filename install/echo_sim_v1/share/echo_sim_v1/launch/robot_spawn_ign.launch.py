from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('echo_sim_v1'),
            'urdf',
            'echo_v1.urdf.xacro'
        ])
    ])

    return LaunchDescription([
        # Publish URDF to /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # Delay to ensure state publisher starts before spawning
        TimerAction(
            period=3.0,  # adjust if needed
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_entity',
                    output='screen',
                    arguments=[
                        '-name', 'echo_v1',
                        '-topic', 'robot_description',
                    ]
                )
            ]
        )
    ])
