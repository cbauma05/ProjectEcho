from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Gazebo world path
    world_path = PathJoinSubstitution([
        FindPackageShare('echo_sim_v1'),
        'worlds',
        'obstacles.world'
    ])

    # RViz config path
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('echo_sim_v1'),
        'rviz',
        'echo.rviz'
    ])

    # Robot description via xacro
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare('echo_sim_v1'),
            'urdf',
            'echo_v1.urdf.xacro'
        ])
    ])

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'echo_gazebo.launch.py'
                ])
            ),
            launch_arguments={'world': world_path}.items()
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
            output='screen'
        ),

        # Spawn robot entity
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'echo_v1'
            ],
            output='screen'
        ),

        # Launch RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
