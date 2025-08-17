from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gazebo_pkg = FindPackageShare('gazebo_ros')
    world_path = PathJoinSubstitution([
        FindPackageShare('echo_sim_v1'),
        'worlds',
        'empty.world'
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gazebo_pkg, 'launch', 'gazebo.launch.py'])
            ),
            launch_arguments={'world': world_path}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('echo_sim_v1'),
                    'launch',
                    'robot_spawn.launch.py'
                ])
            )
        )
    ])
