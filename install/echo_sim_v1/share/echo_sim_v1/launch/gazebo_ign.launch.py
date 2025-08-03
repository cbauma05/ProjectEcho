

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
    gz_sim_pkg = FindPackageShare('echo_sim_v1')

    world_path = PathJoinSubstitution([
        gz_sim_pkg,
        'worlds',
        'empty.sdf'  # must be an sdf file
    ])

    return LaunchDescription([
        # Start Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_path, '--force-version', '6'],
            output='screen'
        ),

        # Include robot spawn launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gz_sim_pkg, 'launch', 'robot_spawn_ign.launch.py'])
            )
        )
    ])
