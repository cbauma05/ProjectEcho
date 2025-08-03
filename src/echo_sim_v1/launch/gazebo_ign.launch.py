from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gz_sim_pkg = FindPackageShare('ros_gz_sim')
    world_path = PathJoinSubstitution([
        FindPackageShare('echo_sim_v1'),
        'worlds',
        'empty.sdf'  # make sure you have an SDF world
    ])

    return LaunchDescription([
        # Start Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gz_sim_pkg, 'launch', 'gz_sim.launch.py'])
            ),
            #launch_arguments={'gz_args': ['-r', world_path]}.items() #original
            launch_arguments={'gz_args': f'-r {world_path}'}.items()
        ),

        # Include robot spawn launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('echo_sim_v1'),
                    'launch',
                    'robot_spawn_ign.launch.py'
                ])
            )
        )
    ])
