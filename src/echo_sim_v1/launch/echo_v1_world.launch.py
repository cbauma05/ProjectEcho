from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = FindPackageShare('echo_sim_v1')  # replace with your package name
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'echo_v1.sdf'])  # your sdf filename

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_path, '--force-version', '6'],
            output='screen'
        ),
    ])
