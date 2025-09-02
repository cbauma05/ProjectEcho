from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_pkg = FindPackageShare('gazebo_ros')
    echo_pkg = FindPackageShare('echo_sim_v1')

    # Defaults (can be overridden from the terminal)
    world_default = PathJoinSubstitution([echo_pkg, 'worlds', 'empty.world'])
    rviz_default = PathJoinSubstitution([echo_pkg, 'rviz', 'echo.rviz'])

    # Launch-time overrides
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_default,
        description='Path to the Gazebo .world file (absolute path recommended).')

    rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_default,
        description='Path to an RViz2 .rviz config file.')

    # Gazebo (Classic) with a specified world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_pkg, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Delay the spawn a bit so the world is definitely up
    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([echo_pkg, 'launch', 'robot_spawn.launch.py'])
                )
            )
        ]
    )

    # RViz2 with provided config
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Keyboard teleop in a separate terminal (requires xterm)
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',  # install with: sudo apt install xterm
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        rviz_arg,
        gazebo,
        spawn_robot,
        rviz,
        teleop,
    ])
