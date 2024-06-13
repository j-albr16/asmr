import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    gz_sim = get_package_share_directory('ros_gz_sim')
    pid = get_package_share_directory('pid')

    # paths
    world_path = os.path.join(pid, 'worlds', 'park.sdf')
    tbot_path = os.path.join(pid, 'models', 'waffle.sdf')
    gz_sim_launch = os.path.join(gz_sim, 'launch', 'gz_sim.launch.py')
    bridge_config = os.path.join(pid, 'config', 'bridge.yaml')

    # launch gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gz_sim_launch
        ),
        launch_arguments={'gz_args': world_path}.items(),
    )

    # robot spawn
    tbot_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', tbot_path,
            '-name', 'tbot',
            '-allow-renaming', 'true',
            '-x', '-4', '-y', '-25',
            '-Y', '1.7'
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
        }],
        output='screen'
    )

    ld.add_action(gazebo)
    ld.add_action(tbot_spawn)
    ld.add_action(bridge)

    return ld

