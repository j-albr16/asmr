import os
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # create launch argument for the world file
    world_file = LaunchConfiguration('world')
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Specify world file name'
    )

    # get package directory
    gz_pkg = get_package_share_directory('ros_gz_sim')

    # start gz sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_file}.items(),
    )

    return LaunchDescription([
        world_file_arg,
        gazebo
    ])
