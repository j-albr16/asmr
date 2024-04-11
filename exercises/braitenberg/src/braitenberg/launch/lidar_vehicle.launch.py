import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # include gazebo braitenberg launch file
    braitenberg_dir = get_package_share_directory('braitenberg')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(braitenberg_dir, 'launch', 'gazebo_lidar.launch.py')),
    )

    # include lidar controller node here !

    return LaunchDescription([
        gz_sim,
        # add lidar controller node here
    ])



