
import os
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription


def generate_launch_description():

    arm_pkg = get_package_share_directory('arm')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_pkg, 'launch', 'grabber.launch.py')))

    
    # add your node here

    return LaunchDescription([
        sim,
    ])
