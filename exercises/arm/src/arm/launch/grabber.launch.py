
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # get package directory
    arm_pkg = get_package_share_directory('arm')

    # get file paths
    world_file = os.path.join(arm_pkg, 'worlds', 'obstacle.sdf')
    model_file = os.path.join(arm_pkg, 'models', 'grabber.urdf.xacro')

    # start arm simulation
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_pkg, 'launch', 'arm.launch.py')),
        launch_arguments={'world': world_file, 'model': model_file}.items(),
    )

    return LaunchDescription([
        sim
    ])


