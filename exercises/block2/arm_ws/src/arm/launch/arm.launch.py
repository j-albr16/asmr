
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # get package directory
    arm_pkg = get_package_share_directory('arm')

    # get file paths
    model_file = LaunchConfiguration('model')
    world_file = LaunchConfiguration('world')

    # create launch argument for the model file
    model_file_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(arm_pkg, 'models', 'arm.urdf.xacro'),
        description='Specify model file name'
    )
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Specify world file name'
    )

    # load the urdf file
    urdf_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            model_file,
            " ",
            "use_gazebo:=true",
        ]
    )

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_pkg, 'launch', 'world.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # spawn the robot
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', urdf_content,
                   '-name', 'arm robot',
                   '-allow_renaming', 'true',
                   "-x", "0", "-y", "0"],
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': ParameterValue(urdf_content, value_type=str)}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(arm_pkg, 'configs', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    RegisterEventHandler(
        OnProcessExit(
            target_action=ignition_spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                robot_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        world_file_arg,
        model_file_arg,
        gazebo,
        node_robot_state_publisher,
        bridge,
        ignition_spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ])


