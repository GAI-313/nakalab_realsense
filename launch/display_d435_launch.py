#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ld = LaunchDescription()


    default_xacro = os.path.join(
        get_package_share_directory('nakalab_realsense'),
        'urdf', 'd435.urdf.xacro'
    )


    declare_camera_name = DeclareLaunchArgument(
        'camera_name', default_value='d435',
        description='カメラ名'
    )

    declare_xacro_file = DeclareLaunchArgument(
        'xacro_file', default_value=default_xacro,
        description='デモ用カメラの Xacro モデルファイルの絶対パス'
    )
    ld.add_action(declare_xacro_file)
    ld.add_action(declare_camera_name)


    robot_description = ParameterValue(
        Command([
            'xacro ', LaunchConfiguration('xacro_file'),
            ' camera_name:=', LaunchConfiguration('camera_name')
        ]),
        value_type=str
    )


    node_robot_state_publishe = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description}
        ],
        output='screen',
            emulate_tty=True,
    )

    ld.add_action(node_robot_state_publishe)


    return ld
