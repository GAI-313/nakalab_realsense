#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ld = LaunchDescription()


    default_params = os.path.join(
        get_package_share_directory('nakalab_realsense'),
        'params', 'd435.yaml'
    )


    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='YAML ROS2 パラメータファイルの絶対パス'
    )
    declare_camera_name = DeclareLaunchArgument(
        'camera_name', default_value='d435',
        description='カメラ名'
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='',
        description='親名前空間'
    )

    ld.add_action(declare_params_file)
    ld.add_action(declare_camera_name)
    ld.add_action(declare_namespace)


    node_realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=LaunchConfiguration('camera_name'),
        parameters=[
            LaunchConfiguration('params_file'),
            {'camera_name': LaunchConfiguration('camera_name')}
        ],
        output='screen',
        emulate_tty=True,
        namespace=LaunchConfiguration('namespace')
    )

    ld.add_action(node_realsense)


    return ld
