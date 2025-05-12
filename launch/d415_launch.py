#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ld = LaunchDescription()


    default_params = os.path.join(
        get_package_share_directory('nakalab_realsense'),
        'params', 'd415.yaml'
    )
    default_xacro = os.path.join(
        get_package_share_directory('nakalab_realsense'),
        'urdf', 'd415.urdf.xacro'
    )



    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='YAML ROS2 パラメータファイルの絶対パス'
    )
    declare_camera_name = DeclareLaunchArgument(
        'camera_name', default_value='d415',
        description='カメラ名'
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='',
        description='親名前空間'
    )
    declare_xacro_file = DeclareLaunchArgument(
        'xacro_file', default_value=default_xacro,
        description='デモ用カメラの Xacro モデルファイルの絶対パス'
    )
    declare_use_description = DeclareLaunchArgument(
        'use_description', default_value='False',
        description='デモ用のカメラモデルを使用し、Rviz2 を起動する'
    )

    ld.add_action(declare_params_file)
    ld.add_action(declare_camera_name)
    ld.add_action(declare_namespace)
    ld.add_action(declare_xacro_file)
    ld.add_action(declare_use_description)


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
    node_demo_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',
            os.path.join(
                get_package_share_directory('nakalab_realsense'),
                'rviz', 'd415.rviz'
            )
        ],
        condition=IfCondition(LaunchConfiguration('use_description'))
    )

    ld.add_action(node_realsense)
    ld.add_action(node_demo_rviz)


    launch_demo_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nakalab_realsense'),
                'launch', 'display_launch.py'
            )
        ]),
        launch_arguments={
            'xacro_file': LaunchConfiguration('xacro_file'),
            'camera_name': LaunchConfiguration('camera_name')
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_description'))
    )

    ld.add_action(launch_demo_description)


    return ld
