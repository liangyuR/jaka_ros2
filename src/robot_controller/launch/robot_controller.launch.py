#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # 声明 'ip' 和 'model' 参数
        DeclareLaunchArgument('ip', default_value='192.168.0.122', description='Robot IP address'),
        DeclareLaunchArgument('model', default_value='zu12', description='Robot model name'),

        # 启动 'robot_controller_node' 节点
        Node(
            package='robot_controller',
            executable='robot_controller_node',
            name='robot_controller',
            output='screen',
            parameters=[
                {'robot_ip': LaunchConfiguration('ip')},  # 传递 'ip' 参数
                {'robot_model': LaunchConfiguration('model')}  # 传递 'model' 参数
            ],
        ),
    ]) 