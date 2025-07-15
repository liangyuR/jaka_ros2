import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    启动MoveIt演示和manipulation节点
    """
    
    # 获取包路径
    jaka_zu12_moveit_config_share = FindPackageShare("jaka_zu12_moveit_config")
    
    # 启动MoveIt演示
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            jaka_zu12_moveit_config_share, "/launch/demo.launch.py"
        ])
    )
    
    # 启动manipulation节点
    manipulation_node = Node(
        package="jaka_manipulation",
        executable="manipulation_node",
        name="manipulation_node",
        output="screen",
        parameters=[
            {"planning_group": "jaka_zu12"}
        ]
    )
    
    return LaunchDescription([
        moveit_demo,
        manipulation_node
    ])