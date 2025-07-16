import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    启动manipulation节点
    """
    
    # 启动manipulation节点
    manipulation_node = Node(
        package="jaka_manipulation",
        executable="manipulation_node",
        name="manipulation_node",
        output="screen",
        parameters=[
            {"planning_group": "jaka_zu12"},
            {"use_sim_time": True}
        ]
    )
    
    return LaunchDescription([
        # 启动节点
        manipulation_node
    ])