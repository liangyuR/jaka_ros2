import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Gazebo 仿真演示
    """
    
    # 获取包路径
    jaka_zu12_moveit_config_share = get_package_share_directory("jaka_zu12_moveit_config")
    
    # 构建 MoveIt 配置
    moveit_config = MoveItConfigsBuilder("jaka_zu12", package_name="jaka_zu12_moveit_config").to_moveit_configs()
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            "use_rviz", 
            default_value="true",
            description="Whether to start RViz"
        ),
        
        LogInfo(msg="=== Starting JAKA Gazebo Demo ==="),
        
        # 1. 启动 Gazebo 仿真
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(jaka_zu12_moveit_config_share, "launch", "demo_gazebo.launch.py")
            ),
            launch_arguments={
                "use_gazebo": "true",
                "use_rviz": "false",  # 我们自己控制 RViz
            }.items()
        ),
        
        # 2. 启动 RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(jaka_zu12_moveit_config_share, "launch", "moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz"))
        ),
        
        # 3. 启动 manipulation node
        Node(
            package="jaka_manipulation",
            executable="manipulation_node",
            name="manipulation_node",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": True}
            ]
        ),
        
        LogInfo(msg="=== Gazebo manipulation ready! ===")
    ])