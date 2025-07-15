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
    真实机器人操作演示
    """
    
    # 获取包路径
    jaka_zu12_moveit_config_share = get_package_share_directory("jaka_zu12_moveit_config")
    jaka_planner_share = get_package_share_directory("jaka_planner")
    
    # 构建 MoveIt 配置
    moveit_config = MoveItConfigsBuilder("jaka_zu12", package_name="jaka_zu12_moveit_config").to_moveit_configs()
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            "robot_ip", 
            default_value="192.168.1.100",
            description="JAKA robot IP address"
        ),
        
        DeclareLaunchArgument(
            "use_rviz", 
            default_value="true",
            description="Whether to start RViz"
        ),
        
        LogInfo(msg="=== Starting JAKA Real Robot Demo ==="),
        
        # 1. 启动 MoveIt server (真实机器人)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(jaka_planner_share, "launch", "moveit_server.launch.py")
            ),
            launch_arguments={
                "ip": LaunchConfiguration("robot_ip"),
                "model": "zu12",
            }.items()
        ),
        
        # 2. 启动 robot_state_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(jaka_zu12_moveit_config_share, "launch", "rsp.launch.py")
            )
        ),
        
        # 3. 启动 move_group
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(jaka_zu12_moveit_config_share, "launch", "move_group.launch.py")
            )
        ),
        
        # 4. 启动 RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(jaka_zu12_moveit_config_share, "launch", "moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz"))
        ),
        
        # 5. 启动 manipulation node
        Node(
            package="jaka_manipulation",
            executable="manipulation_node",
            name="manipulation_node",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "robot_ip": LaunchConfiguration("robot_ip"),
                    "use_sim_time": False
                }
            ]
        ),
        
        LogInfo(msg="=== Real robot manipulation ready! ===")
    ])