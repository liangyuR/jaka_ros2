from launch import LaunchDescription
from launch_ros.actions import Node

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

    # 启动alson_client_node
    alson_client_node = Node(
        package="jaka_manipulation",
        executable="alson_client_node",
        name="alson_client_node",
        output="screen",
        parameters=[
            {"host": "192.168.0.188"},
            {"port": 54600}
        ]
    )

    return LaunchDescription([
        # 启动节点
        manipulation_node,
        alson_client_node
    ])