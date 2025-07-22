from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jaka_manipulation',
            executable='alson_client_node',
            name='alson_client_node',
            output='screen',
            parameters=[
                {'host': '127.0.0.1'},
                {'port': 59999}
            ]
        ),
        # 延迟启动测试节点，确保 alson_client_node 已启动
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='jaka_manipulation',
                    executable='alson_client_test_node.py',
                    name='alson_client_test_node',
                    output='screen',
                )
            ]
        )
    ])
