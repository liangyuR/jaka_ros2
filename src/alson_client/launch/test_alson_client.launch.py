from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='alson_client',
            executable='alson_client_node',
            name='alson_client_node',
            output='screen',
            parameters=[
                {'host': '192.168.0.188'},
                {'port': 54600}
            ]
        ),
        # 延迟启动测试节点，确保 alson_client_node 已启动
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='alson_client',
                    executable='alson_client_test_node.py',
                    name='alson_client_test_node',
                    output='screen',
                )
            ]
        )
    ])
