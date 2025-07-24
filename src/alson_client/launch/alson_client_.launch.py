from launch import LaunchDescription
from launch_ros.actions import Node

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
    ])
