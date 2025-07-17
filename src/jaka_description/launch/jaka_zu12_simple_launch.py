import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the URDF file path
    urdf_file_path = PathJoinSubstitution([FindPackageShare('jaka_description'), 'urdf', 'jaka_zu12_with_tool.urdf'])
    
    return LaunchDescription([
        # Start the robot_state_publisher node 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(Command(['cat ', urdf_file_path]), value_type=str)}],
        ),
        
        # Start joint_state_publisher_gui for interactive control
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),

        # Start rviz with the provided configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('jaka_description'), 'config', 'jaka_zu12_rviz2.rviz'])],
        ),
    ])