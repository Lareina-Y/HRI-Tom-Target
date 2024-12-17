from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_catch_pkg',
            executable='robot_catch_node',
            name='robot_catch_game',
            output='screen',
        ),
    ])
