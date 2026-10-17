from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotica_tarea',
            executable='part1_node',
            name='part1_node',
            output='screen',
        )
    ])
