from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotica_tarea',
            executable='part2_node',
            name='part2_node',
            output='screen',
            parameters=[{'gui': True, 'record': False}]
        )
    ])
