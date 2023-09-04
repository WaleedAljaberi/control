from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pid',
            executable='control',
            name='control',
            output='screen'
        ),
        Node(
            package='pid',
            executable='plot',
            name='plot'
        )
    ])