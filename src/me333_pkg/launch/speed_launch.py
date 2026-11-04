from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='me333_pkg',
            executable='speed_monitor',
            name='speed_monitor'
        ),
        Node(
            package='me333_pkg',
            executable='speed_sensor',
            name='speed_sensor'
        ),
    ])