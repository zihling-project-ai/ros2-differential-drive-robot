from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='me333_pkg',
            executable='counter_publisher',
            name='my_talker'
        ),
        Node(
            package='me333_pkg',
            executable='echo_subscriber',
            name='my_listener'
        ),
    ])